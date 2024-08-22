/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#if defined(CONFIG_CLOCK_CONTROL_NRF)
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#endif /* #if defined(CONFIG_CLOCK_CONTROL_NRF) */
#include <zephyr/drivers/timer/nrf_grtc_timer.h>
#include <zephyr/ipc/ipc_service.h>
#include <zephyr/logging/log.h>
#include <esb.h>

LOG_MODULE_REGISTER(esb_rad, 3);


/* Print cycle time, in microseconds */
#define PRINT_CYCLE     1000000


struct print_msg {
	uint32_t ipc_cnt;
	uint32_t drop_cnt;
};

#define IPC_MSG_REPORT          0
#define IPC_MSG_ERROR           1
#define IPC_MSG_TIMER           2

struct ipc_msg {
	uint32_t type;
	uint8_t report[4];
};


K_MSGQ_DEFINE(print_msgq,
	      sizeof(struct print_msg),
	      1,
	      sizeof(uint32_t));

K_MSGQ_DEFINE(ipc_msgq,
	      sizeof(struct ipc_msg),
	      CONFIG_ESB_RX_FIFO_SIZE,
	      sizeof(uint32_t));

static struct k_thread print_thread;

static K_THREAD_STACK_DEFINE(print_stack, 512);

static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17);

static int32_t timer_chan;
static uint64_t timer_tick;

static int timer_target_set(void);
void event_handler(struct esb_evt const *event)
{
	int err;

	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		break;
	case ESB_EVENT_TX_FAILED:
		break;
	case ESB_EVENT_RX_RECEIVED:
		while (!esb_read_rx_payload(&rx_payload)) {
			struct ipc_msg msg;

			if (rx_payload.length >= sizeof(msg.report)) {
				msg.type = IPC_MSG_REPORT;
				memcpy(msg.report,
				       rx_payload.data,
				       sizeof(msg.report));
				err = k_msgq_put(&ipc_msgq, &msg, K_NO_WAIT);
				if (err) {
				}
			}
		}
        break;
	}
}

#if defined(CONFIG_CLOCK_CONTROL_NRF)
static int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		LOG_ERR("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do {
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res) {
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
	} while (err);

	LOG_DBG("HF clock started");
	return 0;
}
#endif

int esb_initialize(void)
{
	int err;
	/* These are arbitrary default addresses. In end user products
	 * different addresses should be used for each set of devices.
	 */
	uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
	uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
	uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

	struct esb_config config = ESB_DEFAULT_CONFIG;

	config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.bitrate = ESB_BITRATE_4MBPS;
	config.mode = ESB_MODE_PRX;
	config.event_handler = event_handler;
	config.crc = ESB_CRC_24BIT;
	config.selective_auto_ack = true;
	if (IS_ENABLED(CONFIG_ESB_FAST_SWITCHING)) {
		config.use_fast_ramp_up = true;
	}

	err = esb_init(&config);
	if (err) {
		return err;
	}

	err = esb_set_base_address_0(base_addr_0);
	if (err) {
		return err;
	}

	err = esb_set_base_address_1(base_addr_1);
	if (err) {
		return err;
	}

	err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	if (err) {
		return err;
	}

	err = esb_set_rf_channel(80);
	if (err) {
		return err;
	}

	return 0;
}

static void timer_compare_handler(int32_t chan_id,
				  uint64_t expire_time,
				  void *user_data)
{
	timer_target_set();

	int err;
	struct ipc_msg msg;

	msg.type = IPC_MSG_TIMER;
	err = k_msgq_put(&ipc_msgq, &msg, K_NO_WAIT);
	if (err) {
		LOG_ERR("Cannot put TIMER count to message queue");
	}
}

static int timer_target_set(void)
{
	int err;

	timer_tick += PRINT_CYCLE; // TODO: convert microseconds to timer tick

	err = z_nrf_grtc_timer_set(timer_chan,
				   timer_tick,
				   timer_compare_handler,
				   NULL);

	return err;
}

static void print_main(void *p1, void *p2, void *p3)
{
	struct print_msg msg;

	/* Process message queue */
	while (!k_msgq_get(&print_msgq, &msg, K_FOREVER)) {
		LOG_INF("Sent %u IPC messages. Dropped %u.",
			msg.ipc_cnt, msg.drop_cnt);
	}
}

static atomic_t ipc_state = ATOMIC_INIT(0);
static struct ipc_ept ipc_ep;

static void ipc_ep_bound(void *priv)
{
	atomic_set_bit(&ipc_state, 0);
}

static void ipc_ep_recv(const void *data, size_t len, void *priv)
{
}

static const struct ipc_ept_cfg ipc_ep_cfg = {
	.name = "ep0",
	.cb = {
		.bound    = ipc_ep_bound,
		.received = ipc_ep_recv,
	},
};

int ipc_initialize(void)
{
	int err;
	const struct device *ipc0_instance;

	ipc0_instance = DEVICE_DT_GET(DT_NODELABEL(ipc0));

	err = ipc_service_open_instance(ipc0_instance);
	if (!err || err == -EALREADY) {
		err = ipc_service_register_endpoint(ipc0_instance,
						    &ipc_ep, &ipc_ep_cfg);
	}

	return err;
}

static void ipc_main(void)
{
	int err;
	struct ipc_msg msg;

	uint32_t ipc_msg_cnt = 0;
	uint32_t ipc_drop_cnt = 0;
	struct print_msg prt_msg;

	/* Process message queue */
	while (!k_msgq_get(&ipc_msgq, &msg, K_FOREVER)) {
		switch(msg.type) {
		case IPC_MSG_REPORT:
			err = ipc_service_send(&ipc_ep,
					       msg.report,
					       sizeof(msg.report));
			if (err == sizeof(msg.report)) {
				ipc_msg_cnt++;
			} else {
				ipc_drop_cnt++;
			}

			break;
		case IPC_MSG_TIMER:
			prt_msg.ipc_cnt = ipc_msg_cnt;
			ipc_msg_cnt = 0;
			prt_msg.drop_cnt = ipc_drop_cnt;
			ipc_drop_cnt = 0;
			err = k_msgq_put(&print_msgq, &prt_msg, K_NO_WAIT);
			if (err) {
				LOG_ERR("Cannot put PRX statistics to message queue");
			}
			break;
		default:
			break;
		}
	}
}

int main(void)
{
	int err;

	LOG_INF("OctoBurst central sample");

#if defined(CONFIG_CLOCK_CONTROL_NRF)
	err = clocks_start();
	if (err) {
		return 0;
	}
#endif
	err = esb_initialize();
	if (err) {
		LOG_ERR("ESB initialization failed (err %d)", err);
		return 0;
	}

	err = ipc_initialize();
	if (err) {
		LOG_ERR("IPC initialization failed, err %d", err);
		return 0;
	}

	k_thread_create(&print_thread,
			print_stack,
			K_THREAD_STACK_SIZEOF(print_stack),
			print_main,
			NULL,
			NULL,
			NULL,
			K_LOWEST_APPLICATION_THREAD_PRIO,
			0,
			K_NO_WAIT);

	timer_chan = z_nrf_grtc_timer_chan_alloc();
	if (timer_chan < 0) {
		LOG_ERR("Cannot get a timer channel");
		return 0;
	}

	LOG_INF("Initialization complete");

	LOG_INF("Print packet counts every %u.%02u seconds",
		PRINT_CYCLE / USEC_PER_SEC,
		(PRINT_CYCLE % USEC_PER_SEC) * 100 / USEC_PER_SEC);

	LOG_INF("Setting up for packet receiption");

	timer_tick = z_nrf_grtc_timer_read();
	timer_target_set();

	err = esb_start_rx();
	if (err) {
		LOG_ERR("ESB failed to start (err %d)", err);
		return 0;
	}

	ipc_main();

	LOG_WRN("main loop exited");

	/* return to idle thread */
	return 0;
}

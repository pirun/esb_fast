/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#if defined(CONFIG_CLOCK_CONTROL_NRF)
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#endif /* #if defined(CONFIG_CLOCK_CONTROL_NRF) */
#include <zephyr/drivers/timer/nrf_grtc_timer.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <nrf.h>
#include <esb.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <dk_buttons_and_leds.h>

LOG_MODULE_REGISTER(esb_ptx, CONFIG_ESB_PTX_APP_LOG_LEVEL);

struct main_msg {
	uint32_t type;
	uint32_t cnt;
#ifdef CONFIG_ESB_PTX_DATA_CHECK
	uint32_t cnt_err;
#endif
};

#define MAIN_MSG_TX             0
#define MAIN_MSG_TIMER          1
#define MAIN_MSG_TX_FAIL        2
#define MAIN_MSG_RX             3

struct print_msg {
	uint32_t tx_cnt;
	uint32_t tx_fail_cnt;
	uint32_t rx_cnt;
#ifdef CONFIG_ESB_PTX_DATA_CHECK
	uint32_t rx_error_cnt;
#endif
	int64_t time_cnt;
};

static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0, 0x01, 0x02, 0x03, 0x04);
K_MSGQ_DEFINE(main_msgq,
	      sizeof(struct main_msg),
	      8,
	      sizeof(uint32_t));

K_MSGQ_DEFINE(print_msgq,
	      sizeof(struct print_msg),
	      1,
	      sizeof(uint32_t));


static struct k_thread print_thread;

static K_THREAD_STACK_DEFINE(print_stack, 512);

#if CONFIG_ESB_PTX_DELAY > 0
static int32_t timer_chan;
static uint64_t timer_tick;

static int timer_target_set(void);
#endif
static const int8_t hid_xy[][2] = {
	{35,7}, {30,21}, {21,30}, {7,35},
	{-7,35}, {-21,30}, {-30,21}, {-35,7},
	{-35,-7}, {-30,-21}, {-21,-30}, {-7,-35},
	{7,-35}, {21,-30}, {30,-21}, {35,-7}
};

void event_handler(struct esb_evt const *event)
{
	struct main_msg msg;
	int err;

	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		msg.type = MAIN_MSG_TX;
		msg.cnt = 1;
		err = k_msgq_put(&main_msgq, &msg, K_NO_WAIT);
		if (err) {
			LOG_ERR("Cannot put TX done to message queue");
		}
		break;
	case ESB_EVENT_TX_FAILED:
		esb_start_tx();
		msg.type = MAIN_MSG_TX_FAIL;
		msg.cnt = 1;
		err = k_msgq_put(&main_msgq, &msg, K_NO_WAIT);
		if (err) {
			LOG_ERR("Cannot put TX fail to message queue");
		}
		break;
		break;
	case ESB_EVENT_RX_RECEIVED:
		msg.type = MAIN_MSG_RX;
		msg.cnt = 0;
#ifdef CONFIG_ESB_PTX_DATA_CHECK
		msg.cnt_err = 0;
#endif
		while (esb_read_rx_payload(&rx_payload) == 0) {
			msg.cnt++;

#ifdef CONFIG_ESB_PTX_DATA_CHECK
			if (verify_rx_payload_data(&rx_payload)) {
				msg.cnt_err++;
			}
#endif
		}
		err = k_msgq_put(&main_msgq, &msg, K_NO_WAIT);
		if (err) {
			LOG_ERR("Cannot put RX count to message queue");
		}
		break;
	}
}

#if defined(CONFIG_CLOCK_CONTROL_NRF)
int clocks_start(void)
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
#endif /* #if defined(CONFIG_CLOCK_CONTROL_NRF) */

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
	config.retransmit_delay = 600;
	config.retransmit_count = 400;
	config.bitrate = ESB_BITRATE_4MBPS;
	config.event_handler = event_handler;
	config.mode = ESB_MODE_PTX;
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

	err = esb_set_rf_channel(0);
	if (err) {
		return err;
	}

	return 0;
}

static void leds_update(uint8_t value)
{
	uint32_t leds_mask =
		(!(value % 8 > 0 && value % 8 <= 4) ? DK_LED1_MSK : 0) |
		(!(value % 8 > 1 && value % 8 <= 5) ? DK_LED2_MSK : 0) |
		(!(value % 8 > 2 && value % 8 <= 6) ? DK_LED3_MSK : 0) |
		(!(value % 8 > 3) ? DK_LED4_MSK : 0);

	dk_set_leds(leds_mask);
}
static void fill_tx_payload_data(struct esb_payload *payload)
{
	static uint8_t tx_data_next;

	payload->data[0] = 0;
	payload->data[1] = hid_xy[tx_data_next][0];
	payload->data[2] = hid_xy[tx_data_next][1];
	payload->data[3] = 0;

	tx_data_next = (tx_data_next + 1 < ARRAY_SIZE(hid_xy)) ? tx_data_next + 1 : 0;

}

static void fill_tx_fifo(void)
{
	int err;
	static bool tx_data_set;

	while (1) {
		if (!tx_data_set) {
			fill_tx_payload_data(&tx_payload);
		}

		err = esb_write_payload(&tx_payload);
		tx_data_set = (!err) ? false : true;
		if (err) {
			break;
		}

#if CONFIG_ESB_PTX_DELAY > 0
		break;
#endif
	}
}

#if CONFIG_ESB_PTX_DELAY > 0
static void timer_compare_handler(int32_t chan_id,
				  uint64_t expire_time,
				  void *user_data)
{
	timer_target_set();

	struct main_msg msg;
	int err;

	msg.type = MAIN_MSG_TIMER;
	err = k_msgq_put(&main_msgq, &msg, K_NO_WAIT);
	if (err) {
		LOG_ERR("Cannot put TIMER count to message queue");
	}
}

static int timer_target_set(void)
{
	int err;

	timer_tick += CONFIG_ESB_PTX_DELAY; // TODO: convert microseconds to timer tick

	err = z_nrf_grtc_timer_set(timer_chan,
				   timer_tick,
				   timer_compare_handler,
				   NULL);

	return err;
}
#endif

static void print_main(void *p1, void *p2, void *p3)
{
	struct print_msg msg;

	/* Process message queue */
	while (!k_msgq_get(&print_msgq, &msg, K_FOREVER)) {
#ifdef CONFIG_ESB_PTX_DATA_CHECK
		LOG_INF("Sent %u packets. Failed %u packets. Received %u packets. %u data error.",
			msg.tx_cnt, msg.tx_fail_cnt, msg.rx_cnt, msg.rx_error_cnt);
#else
		LOG_INF("Sent %u packets. Failed %u packets. Received %u packets.",
			msg.tx_cnt, msg.tx_fail_cnt, msg.rx_cnt);
#endif
		LOG_INF("Elapsed %lld milliseconds.", msg.time_cnt);
	}
}


int main(void)
{
	int err;

	LOG_INF("Enhanced ShockBurst ptx sample");

#if defined(CONFIG_CLOCK_CONTROL_NRF)
	err = clocks_start();
	if (err) {
		return 0;
	}
#endif /* #if defined(CONFIG_CLOCK_CONTROL_NRF) */

	err = dk_leds_init();
	if (err) {
		LOG_ERR("LEDs initialization failed, err %d", err);
		return 0;
	}

	err = esb_initialize();
	if (err) {
		LOG_ERR("ESB initialization failed, err %d", err);
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

#if CONFIG_ESB_PTX_DELAY > 0
	timer_chan = z_nrf_grtc_timer_chan_alloc();
	if (timer_chan < 0) {
		LOG_ERR("Cannot get a timer channel");
		return 0;
	}
#endif

	LOG_INF("Initialization complete");
	LOG_INF("Sending test packet");

	tx_payload.length = CONFIG_ESB_PTX_TX_LENGTH;
#ifndef CONFIG_ESB_PTX_DATA_CHECK
	fill_tx_payload_data(&tx_payload);
#endif
	tx_payload.noack = false;

#if CONFIG_ESB_PTX_DELAY > 0
	timer_tick = z_nrf_grtc_timer_read();
	timer_target_set();
#endif

	uint32_t tx_event_cnt = 0;
	uint32_t tx_fail_cnt = 0;
	uint32_t rx_fifo_cnt = 0;
#ifdef CONFIG_ESB_PTX_DATA_CHECK
	uint32_t rx_error_cnt = 0;
#endif

	struct main_msg msg;

	while (1) {
		int64_t time_cnt;

		time_cnt = k_uptime_get();

#if !CONFIG_ESB_PTX_DELAY
		fill_tx_fifo();
#endif

		do {
			k_msgq_get(&main_msgq, &msg, K_FOREVER);
			do {
				switch(msg.type) {
				case MAIN_MSG_TX_FAIL:
					tx_fail_cnt += msg.cnt;
				case MAIN_MSG_TX:
					tx_event_cnt += msg.cnt;
#if !CONFIG_ESB_PTX_DELAY
					fill_tx_fifo();
#endif
					break;
#if CONFIG_ESB_PTX_DELAY > 0
				case MAIN_MSG_TIMER:
					fill_tx_fifo();
					break;
#endif
				case MAIN_MSG_RX:
					rx_fifo_cnt += msg.cnt;
#ifdef CONFIG_ESB_PTX_DATA_CHECK
					rx_error_cnt += msg.cnt_err;
#endif
					break;
				default:
					break;
				}
			} while (!k_msgq_get(&main_msgq, &msg, K_NO_WAIT));
		} while (tx_event_cnt < CONFIG_ESB_PTX_BATCH_SIZE);

		time_cnt = k_uptime_delta(&time_cnt);

		struct print_msg prt_msg;

		prt_msg.tx_cnt = tx_event_cnt;
		tx_event_cnt = 0;
		prt_msg.tx_fail_cnt = tx_fail_cnt;
		tx_fail_cnt = 0;
		prt_msg.rx_cnt = rx_fifo_cnt;
		rx_fifo_cnt = 0;
#ifdef CONFIG_ESB_PTX_DATA_CHECK
		prt_msg.rx_error_cnt = rx_error_cnt;
		rx_error_cnt = 0;
#endif
		prt_msg.time_cnt = time_cnt;

		err = k_msgq_put(&print_msgq, &prt_msg, K_NO_WAIT);
		if (err) {
			LOG_ERR("Cannot put PTX statistics to message queue");
		}
	}
}

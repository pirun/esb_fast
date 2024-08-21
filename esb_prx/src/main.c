/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
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
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <dk_buttons_and_leds.h>

LOG_MODULE_REGISTER(esb_prx, CONFIG_ESB_PRX_APP_LOG_LEVEL);

/* Print cycle time, in microseconds */
#define PRINT_CYCLE     1000000

struct main_msg {
	uint32_t type;
	uint32_t cnt;
#ifdef CONFIG_ESB_PRX_DATA_CHECK
	uint32_t cnt_err;
#endif
};

#define MAIN_MSG_RX     0
#define MAIN_MSG_TX     1
#define MAIN_MSG_TIMER  2

struct print_msg {
	uint32_t rx_cnt;
	uint32_t tx_cnt;
#ifdef CONFIG_ESB_PRX_DATA_CHECK
	uint32_t rx_error_cnt;
#endif
};

static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17);

#if CONFIG_ESB_PRX_TX_LENGTH > 0
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0, 0x00);

static void fill_tx_payload_data(struct esb_payload *payload)
{
	static uint8_t tx_data_next;
	uint8_t i;

	for (i = 0; i < CONFIG_ESB_PRX_TX_LENGTH; i++) {
		payload->data[i] = tx_data_next + i;
	}

	tx_data_next += CONFIG_ESB_PRX_TX_LENGTH;
}

static void fill_tx_fifo(void)
{
	int err;
#ifdef CONFIG_ESB_PRX_DATA_CHECK
	static bool tx_data_set;
#endif

	while (1) {
#ifdef CONFIG_ESB_PRX_DATA_CHECK
		if (!tx_data_set) {
			fill_tx_payload_data(&tx_payload);
		}
#endif

		err = esb_write_payload(&tx_payload);
#ifdef CONFIG_ESB_PRX_DATA_CHECK
		tx_data_set = (!err) ? false : true;
#endif
		if (err) {
			break;
		}
	}
}
#endif

K_MSGQ_DEFINE(main_msgq,
	      sizeof(struct main_msg),
	      5,
	      sizeof(uint32_t));

K_MSGQ_DEFINE(print_msgq,
	      sizeof(struct print_msg),
	      1,
	      sizeof(uint32_t));

static struct k_thread print_thread;

static K_THREAD_STACK_DEFINE(print_stack, 512);

static int32_t timer_chan;
static uint64_t timer_tick;

static int timer_target_set(void);

static void leds_update(uint8_t value)
{
	uint32_t leds_mask =
		(!(value % 8 > 0 && value % 8 <= 4) ? DK_LED1_MSK : 0) |
		(!(value % 8 > 1 && value % 8 <= 5) ? DK_LED2_MSK : 0) |
		(!(value % 8 > 2 && value % 8 <= 6) ? DK_LED3_MSK : 0) |
		(!(value % 8 > 3) ? DK_LED4_MSK : 0);

	dk_set_leds(leds_mask);
}

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
			LOG_ERR("Cannot put TX count to message queue");
		}
		break;
	case ESB_EVENT_TX_FAILED:
		break;
	case ESB_EVENT_RX_RECEIVED:
		msg.type = MAIN_MSG_RX;
		msg.cnt = 0;
	    	while (!esb_read_rx_payload(&rx_payload)) {
			msg.cnt++;
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
【
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

	err = esb_set_rf_channel(40);
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
#ifdef CONFIG_ESB_PRX_DATA_CHECK
		LOG_INF("Received %u packets. Sent %u packets. %u data error.",
			msg.rx_cnt, msg.tx_cnt, msg.rx_error_cnt);
#else
		LOG_INF("Received %u packets. Sent %u packets.",
			msg.rx_cnt, msg.tx_cnt);
#endif
	}
}

int main(void)
{
	int err;

	LOG_INF("Enhanced ShockBurst prx sample");

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

	timer_chan = z_nrf_grtc_timer_chan_alloc();
	if (timer_chan < 0) {
		LOG_ERR("Cannot get a timer channel");
		return 0;
	}

	LOG_INF("Initialization complete");

	err = esb_write_payload(&tx_payload);
	if (err) {
		LOG_ERR("Write payload, err %d", err);
		return 0;
	}

	LOG_INF("Setting up for packet receiption");

	timer_tick = z_nrf_grtc_timer_read();
	timer_target_set();

	err = esb_start_rx();
	if (err) {
		LOG_ERR("RX setup failed, err %d", err);
		return 0;
	}

	uint32_t rx_done_cnt = 0;
	uint32_t tx_done_cnt = 0;

	struct main_msg msg;
	struct print_msg prt_msg;

	while (!k_msgq_get(&main_msgq, &msg, K_FOREVER)) {
		switch(msg.type) {
		case MAIN_MSG_RX:
			rx_done_cnt += msg.cnt;
#ifdef CONFIG_ESB_PRX_DATA_CHECK
			rx_error_cnt += msg.cnt_err;
#endif
			break;
		case MAIN_MSG_TX:
			tx_done_cnt += msg.cnt;
#if CONFIG_ESB_PRX_TX_LENGTH > 0
			fill_tx_fifo();
#endif
			break;
		case MAIN_MSG_TIMER:
			prt_msg.rx_cnt = rx_done_cnt;
			rx_done_cnt = 0;
			prt_msg.tx_cnt = tx_done_cnt;
			tx_done_cnt = 0;
#ifdef CONFIG_ESB_PRX_DATA_CHECK
			prt_msg.rx_error_cnt = rx_error_cnt;
			rx_error_cnt = 0;
#endif
			err = k_msgq_put(&print_msgq, &prt_msg, K_NO_WAIT);
			if (err) {
				LOG_ERR("Cannot put PRX statistics to message queue");
			}
			break;
		default:
			break;
		}
	}

	LOG_WRN("main loop exited");

	/* return to idle thread */
	return 0;
}

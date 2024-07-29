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
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
// #include <zephyr/usb/class/usb_hid.h>
#include <zephyr/usb/class/usbd_hid.h>
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

struct print_msg {
	uint32_t rep_cnt;
	uint32_t drop_cnt;
};

#define USB_HID_MSG_REPORT      0
#define USB_HID_MSG_TIMER       1

struct usb_hid_msg {
	uint32_t type;
	uint8_t report[4];
};
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);

static const uint8_t hid_report_desc[] = HID_MOUSE_REPORT_DESC(2);

static atomic_t usb_state = ATOMIC_INIT(0);
static K_SEM_DEFINE(report_sem, 0, 1);

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

K_MSGQ_DEFINE(print_msgq,
	      sizeof(struct print_msg),
	      1,
	      sizeof(uint32_t));

K_MSGQ_DEFINE(usb_hid_msgq,
	      sizeof(struct usb_hid_msg),
	      16,
	      sizeof(uint32_t));

static struct k_thread print_thread;

static K_THREAD_STACK_DEFINE(print_stack, 512);

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
			struct usb_hid_msg msg;

			if (rx_payload.length >= sizeof(msg.report) &&
			    atomic_test_bit(&usb_state, 0)) {
				msg.type = USB_HID_MSG_REPORT;
				memcpy(msg.report,
				       rx_payload.data,
				       sizeof(msg.report));
				err = k_msgq_put(&usb_hid_msgq, &msg, K_NO_WAIT);
				if (err) {
				}
			}
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
	config.bitrate = ESB_BITRATE_2MBPS;
	config.mode = ESB_MODE_PRX;
	config.event_handler = event_handler;
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

	struct usb_hid_msg  msg;
	int err;

	msg.type = USB_HID_MSG_TIMER;
	err = k_msgq_put(&usb_hid_msgq, &msg, K_NO_WAIT);
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
		LOG_INF("Sent %u HID reports. Dropped %u reports.",
			msg.rep_cnt, msg.drop_cnt);
	}
}

static void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	if (status == USB_DC_CONFIGURED) {
		atomic_set_bit(&usb_state, 0);
	} else {
		atomic_clear_bit(&usb_state, 0);
	}
}

static uint32_t idle_duration;

void user_sof(const struct device *dev)
{
	// gpio_pin_toggle(led3.port, led3.pin);
	k_sem_give(&report_sem);
}

static void iface_ready_next(const struct device *dev, const bool ready)
{
	return;
}

static int get_report_next(const struct device *dev, const uint8_t type, const uint8_t id,
			   const uint16_t len, uint8_t *const buf)
{
	return len;
}

static int set_report_next(const struct device *dev, const uint8_t type, const uint8_t id,
			   const uint16_t len, const uint8_t *const buf)
{
	return len;
}
static void set_idle_next(const struct device *dev, const uint8_t id, const uint32_t duration)
{
	idle_duration = duration;
}

static uint32_t get_idle_next(const struct device *dev, const uint8_t id)
{
	return idle_duration;
}

static void report_sent_cb_next(const struct device *dev)
{
	return;
}
static void protocol_change(const struct device *dev, uint8_t protocol)
{
	return;
}
#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
static int enable_usb_device_next(void)
{
	struct usbd_context *sample_usbd;
	int err;

	sample_usbd = sample_usbd_init_device(NULL);
	if (sample_usbd == NULL) {
		LOG_ERR("Failed to initialize USB device");
		return -ENODEV;
	}

	err = usbd_enable(sample_usbd);
	if (err) {
		LOG_ERR("Failed to enable device support");
		return err;
	}

	LOG_DBG("USB device support enabled");

	return 0;
}
#endif /* IS_ENABLED(CONFIG_USB_DEVICE_STACK_NEXT) */

static void usb_hid_main(void)
{
	int err;
	const struct device *hid_dev;
	struct usb_hid_msg msg;
	static const struct hid_device_ops my_ops = {
		.iface_ready = iface_ready_next,
		.get_report = get_report_next,
		.set_report = set_report_next,
		.set_idle = set_idle_next,
		.get_idle = get_idle_next,
		.set_protocol = protocol_change,
		.input_report_done = report_sent_cb_next,
		.sof = user_sof,

	};

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
	hid_dev = DEVICE_DT_GET_ONE(zephyr_hid_device);
#else
	hid_dev = device_get_binding("HID_0");
#endif

	// usb_hid_register_device(hid_dev,
	// 			hid_report_desc, sizeof(hid_report_desc),
	// 			&my_ops);

	hid_device_register(hid_dev, hid_report_desc, sizeof(hid_report_desc), &my_ops);

	usb_hid_init(hid_dev);

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
	err = enable_usb_device_next();
#else
	err = usb_enable(status_cb);
#endif
	if (err) {
		LOG_ERR("Failed to enable USB");
		return;
	}

	uint32_t hid_rep_cnt = 0;
	uint32_t hid_drop_cnt = 0;
	struct print_msg prt_msg;
	uint32_t bytes_ret;

	atomic_set_bit(&usb_state, 0);

	/* Process message queue */
	while (!k_msgq_get(&usb_hid_msgq, &msg, K_FOREVER)) {
		switch(msg.type) {
		case USB_HID_MSG_REPORT:
			k_sem_take(&report_sem, K_FOREVER);
			{
			// if (atomic_test_bit(&usb_state, 0)) {
				err = hid_int_ep_write(hid_dev,
						       msg.report,
						       sizeof(msg.report),
						       &bytes_ret);
				if (!err && bytes_ret == sizeof(msg.report)) {
					hid_rep_cnt++;
				} else {
					hid_drop_cnt++;
				}
			}
			break;
		case USB_HID_MSG_TIMER:
			prt_msg.rep_cnt = hid_rep_cnt;
			hid_rep_cnt = 0;
			prt_msg.drop_cnt = hid_drop_cnt;
			hid_drop_cnt = 0;
			// err = k_msgq_put(&print_msgq, &prt_msg, K_NO_WAIT);
			// if (err) {
			// 	LOG_ERR("Cannot put PRX statistics to message queue");
			// }
			break;
		default:
			break;
		}
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

	err = gpio_pin_configure_dt(&led3, GPIO_OUTPUT);
	if (err < 0) {
		LOG_ERR("Failed to configure the LED pin, error: %d", err);
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

	LOG_INF("Print packet counts every %u.%02u seconds",
		PRINT_CYCLE / USEC_PER_SEC,
		(PRINT_CYCLE % USEC_PER_SEC) * 100 / USEC_PER_SEC);

	LOG_INF("Setting up for packet receiption");

	timer_tick = z_nrf_grtc_timer_read();
	timer_target_set();

	err = esb_start_rx();
	if (err) {
		LOG_ERR("RX setup failed, err %d", err);
		return 0;
	}

	usb_hid_main();

	LOG_WRN("main loop exited");

	/* return to idle thread */
	return 0;
}

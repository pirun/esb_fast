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
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/ipc/ipc_service.h>

LOG_MODULE_REGISTER(host, LOG_LEVEL_INF);

/* Print cycle time, in microseconds */
#define PRINT_CYCLE     1000000

struct print_msg {
	uint32_t rep_cnt;
	uint32_t drop_cnt;
};

#define USB_HID_MSG_REPORT      0
#define USB_HID_MSG_TIMER       1
#define USB_HID_MSG_FORWARD     2


struct usb_hid_msg {
	uint32_t type;
	uint8_t report[4];
};
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);

static const uint8_t hid_report_desc[] = HID_MOUSE_REPORT_DESC(2);
static K_SEM_DEFINE(report_sem, 0, 1);

K_MSGQ_DEFINE(usb_hid_msgq,
	      sizeof(struct usb_hid_msg),
	      16,
	      sizeof(uint32_t));

K_MSGQ_DEFINE(print_msgq,
	      sizeof(struct print_msg),
	      1,
	      sizeof(uint32_t));

static struct k_thread print_thread;

static K_THREAD_STACK_DEFINE(print_stack, 512);

static int32_t timer_chan;
static uint64_t timer_tick;
struct payload {
	unsigned long cnt;
	unsigned long size;
	uint8_t data[];
};

struct payload *p_payload;

static K_SEM_DEFINE(bound_sem, 0, 1);
static int timer_target_set(void);

static void ep_bound(void *priv)
{
	k_sem_give(&bound_sem);
}

static void ep_recv(const void *data, size_t len, void *priv)
{
	struct usb_hid_msg msg;

	/* No message loss check here.
	 * If USB processing is stuck, HID reports may be lost.
	 * Normally it works fine with USB 2.0.
	 */
	if (len == sizeof(msg.report)) {
		msg.type = USB_HID_MSG_REPORT;
		memcpy(msg.report, data, len);
		k_msgq_put(&usb_hid_msgq, &msg, K_NO_WAIT);
	}
}

static struct ipc_ept_cfg ep_cfg = {
	.name = "ep0",
	.cb = {
		.bound    = ep_bound,
		.received = ep_recv,
	},
};

static void timer_compare_handler(int32_t chan_id,
				  uint64_t expire_time,
				  void *user_data)
{
	timer_target_set();

	struct usb_hid_msg msg;
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

	hid_dev = DEVICE_DT_GET_ONE(zephyr_hid_device);


	hid_device_register(hid_dev, hid_report_desc, sizeof(hid_report_desc), &my_ops);

	usb_hid_init(hid_dev);

	err = enable_usb_device_next();

	if (err) {
		LOG_ERR("Failed to enable USB");
		return;
	}

	uint32_t hid_rep_cnt = 0;
	uint32_t hid_drop_cnt = 0;
	struct print_msg prt_msg;
	uint32_t bytes_ret;

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
static void button_handler(uint32_t button_state, uint32_t has_changed)
{
	int err;
	struct usb_hid_msg msg;

	if ((has_changed & DK_BTN1_MSK) && (button_state & DK_BTN1_MSK)) {
		msg.type = USB_HID_MSG_FORWARD;
		err = k_msgq_put(&usb_hid_msgq, &msg, K_NO_WAIT);
		if (err) {
			LOG_WRN("Cannot put button to message queue");
		}
	}
}

int main(void)
{
	const struct device *ipc0_instance;
	struct ipc_ept ep;

	int err;

	LOG_INF("Enhanced ShockBurst hid dongle sample");

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
	ipc0_instance = DEVICE_DT_GET(DT_NODELABEL(ipc0));

	err = ipc_service_open_instance(ipc0_instance);
	if ((err < 0) && (err != -EALREADY)) {
		LOG_INF("ipc_service_open_instance() failure (%d)", err);
		return err;
	}

	err = ipc_service_register_endpoint(ipc0_instance, &ep, &ep_cfg);
	if (err < 0) {
		printf("ipc_service_register_endpoint() failure (%d)", err);
		return err;
	}

	k_sem_take(&bound_sem, K_FOREVER);
	err = dk_buttons_init(button_handler);
		if (err) {
		LOG_ERR("Buttons initialization failed, err %d", err);
		return 0;
	}
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

	usb_hid_main();

	LOG_WRN("main loop exited");

	/* return to idle thread */
	return 0;
}

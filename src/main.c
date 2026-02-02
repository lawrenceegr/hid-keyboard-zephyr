/*
 * Copyright (c) 2026 Lawrence Langat <lawrencelangatmi@gmail.com>
 * 88-key USB HID Keyboard implementation
 */

#include "usbd_init.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>

#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_hid.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static const uint8_t hid_report_desc[] = HID_KEYBOARD_REPORT_DESC();

/* LED indices for keyboard status LEDs */
enum kb_leds_idx {
	KB_LED_NUMLOCK = 0,
	KB_LED_CAPSLOCK,
	KB_LED_SCROLLLOCK,
	KB_LED_COUNT,
};

static const struct gpio_dt_spec kb_leds[KB_LED_COUNT] = {
	GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0}),
	GPIO_DT_SPEC_GET_OR(DT_ALIAS(led1), gpios, {0}),
	GPIO_DT_SPEC_GET_OR(DT_ALIAS(led2), gpios, {0}),
};

/* HID keyboard report structure (8 bytes for boot protocol) */
enum kb_report_idx {
	KB_MOD_KEY = 0,
	KB_RESERVED,
	KB_KEY_CODE1,
	KB_KEY_CODE2,
	KB_KEY_CODE3,
	KB_KEY_CODE4,
	KB_KEY_CODE5,
	KB_KEY_CODE6,
	KB_REPORT_COUNT,
};

#define MAX_PRESSED_KEYS 6

struct kb_event {
	uint16_t code;
	int32_t value;
};

K_MSGQ_DEFINE(kb_msgq, sizeof(struct kb_event), 16, 4);

UDC_STATIC_BUF_DEFINE(report, KB_REPORT_COUNT);
static uint32_t kb_duration;
static bool kb_ready;

/* Track currently pressed keys (up to 6 for 6KRO) */
static uint8_t pressed_keys[MAX_PRESSED_KEYS];
static uint8_t pressed_count;
static uint8_t modifier_state;

/*
 * INPUT_KEY to HID_KEY conversion table
 * Maps Linux input event codes to USB HID keyboard codes
 * Index: INPUT_KEY_* value, Value: HID_KEY_* value (0 = not mapped)
 */
static const uint8_t input_to_hid_key[] = {
	[INPUT_KEY_RESERVED] = 0,
	[INPUT_KEY_ESC] = HID_KEY_ESC,
	[INPUT_KEY_1] = HID_KEY_1,
	[INPUT_KEY_2] = HID_KEY_2,
	[INPUT_KEY_3] = HID_KEY_3,
	[INPUT_KEY_4] = HID_KEY_4,
	[INPUT_KEY_5] = HID_KEY_5,
	[INPUT_KEY_6] = HID_KEY_6,
	[INPUT_KEY_7] = HID_KEY_7,
	[INPUT_KEY_8] = HID_KEY_8,
	[INPUT_KEY_9] = HID_KEY_9,
	[INPUT_KEY_0] = HID_KEY_0,
	[INPUT_KEY_MINUS] = HID_KEY_MINUS,
	[INPUT_KEY_EQUAL] = HID_KEY_EQUAL,
	[INPUT_KEY_BACKSPACE] = HID_KEY_BACKSPACE,
	[INPUT_KEY_TAB] = HID_KEY_TAB,
	[INPUT_KEY_Q] = HID_KEY_Q,
	[INPUT_KEY_W] = HID_KEY_W,
	[INPUT_KEY_E] = HID_KEY_E,
	[INPUT_KEY_R] = HID_KEY_R,
	[INPUT_KEY_T] = HID_KEY_T,
	[INPUT_KEY_Y] = HID_KEY_Y,
	[INPUT_KEY_U] = HID_KEY_U,
	[INPUT_KEY_I] = HID_KEY_I,
	[INPUT_KEY_O] = HID_KEY_O,
	[INPUT_KEY_P] = HID_KEY_P,
	[INPUT_KEY_LEFTBRACE] = HID_KEY_LEFTBRACE,
	[INPUT_KEY_RIGHTBRACE] = HID_KEY_RIGHTBRACE,
	[INPUT_KEY_ENTER] = HID_KEY_ENTER,
	[INPUT_KEY_LEFTCTRL] = 0, /* Handled as modifier */
	[INPUT_KEY_A] = HID_KEY_A,
	[INPUT_KEY_S] = HID_KEY_S,
	[INPUT_KEY_D] = HID_KEY_D,
	[INPUT_KEY_F] = HID_KEY_F,
	[INPUT_KEY_G] = HID_KEY_G,
	[INPUT_KEY_H] = HID_KEY_H,
	[INPUT_KEY_J] = HID_KEY_J,
	[INPUT_KEY_K] = HID_KEY_K,
	[INPUT_KEY_L] = HID_KEY_L,
	[INPUT_KEY_SEMICOLON] = HID_KEY_SEMICOLON,
	[INPUT_KEY_APOSTROPHE] = HID_KEY_APOSTROPHE,
	[INPUT_KEY_GRAVE] = HID_KEY_GRAVE,
	[INPUT_KEY_LEFTSHIFT] = 0, /* Handled as modifier */
	[INPUT_KEY_BACKSLASH] = HID_KEY_BACKSLASH,
	[INPUT_KEY_Z] = HID_KEY_Z,
	[INPUT_KEY_X] = HID_KEY_X,
	[INPUT_KEY_C] = HID_KEY_C,
	[INPUT_KEY_V] = HID_KEY_V,
	[INPUT_KEY_B] = HID_KEY_B,
	[INPUT_KEY_N] = HID_KEY_N,
	[INPUT_KEY_M] = HID_KEY_M,
	[INPUT_KEY_COMMA] = HID_KEY_COMMA,
	[INPUT_KEY_DOT] = HID_KEY_DOT,
	[INPUT_KEY_SLASH] = HID_KEY_SLASH,
	[INPUT_KEY_RIGHTSHIFT] = 0, /* Handled as modifier */
	[INPUT_KEY_KPASTERISK] = HID_KEY_KPASTERISK,
	[INPUT_KEY_LEFTALT] = 0, /* Handled as modifier */
	[INPUT_KEY_SPACE] = HID_KEY_SPACE,
	[INPUT_KEY_CAPSLOCK] = HID_KEY_CAPSLOCK,
	[INPUT_KEY_F1] = HID_KEY_F1,
	[INPUT_KEY_F2] = HID_KEY_F2,
	[INPUT_KEY_F3] = HID_KEY_F3,
	[INPUT_KEY_F4] = HID_KEY_F4,
	[INPUT_KEY_F5] = HID_KEY_F5,
	[INPUT_KEY_F6] = HID_KEY_F6,
	[INPUT_KEY_F7] = HID_KEY_F7,
	[INPUT_KEY_F8] = HID_KEY_F8,
	[INPUT_KEY_F9] = HID_KEY_F9,
	[INPUT_KEY_F10] = HID_KEY_F10,
	[INPUT_KEY_NUMLOCK] = HID_KEY_NUMLOCK,
	[INPUT_KEY_SCROLLLOCK] = HID_KEY_SCROLLLOCK,
	[INPUT_KEY_KP7] = HID_KEY_KP_7,
	[INPUT_KEY_KP8] = HID_KEY_KP_8,
	[INPUT_KEY_KP9] = HID_KEY_KP_9,
	[INPUT_KEY_KPMINUS] = HID_KEY_KPMINUS,
	[INPUT_KEY_KP4] = HID_KEY_KP_4,
	[INPUT_KEY_KP5] = HID_KEY_KP_5,
	[INPUT_KEY_KP6] = HID_KEY_KP_6,
	[INPUT_KEY_KPPLUS] = HID_KEY_KPPLUS,
	[INPUT_KEY_KP1] = HID_KEY_KP_1,
	[INPUT_KEY_KP2] = HID_KEY_KP_2,
	[INPUT_KEY_KP3] = HID_KEY_KP_3,
	[INPUT_KEY_KP0] = HID_KEY_KP_0,
	[INPUT_KEY_KPDOT] = 99, /* HID_KEY_KPDOT */
	[INPUT_KEY_F11] = HID_KEY_F11,
	[INPUT_KEY_F12] = HID_KEY_F12,
	[INPUT_KEY_KPENTER] = HID_KEY_KPENTER,
	[INPUT_KEY_RIGHTCTRL] = 0, /* Handled as modifier */
	[INPUT_KEY_KPSLASH] = HID_KEY_KPSLASH,
	[INPUT_KEY_SYSRQ] = HID_KEY_SYSRQ,
	[INPUT_KEY_RIGHTALT] = 0, /* Handled as modifier */
	[INPUT_KEY_HOME] = HID_KEY_HOME,
	[INPUT_KEY_UP] = HID_KEY_UP,
	[INPUT_KEY_PAGEUP] = HID_KEY_PAGEUP,
	[INPUT_KEY_LEFT] = HID_KEY_LEFT,
	[INPUT_KEY_RIGHT] = HID_KEY_RIGHT,
	[INPUT_KEY_END] = HID_KEY_END,
	[INPUT_KEY_DOWN] = HID_KEY_DOWN,
	[INPUT_KEY_PAGEDOWN] = HID_KEY_PAGEDOWN,
	[INPUT_KEY_INSERT] = HID_KEY_INSERT,
	[INPUT_KEY_DELETE] = HID_KEY_DELETE,
	[INPUT_KEY_PAUSE] = HID_KEY_PAUSE,
	[INPUT_KEY_LEFTMETA] = 0, /* Handled as modifier */
	[INPUT_KEY_RIGHTMETA] = 0, /* Handled as modifier */
	[INPUT_KEY_COMPOSE] = 101, /* HID_KEY_COMPOSE / Application */
};

#define INPUT_TO_HID_TABLE_SIZE ARRAY_SIZE(input_to_hid_key)

/*
 * Check if the input key is a modifier key and return the modifier bit
 * Returns 0 if not a modifier
 */
static uint8_t get_modifier_bit(uint16_t input_code)
{
	switch (input_code) {
	case INPUT_KEY_LEFTCTRL:
		return HID_KBD_MODIFIER_LEFT_CTRL;
	case INPUT_KEY_LEFTSHIFT:
		return HID_KBD_MODIFIER_LEFT_SHIFT;
	case INPUT_KEY_LEFTALT:
		return HID_KBD_MODIFIER_LEFT_ALT;
	case INPUT_KEY_LEFTMETA:
		return HID_KBD_MODIFIER_LEFT_UI;
	case INPUT_KEY_RIGHTCTRL:
		return HID_KBD_MODIFIER_RIGHT_CTRL;
	case INPUT_KEY_RIGHTSHIFT:
		return HID_KBD_MODIFIER_RIGHT_SHIFT;
	case INPUT_KEY_RIGHTALT:
		return HID_KBD_MODIFIER_RIGHT_ALT;
	case INPUT_KEY_RIGHTMETA:
		return HID_KBD_MODIFIER_RIGHT_UI;
	default:
		return 0;
	}
}

/*
 * Convert INPUT_KEY code to HID key code
 * Returns 0 if not mappable (modifiers return 0 here, handled separately)
 */
static uint8_t input_to_hid(uint16_t input_code)
{
	if (input_code < INPUT_TO_HID_TABLE_SIZE) {
		return input_to_hid_key[input_code];
	}
	return 0;
}

/*
 * Add a key to the pressed keys array
 * Returns true if the key was added or already present
 */
static bool add_pressed_key(uint8_t hid_key)
{
	/* Check if already pressed */
	for (int i = 0; i < pressed_count; i++) {
		if (pressed_keys[i] == hid_key) {
			return true;
		}
	}

	/* Add if room available */
	if (pressed_count < MAX_PRESSED_KEYS) {
		pressed_keys[pressed_count++] = hid_key;
		return true;
	}

	LOG_WRN("6KRO limit reached, key ignored");
	return false;
}

/*
 * Remove a key from the pressed keys array
 */
static void remove_pressed_key(uint8_t hid_key)
{
	for (int i = 0; i < pressed_count; i++) {
		if (pressed_keys[i] == hid_key) {
			/* Shift remaining keys */
			for (int j = i; j < pressed_count - 1; j++) {
				pressed_keys[j] = pressed_keys[j + 1];
			}
			pressed_count--;
			return;
		}
	}
}

/*
 * Build the HID report from current state
 */
static void build_hid_report(void)
{
	memset(report, 0, KB_REPORT_COUNT);

	report[KB_MOD_KEY] = modifier_state;
	report[KB_RESERVED] = 0;

	for (int i = 0; i < pressed_count && i < MAX_PRESSED_KEYS; i++) {
		report[KB_KEY_CODE1 + i] = pressed_keys[i];
	}
}

/*
 * Process a key event and update state
 */
static void process_key_event(uint16_t input_code, bool pressed)
{
	uint8_t mod_bit = get_modifier_bit(input_code);

	if (mod_bit != 0) {
		/* Handle modifier keys */
		if (pressed) {
			modifier_state |= mod_bit;
		} else {
			modifier_state &= ~mod_bit;
		}
	} else {
		/* Handle regular keys */
		uint8_t hid_key = input_to_hid(input_code);

		if (hid_key != 0) {
			if (pressed) {
				add_pressed_key(hid_key);
			} else {
				remove_pressed_key(hid_key);
			}
		} else {
			LOG_DBG("Unmapped input code: %u", input_code);
		}
	}

	build_hid_report();
}

static void input_cb(struct input_event *evt, void *user_data)
{
	struct kb_event kb_evt;

	ARG_UNUSED(user_data);

	/* Only process key events */
	if (evt->type != INPUT_EV_KEY) {
		return;
	}

	kb_evt.code = evt->code;
	kb_evt.value = evt->value;
	if (k_msgq_put(&kb_msgq, &kb_evt, K_NO_WAIT) != 0) {
		LOG_ERR("Failed to put new input event");
	}
}

INPUT_CALLBACK_DEFINE(NULL, input_cb, NULL);

static void kb_iface_ready(const struct device *dev, const bool ready)
{
	LOG_INF("HID device %s interface is %s",
		dev->name, ready ? "ready" : "not ready");
	kb_ready = ready;
}

static int kb_get_report(const struct device *dev,
			 const uint8_t type, const uint8_t id, const uint16_t len,
			 uint8_t *const buf)
{
	if (type == HID_REPORT_TYPE_INPUT && len >= KB_REPORT_COUNT) {
		memcpy(buf, report, KB_REPORT_COUNT);
		return KB_REPORT_COUNT;
	}

	LOG_WRN("Get Report not implemented, Type %u ID %u", type, id);
	return 0;
}

static int kb_set_report(const struct device *dev,
			 const uint8_t type, const uint8_t id, const uint16_t len,
			 const uint8_t *const buf)
{
	if (type != HID_REPORT_TYPE_OUTPUT) {
		LOG_WRN("Unsupported report type");
		return -ENOTSUP;
	}

	/* Handle LED state from host */
	for (unsigned int i = 0; i < ARRAY_SIZE(kb_leds); i++) {
		if (kb_leds[i].port == NULL) {
			continue;
		}

		(void)gpio_pin_set_dt(&kb_leds[i], buf[0] & BIT(i));
	}

	return 0;
}

static void kb_set_idle(const struct device *dev,
			const uint8_t id, const uint32_t duration)
{
	LOG_INF("Set Idle %u to %u", id, duration);
	kb_duration = duration;
}

static uint32_t kb_get_idle(const struct device *dev, const uint8_t id)
{
	LOG_INF("Get Idle %u to %u", id, kb_duration);
	return kb_duration;
}

static void kb_set_protocol(const struct device *dev, const uint8_t proto)
{
	LOG_INF("Protocol changed to %s",
		proto == 0U ? "Boot Protocol" : "Report Protocol");
}

static void kb_output_report(const struct device *dev, const uint16_t len,
			     const uint8_t *const buf)
{
	LOG_HEXDUMP_DBG(buf, len, "o.r.");
	kb_set_report(dev, HID_REPORT_TYPE_OUTPUT, 0U, len, buf);
}

struct hid_device_ops kb_ops = {
	.iface_ready = kb_iface_ready,
	.get_report = kb_get_report,
	.set_report = kb_set_report,
	.set_idle = kb_set_idle,
	.get_idle = kb_get_idle,
	.set_protocol = kb_set_protocol,
	.output_report = kb_output_report,
};

static void msg_cb(struct usbd_context *const usbd_ctx,
		   const struct usbd_msg *const msg)
{
	LOG_INF("USBD message: %s", usbd_msg_type_string(msg->type));

	if (msg->type == USBD_MSG_CONFIGURATION) {
		LOG_INF("\tConfiguration value %d", msg->status);
	}

	if (usbd_can_detect_vbus(usbd_ctx)) {
		if (msg->type == USBD_MSG_VBUS_READY) {
			if (usbd_enable(usbd_ctx)) {
				LOG_ERR("Failed to enable device support");
			}
		}

		if (msg->type == USBD_MSG_VBUS_REMOVED) {
			if (usbd_disable(usbd_ctx)) {
				LOG_ERR("Failed to disable device support");
			}
		}
	}
}

int main(void)
{
	struct usbd_context *kbd_usbd;
	const struct device *hid_dev;
	int ret;

	/* Initialize LEDs */
	for (unsigned int i = 0; i < ARRAY_SIZE(kb_leds); i++) {
		if (kb_leds[i].port == NULL) {
			continue;
		}

		if (!gpio_is_ready_dt(&kb_leds[i])) {
			LOG_ERR("LED device %s is not ready", kb_leds[i].port->name);
			return -EIO;
		}

		ret = gpio_pin_configure_dt(&kb_leds[i], GPIO_OUTPUT_INACTIVE);
		if (ret != 0) {
			LOG_ERR("Failed to configure the LED pin, %d", ret);
			return -EIO;
		}
	}

	/* Initialize HID device */
	hid_dev = DEVICE_DT_GET_ONE(zephyr_hid_device);
	if (!device_is_ready(hid_dev)) {
		LOG_ERR("HID Device is not ready");
		return -EIO;
	}

	ret = hid_device_register(hid_dev,
				  hid_report_desc, sizeof(hid_report_desc),
				  &kb_ops);
	if (ret != 0) {
		LOG_ERR("Failed to register HID Device, %d", ret);
		return ret;
	}

	if (IS_ENABLED(CONFIG_USBD_HID_SET_POLLING_PERIOD)) {
		ret = hid_device_set_in_polling(hid_dev, 1000);
		if (ret) {
			LOG_WRN("Failed to set IN report polling period, %d", ret);
		}

		ret = hid_device_set_out_polling(hid_dev, 1000);
		if (ret != 0 && ret != -ENOTSUP) {
			LOG_WRN("Failed to set OUT report polling period, %d", ret);
		}
	}

	/* Initialize USB device */
	kbd_usbd = keyboard_usbd_init(msg_cb);
	if (kbd_usbd == NULL) {
		LOG_ERR("Failed to initialize USB device");
		return -ENODEV;
	}

	if (!usbd_can_detect_vbus(kbd_usbd)) {
		ret = usbd_enable(kbd_usbd);
		if (ret) {
			LOG_ERR("Failed to enable device support");
			return ret;
		}
	}

	LOG_INF("88-key HID keyboard initialized");

	/* Main event loop */
	while (true) {
		struct kb_event kb_evt;

		k_msgq_get(&kb_msgq, &kb_evt, K_FOREVER);

		/* Process the key event */
		process_key_event(kb_evt.code, kb_evt.value != 0);

		if (!kb_ready) {
			LOG_DBG("USB HID device is not ready");
			continue;
		}

		/* Handle remote wakeup if suspended */
		if (IS_ENABLED(CONFIG_KEYBOARD_USBD_REMOTE_WAKEUP) &&
		    usbd_is_suspended(kbd_usbd)) {
			if (kb_evt.value) {
				ret = usbd_wakeup_request(kbd_usbd);
				if (ret) {
					LOG_ERR("Remote wakeup error, %d", ret);
				}
			}
			continue;
		}

		/* Submit the HID report */
		ret = hid_device_submit_report(hid_dev, KB_REPORT_COUNT, report);
		if (ret) {
			LOG_ERR("HID submit report error, %d", ret);
		}
	}

	return 0;
}

/*
 * Copyright (c) 2026 Lawrence Langat <lawrencelangatmi@gmail.com>
 * USB device initialization for custom keyboard
 */

#include "usbd_init.h"

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/usb/usbd.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usbd_keyboard_config);

/*
 * Instantiate USB device context using configured VID/PID
 */
USBD_DEVICE_DEFINE(kbd_usbd,
		   DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
		   CONFIG_KEYBOARD_USBD_VID, CONFIG_KEYBOARD_USBD_PID);

/* String descriptors */
USBD_DESC_LANG_DEFINE(kbd_lang);
USBD_DESC_MANUFACTURER_DEFINE(kbd_mfr, CONFIG_KEYBOARD_USBD_MANUFACTURER);
USBD_DESC_PRODUCT_DEFINE(kbd_product, CONFIG_KEYBOARD_USBD_PRODUCT);

/* Configuration descriptors */
USBD_DESC_CONFIG_DEFINE(fs_cfg_desc, "FS Configuration");
USBD_DESC_CONFIG_DEFINE(hs_cfg_desc, "HS Configuration");

/* Configuration attributes */
static const uint8_t attributes = (IS_ENABLED(CONFIG_KEYBOARD_USBD_SELF_POWERED) ?
				   USB_SCD_SELF_POWERED : 0) |
				  (IS_ENABLED(CONFIG_KEYBOARD_USBD_REMOTE_WAKEUP) ?
				   USB_SCD_REMOTE_WAKEUP : 0);

/* Full speed configuration */
USBD_CONFIGURATION_DEFINE(kbd_fs_config,
			  attributes,
			  CONFIG_KEYBOARD_USBD_MAX_POWER, &fs_cfg_desc);

/* High speed configuration */
USBD_CONFIGURATION_DEFINE(kbd_hs_config,
			  attributes,
			  CONFIG_KEYBOARD_USBD_MAX_POWER, &hs_cfg_desc);

struct usbd_context *keyboard_usbd_init(usbd_msg_cb_t msg_cb)
{
	int err;

	/* Add string descriptors */
	err = usbd_add_descriptor(&kbd_usbd, &kbd_lang);
	if (err) {
		LOG_ERR("Failed to initialize language descriptor (%d)", err);
		return NULL;
	}

	err = usbd_add_descriptor(&kbd_usbd, &kbd_mfr);
	if (err) {
		LOG_ERR("Failed to initialize manufacturer descriptor (%d)", err);
		return NULL;
	}

	err = usbd_add_descriptor(&kbd_usbd, &kbd_product);
	if (err) {
		LOG_ERR("Failed to initialize product descriptor (%d)", err);
		return NULL;
	}

	/* Add high-speed configuration if supported */
	if (USBD_SUPPORTS_HIGH_SPEED &&
	    usbd_caps_speed(&kbd_usbd) == USBD_SPEED_HS) {
		err = usbd_add_configuration(&kbd_usbd, USBD_SPEED_HS,
					     &kbd_hs_config);
		if (err) {
			LOG_ERR("Failed to add High-Speed configuration");
			return NULL;
		}

		err = usbd_register_all_classes(&kbd_usbd, USBD_SPEED_HS, 1, NULL);
		if (err) {
			LOG_ERR("Failed to register HS classes");
			return NULL;
		}

		usbd_device_set_code_triple(&kbd_usbd, USBD_SPEED_HS, 0, 0, 0);
	}

	/* Add full-speed configuration */
	err = usbd_add_configuration(&kbd_usbd, USBD_SPEED_FS, &kbd_fs_config);
	if (err) {
		LOG_ERR("Failed to add Full-Speed configuration");
		return NULL;
	}

	err = usbd_register_all_classes(&kbd_usbd, USBD_SPEED_FS, 1, NULL);
	if (err) {
		LOG_ERR("Failed to register FS classes");
		return NULL;
	}

	usbd_device_set_code_triple(&kbd_usbd, USBD_SPEED_FS, 0, 0, 0);
	usbd_self_powered(&kbd_usbd, attributes & USB_SCD_SELF_POWERED);

	/* Register message callback */
	if (msg_cb != NULL) {
		err = usbd_msg_register_cb(&kbd_usbd, msg_cb);
		if (err) {
			LOG_ERR("Failed to register message callback");
			return NULL;
		}
	}

	/* Initialize USB device */
	err = usbd_init(&kbd_usbd);
	if (err) {
		LOG_ERR("Failed to initialize device support");
		return NULL;
	}

	return &kbd_usbd;
}

/*
 * Copyright (c) 2026 Lawrence Langat <lawrencelangatmi@gmail.com>
 */

#ifndef KEYBOARD_USBD_INIT_H
#define KEYBOARD_USBD_INIT_H

#include <zephyr/usb/usbd.h>

/*
 * Initialize and configure the USB device for the keyboard.
 *
 * This function configures the USB device context, string descriptors,
 * USB device configuration, registers HID class, and initializes the
 * USB device stack.
 *
 * @param msg_cb Optional message callback for USB events (can be NULL)
 * @return Pointer to initialized USB device context, or NULL on failure
 */
struct usbd_context *keyboard_usbd_init(usbd_msg_cb_t msg_cb);

#endif /* KEYBOARD_USBD_INIT_H */

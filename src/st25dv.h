/*
 * Copyright (c) 2026 Telink Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr ST25DVXXKC mailbox driver implementation
 */

#ifndef ST25DV_H
#define ST25DV_H

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#define ST25DV_DEF_PASS (uint8_t[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

struct st25dv_data;

enum st25dv_mode {
	ST25DV_MODE_OFF,
	ST25DV_MODE_TAG,
	ST25DV_MODE_MAILBOX,
};

enum st25dv_event {
	ST25DV_EVENT_FIELD_ON,
	ST25DV_EVENT_FIELD_OFF,
	ST25DV_EVENT_DATA_RECEIVED,
	ST25DV_EVENT_DATA_SENT,
};

typedef void (*st25dv_on_event)(enum st25dv_event event, void *ctx, uint8_t *buf, size_t buf_len);

struct st25dv_data {
	const struct device *const i2c_bus;
	const struct device *const isr_port;
	const uint8_t isr_port_pin;
	bool inited;
	struct gpio_callback gpio_cb;
	struct k_work isr_work;
	st25dv_on_event event_handler;
	void *event_ctx;
	IF_ENABLED(CONFIG_ST25DV_MULTITHREAD_SUPPORT,
		(struct k_mutex protector;)
	)
};

#define ST25DV_DATA_DEFINE(name, st25dv_node_label)                                                \
	struct st25dv_data name = {                                                                \
		.i2c_bus = DEVICE_DT_GET(DT_PHANDLE(DT_NODELABEL(st25dv_node_label), i2c_bus)),    \
		.isr_port = DEVICE_DT_GET(DT_PHANDLE(DT_NODELABEL(st25dv_node_label), isr_port)),  \
		.isr_port_pin = DT_PROP(DT_NODELABEL(st25dv_node_label), isr_port_pin),            \
		IF_ENABLED(CONFIG_ST25DV_MULTITHREAD_SUPPORT,                                      \
			(.protector = Z_MUTEX_INITIALIZER(name.protector),)                        \
		) }

int st25dv_probe(struct st25dv_data *data);
int st25dv_unlock(struct st25dv_data *data, const uint8_t pass[8]);
int st25dv_init(struct st25dv_data *data);
int st25dv_deinit(struct st25dv_data *data);
int st25dv_mode_get(struct st25dv_data *data);
int st25dv_mode_set(struct st25dv_data *data, enum st25dv_mode mode);
int st25dv_mailbox_send(struct st25dv_data *data, const void *buf, size_t buf_len);
void st25dv_set_callback(struct st25dv_data *data, st25dv_on_event event_handler, void *ctx);
int st25dv_tag_type5_add(struct st25dv_data *data, const void *buf, size_t buf_len);
int st25dv_tag_type5_rm(struct st25dv_data *data);

#endif /* ST25DV_H */

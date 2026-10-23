/*
 * Copyright (c) 2026 Telink Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr ST25DVXXKC mailbox driver implementation
 */

#include "st25dv.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/logging/log.h>

#define MODULE_NAME st25dv
LOG_MODULE_REGISTER(MODULE_NAME, CONFIG_ST25DV_LOG_LEVEL);

#define ST25DV_USER_MEMORY_ADDR     0x53
#define ST25DV_SYSTEM_AREA_ADDR     0x57
#define ST25DV_FLASH_UPDATE_TIME_MS 10
#define ST25DV_FLASH_BLOCK_SIZE     16
#define ST25DV_MBOX_LENGTH_MAX      256

#define ST25DV_REG_SYS_GPO_1        0x0000
#define ST25DV_REG_SYS_GPO_2        0x0001
#define ST25DV_REG_SYS_RF_MNGT      0x0003
#define ST25DV_REG_SYS_MB_MODE      0x000d
#define ST25DV_REG_SYS_MEM_SIZE_LSB 0x0014
#define ST25DV_REG_SYS_MEM_SIZE_MSB 0x0015
#define ST25DV_REG_SYS_IC_REF       0x0017
#define ST25DV_REG_SYS_IC_REV       0x0020
#define ST25DV_REG_SYS_I2C_PWD      0x0900
#define ST25DV_REG_USR_GPO_DYN      0x2000
#define ST25DV_REG_USR_RF_MNGT_DYN  0x2003
#define ST25DV_REG_USR_SSM_ST_DYN   0x2004
#define ST25DV_REG_USR_IT_STS_DYN   0x2005
#define ST25DV_REG_USR_MB_CTRL_DYN  0x2006
#define ST25DV_REG_USR_MB_LEN_DYN   0x2007
#define ST25DV_REG_USR_MB_DATA_DYN  0x2008

static const char *st25dv_module_name = STRINGIFY(MODULE_NAME);
static const uint8_t st25dv_chip_ref[] = {0x51, 0x50};

static struct k_work_q st25dv_workq;

static inline int st25dv_probe_internal(struct st25dv_data *data);
static inline int st25dv_unlock_internal(struct st25dv_data *data, const uint8_t pass[8]);
static inline int st25dv_is_locked(struct st25dv_data *data);
static inline int st25dv_update_registers(struct st25dv_data *data);
static inline int st25dv_reset(struct st25dv_data *data);
static inline int st25dv_type5_tag_precondition(struct st25dv_data *data, size_t *mem_len,
						size_t *tag_len);
static inline int st25dv_mode_get_internal(struct st25dv_data *data);
static inline int st25dv_mode_set_internal(struct st25dv_data *data, enum st25dv_mode mode);
static void st25dv_isr_handler(const struct device *port, struct gpio_callback *cb,
			       gpio_port_pins_t pins);
static void st25dv_isr_work_handler(struct k_work *item);

int st25dv_probe(struct st25dv_data *data)
{
	int result = -ENODEV;

	IF_ENABLED(CONFIG_ST25DV_MULTITHREAD_SUPPORT,
		((void)k_mutex_lock(&data->protector, K_FOREVER);)
	)
	do {
		LOG_DBG("%s %s", st25dv_module_name, __func__);
		if (!device_is_ready(data->i2c_bus)) {
			LOG_ERR("%s device %s not ready", st25dv_module_name, data->i2c_bus->name);
			break;
		}
		result = st25dv_probe_internal(data);
	} while (0);
	IF_ENABLED(CONFIG_ST25DV_MULTITHREAD_SUPPORT,
		((void)k_mutex_unlock(&data->protector);)
	)
	return result;
}

int st25dv_unlock(struct st25dv_data *data, const uint8_t pass[8])
{
	int result = -ENODEV;

	IF_ENABLED(CONFIG_ST25DV_MULTITHREAD_SUPPORT,
		((void)k_mutex_lock(&data->protector, K_FOREVER);)
	)
	do {
		LOG_DBG("%s %s", st25dv_module_name, __func__);
		if (!device_is_ready(data->i2c_bus)) {
			LOG_ERR("%s device %s not ready", st25dv_module_name, data->i2c_bus->name);
			break;
		}
		result = st25dv_unlock_internal(data, pass);
	} while (0);
	IF_ENABLED(CONFIG_ST25DV_MULTITHREAD_SUPPORT,
		((void)k_mutex_unlock(&data->protector);)
	)
	return result;
}

int st25dv_init(struct st25dv_data *data)
{
	int result = -EALREADY;

	IF_ENABLED(CONFIG_ST25DV_MULTITHREAD_SUPPORT,
		((void)k_mutex_lock(&data->protector, K_FOREVER);)
	)
	do {
		LOG_DBG("%s %s", st25dv_module_name, __func__);
		if (data->inited) {
			break;
		}
		result = -ENODEV;
		if (!device_is_ready(data->i2c_bus)) {
			LOG_ERR("%s device %s not ready", st25dv_module_name, data->i2c_bus->name);
			break;
		}
		if (!device_is_ready(data->isr_port)) {
			LOG_ERR("%s device %s not ready", st25dv_module_name, data->isr_port->name);
			break;
		}
		result = st25dv_probe_internal(data);
		if (result) {
			LOG_ERR("%s not detected on i2c bus %d", st25dv_module_name, result);
			break;
		}
		result = st25dv_unlock_internal(data, ST25DV_DEF_PASS);
		if (result < 0) {
			LOG_ERR("%s unlock failed %d", st25dv_module_name, result);
			break;
		}
		if (result) {
			result = -EACCES;
			LOG_ERR("%s invalid device password", st25dv_module_name);
			break;
		}
		result = st25dv_update_registers(data);
		if (result) {
			LOG_ERR("%s update registers failed %d", st25dv_module_name, result);
			break;
		}
		result = st25dv_reset(data);
		if (result) {
			LOG_ERR("%s reset failed %d", st25dv_module_name, result);
			break;
		}
		data->event_handler = NULL;
		data->event_ctx = NULL;
		k_work_init(&data->isr_work, st25dv_isr_work_handler);
		result = gpio_pin_configure(data->isr_port, data->isr_port_pin,
					    GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_LOW);
		if (result) {
			LOG_ERR("%s gpio setup failed %d", st25dv_module_name, result);
			break;
		}
		gpio_init_callback(&data->gpio_cb, st25dv_isr_handler, BIT(data->isr_port_pin));
		result = gpio_add_callback(data->isr_port, &data->gpio_cb);
		if (result) {
			LOG_ERR("%s add gpio callback failed %d", st25dv_module_name, result);
			break;
		}
		result = gpio_pin_interrupt_configure(data->isr_port, data->isr_port_pin,
						      GPIO_INT_EDGE_TO_ACTIVE);
		if (result) {
			LOG_ERR("%s gpio isr setup failed %d", st25dv_module_name, result);
			break;
		}
		result = i2c_write(data->i2c_bus,
				   (uint8_t[]){
					   (uint8_t)(ST25DV_REG_USR_GPO_DYN >> 8),
					   (uint8_t)ST25DV_REG_USR_GPO_DYN,
					   0x01, /* bit 0: GPO_EN = 1 */
				   },
				   3, ST25DV_USER_MEMORY_ADDR);
		if (result) {
			LOG_ERR("%s gpo register config failed %d", st25dv_module_name, result);
			break;
		}
		data->inited = true;
		LOG_DBG("%s init ok", st25dv_module_name);
	} while (0);
	IF_ENABLED(CONFIG_ST25DV_MULTITHREAD_SUPPORT,
		((void)k_mutex_unlock(&data->protector);)
	)
	return result;
}

int st25dv_deinit(struct st25dv_data *data)
{
	int result = -EALREADY;

	IF_ENABLED(CONFIG_ST25DV_MULTITHREAD_SUPPORT,
		((void)k_mutex_lock(&data->protector, K_FOREVER);)
	)
	do {
		LOG_DBG("%s %s", st25dv_module_name, __func__);
		if (!data->inited) {
			break;
		}
		result = st25dv_reset(data);
		if (result) {
			LOG_ERR("%s reset failed", st25dv_module_name);
			break;
		}
		result = gpio_pin_interrupt_configure(data->isr_port, data->isr_port_pin,
						      GPIO_INT_DISABLE);
		if (result) {
			LOG_ERR("%s gpio isr disable failed", st25dv_module_name);
			break;
		}
		result = gpio_remove_callback(data->isr_port, &data->gpio_cb);
		if (result) {
			LOG_ERR("%s rm gpio callback failed", st25dv_module_name);
			break;
		}
		result = gpio_pin_configure(data->isr_port, data->isr_port_pin, GPIO_DISCONNECTED);
		if (result) {
			LOG_ERR("%s gpio disable failed", st25dv_module_name);
			break;
		}

		struct k_work_sync sync_handle;

		(void)k_work_cancel_sync(&data->isr_work, &sync_handle);
		data->inited = false;
		LOG_DBG("%s deinit ok", st25dv_module_name);
	} while (0);
	IF_ENABLED(CONFIG_ST25DV_MULTITHREAD_SUPPORT,
		((void)k_mutex_unlock(&data->protector);)
	)
	return result;
}

int st25dv_mode_get(struct st25dv_data *data)
{
	int result = -ENODEV;

	IF_ENABLED(CONFIG_ST25DV_MULTITHREAD_SUPPORT,
		((void)k_mutex_lock(&data->protector, K_FOREVER);)
	)
	do {
		LOG_DBG("%s %s", st25dv_module_name, __func__);
		if (!data->inited) {
			LOG_DBG("%s not inited", st25dv_module_name);
			break;
		}
		result = st25dv_mode_get_internal(data);
	} while (0);
	IF_ENABLED(CONFIG_ST25DV_MULTITHREAD_SUPPORT,
		((void)k_mutex_unlock(&data->protector);)
	)

	return result;
}

int st25dv_mode_set(struct st25dv_data *data, enum st25dv_mode mode)
{
	int result = -ENODEV;

	IF_ENABLED(CONFIG_ST25DV_MULTITHREAD_SUPPORT,
		((void)k_mutex_lock(&data->protector, K_FOREVER);)
	)
	do {
		LOG_DBG("%s %s", st25dv_module_name, __func__);
		if (!data->inited) {
			LOG_DBG("%s not inited", st25dv_module_name);
			break;
		}
		result = st25dv_mode_set_internal(data, mode);
	} while (0);
	IF_ENABLED(CONFIG_ST25DV_MULTITHREAD_SUPPORT,
		((void)k_mutex_unlock(&data->protector);)
	)

	return result;
}

int st25dv_mailbox_send(struct st25dv_data *data, const void *buf, size_t buf_len)
{
	int result = -EINVAL;

	IF_ENABLED(CONFIG_ST25DV_MULTITHREAD_SUPPORT,
		((void)k_mutex_lock(&data->protector, K_FOREVER);)
	)
	do {
		LOG_DBG("%s %s", st25dv_module_name, __func__);
		if (!buf || !buf_len || buf_len > ST25DV_MBOX_LENGTH_MAX) {
			LOG_ERR("%s invalid data send", st25dv_module_name);
			break;
		}
		if (!data->inited) {
			result = -ENODEV;
			LOG_ERR("%s not inited", st25dv_module_name);
			break;
		}

		uint8_t ctrl_reg;

		result = i2c_write_read(data->i2c_bus, ST25DV_USER_MEMORY_ADDR,
					(uint8_t[]){
						(uint8_t)(ST25DV_REG_USR_MB_CTRL_DYN >> 8),
						(uint8_t)ST25DV_REG_USR_MB_CTRL_DYN,
					},
					2, &ctrl_reg, sizeof(ctrl_reg));
		if (result) {
			LOG_ERR("%s mailbox status failed %d", st25dv_module_name, result);
			break;
		}
		if (!(ctrl_reg & 0x01)) { /* bit 0: MB_EN = 1 */
			result = -EIO;
			LOG_ERR("%s mailbox disabled", st25dv_module_name);
			break;
		}
		if (ctrl_reg & 0x06) { /* bit 1: HOST_PUT_MSG = 1, bit 2: RF_PUT_MSG = 1 */
			result = -EBUSY;
			LOG_ERR("%s mailbox busy %x", st25dv_module_name, ctrl_reg & 0x06);
			break;
		}

		uint8_t mbox_data[2 + buf_len];

		mbox_data[0] = (uint8_t)(ST25DV_REG_USR_MB_DATA_DYN >> 8);
		mbox_data[1] = (uint8_t)ST25DV_REG_USR_MB_DATA_DYN;
		memcpy(&mbox_data[2], buf, buf_len);
		result = i2c_write(data->i2c_bus, mbox_data, sizeof(mbox_data),
				   ST25DV_USER_MEMORY_ADDR);
		if (result) {
			LOG_ERR("%s mailbox send failed %d", st25dv_module_name, result);
			break;
		}
		LOG_DBG("%s mailbox send ok", st25dv_module_name);
	} while (0);
	IF_ENABLED(CONFIG_ST25DV_MULTITHREAD_SUPPORT,
		((void)k_mutex_unlock(&data->protector);)
	)
	return result;
}

void st25dv_set_callback(struct st25dv_data *data, st25dv_on_event event_handler, void *ctx)
{
	LOG_DBG("%s %s", st25dv_module_name, __func__);
	IF_ENABLED(CONFIG_ST25DV_MULTITHREAD_SUPPORT,
		((void)k_mutex_lock(&data->protector, K_FOREVER);)
	)
	data->event_handler = event_handler;
	data->event_ctx = ctx;
	IF_ENABLED(CONFIG_ST25DV_MULTITHREAD_SUPPORT,
		((void)k_mutex_unlock(&data->protector);)
	)
}

int st25dv_tag_type5_add(struct st25dv_data *data, const void *buf, size_t buf_len)
{
	int result = -EINVAL;

	IF_ENABLED(CONFIG_ST25DV_MULTITHREAD_SUPPORT,
		((void)k_mutex_lock(&data->protector, K_FOREVER);)
	)
	do {
		LOG_DBG("%s %s", st25dv_module_name, __func__);
		if (!buf || !buf_len || buf_len > 0xffff) {
			LOG_ERR("%s invalid data send", st25dv_module_name);
			break;
		}

		size_t mem_size, tag_size;

		result = st25dv_type5_tag_precondition(data, &mem_size, &tag_size);
		if (result) {
			LOG_ERR("%s tag preconditions failed %d", st25dv_module_name, result);
			break;
		}

		uint8_t header[12];
		size_t header_len;

		if (mem_size > 2044) {
			header[0] = 0xe2;                                 /* magic number     */
			header[1] = 0x40;                                 /* version & access */
			header[2] = 0x00;                                 /* zero             */
			header[3] = 0x09;                                 /* add feature info */
			header[4] = 0x00;                                 /* rfu              */
			header[5] = 0x00;                                 /* rfu              */
			header[6] = (uint8_t)((mem_size - 8) >> (3 + 8)); /* mlen msb         */
			header[7] = (uint8_t)((mem_size - 8) >> 3);       /* mlen lsb         */
			header_len = 8;

		} else {
			header[0] = 0xe1;                           /* magic number     */
			header[1] = 0x40;                           /* version & access */
			header[2] = (uint8_t)((mem_size - 4) >> 3); /* mlen             */
			header[3] = 0x09;                           /* add feature info */
			header_len = 4;
		}
		if (buf_len < 0xff) {
			header[header_len] = 0x03;
			header[header_len + 1] = (uint8_t)buf_len;
			header_len += 2;
		} else {
			header[header_len] = 0x03;
			header[header_len + 1] = 0xff;
			header[header_len + 2] = (uint8_t)(buf_len >> 8);
			header[header_len + 3] = (uint8_t)buf_len;
			header_len += 4;
		}
		if (header_len + buf_len > mem_size) {
			LOG_ERR("%s tag data size %u overflow, available %u", st25dv_module_name,
				header_len + buf_len, mem_size);
			result = -ENOMEM;
			break;
		}

		enum st25dv_mode mode_bkp = st25dv_mode_get_internal(data);

		result = st25dv_mode_set_internal(data, ST25DV_MODE_TAG);
		if (result) {
			LOG_ERR("%s mode tag set failed %d", st25dv_module_name, result);
			break;
		}

		size_t addr = 0;

		while (buf_len) {

			uint8_t wr_buff[2 + 128];
			uint8_t *ptr = &wr_buff[2];
			uint8_t wr_len = buf_len > 128 - header_len ? 128 - header_len : buf_len;

			wr_buff[0] = (uint8_t)(addr >> 8);
			wr_buff[1] = (uint8_t)addr;
			memcpy(ptr, header, header_len);
			ptr += header_len;
			memcpy(ptr, buf, wr_len);
			ptr += wr_len;
			memset(ptr, 0x00, 128 - wr_len - header_len);
			result = i2c_write(data->i2c_bus, wr_buff, sizeof(wr_buff),
					   ST25DV_USER_MEMORY_ADDR);
			if (result) {
				LOG_ERR("%s write tag failed %d", st25dv_module_name, result);
				break;
			}
			k_msleep(ST25DV_FLASH_UPDATE_TIME_MS * 128 / ST25DV_FLASH_BLOCK_SIZE);
			header_len = 0;
			buf_len -= wr_len;
			buf = (uint8_t *)buf + wr_len;
			addr += 128;
		}
		while (!buf_len && addr < tag_size) {

			uint8_t wr_buff[2 + 128];

			wr_buff[0] = (uint8_t)(addr >> 8);
			wr_buff[1] = (uint8_t)addr;
			memset(&wr_buff[2], 0x00, 128);
			result = i2c_write(data->i2c_bus, wr_buff, sizeof(wr_buff),
					   ST25DV_USER_MEMORY_ADDR);
			if (result) {
				LOG_ERR("%s erase tag failed %d", st25dv_module_name, result);
				break;
			}
			k_msleep(ST25DV_FLASH_UPDATE_TIME_MS * 128 / ST25DV_FLASH_BLOCK_SIZE);
			addr += 128;
		}
		result = st25dv_mode_set_internal(data, mode_bkp);
		if (result) {
			LOG_ERR("%s mode restore failed %d", st25dv_module_name, result);
			break;
		}
		if (buf_len || addr < tag_size) {
			break;
		}
		LOG_DBG("%s tag wtite ok", st25dv_module_name);
	} while (0);
	IF_ENABLED(CONFIG_ST25DV_MULTITHREAD_SUPPORT,
		((void)k_mutex_unlock(&data->protector);)
	)
	return result;
}

int st25dv_tag_type5_rm(struct st25dv_data *data)
{
	int result;

	IF_ENABLED(CONFIG_ST25DV_MULTITHREAD_SUPPORT,
		((void)k_mutex_lock(&data->protector, K_FOREVER);)
	)
	do {
		LOG_DBG("%s %s", st25dv_module_name, __func__);

		size_t mem_size, tag_size;

		result = st25dv_type5_tag_precondition(data, &mem_size, &tag_size);
		if (result) {
			LOG_ERR("%s tag preconditions failed %d", st25dv_module_name, result);
			break;
		}

		enum st25dv_mode mode_bkp = st25dv_mode_get_internal(data);

		result = st25dv_mode_set_internal(data, ST25DV_MODE_TAG);
		if (result) {
			LOG_ERR("%s mode tag set failed %d", st25dv_module_name, result);
			break;
		}

		size_t addr = 0;

		while (addr < tag_size) {

			uint8_t wr_buff[2 + 128];

			wr_buff[0] = (uint8_t)(addr >> 8);
			wr_buff[1] = (uint8_t)addr;
			memset(&wr_buff[2], 0x00, 128);
			result = i2c_write(data->i2c_bus, wr_buff, sizeof(wr_buff),
					   ST25DV_USER_MEMORY_ADDR);
			if (result) {
				LOG_ERR("%s erase tag failed %d", st25dv_module_name, result);
				break;
			}
			k_msleep(ST25DV_FLASH_UPDATE_TIME_MS * 128 / ST25DV_FLASH_BLOCK_SIZE);
			addr += 128;
		}
		result = st25dv_mode_set_internal(data, mode_bkp);
		if (result) {
			LOG_ERR("%s mode restore failed %d", st25dv_module_name, result);
			break;
		}
		if (addr < tag_size) {
			break;
		}
		LOG_DBG("%s tag erase ok", st25dv_module_name);
	} while (0);
	IF_ENABLED(CONFIG_ST25DV_MULTITHREAD_SUPPORT,
		((void)k_mutex_unlock(&data->protector);)
	)
	return result;
}

static inline int st25dv_probe_internal(struct st25dv_data *data)
{
	int result;

	do {
		LOG_DBG("%s %s", st25dv_module_name, __func__);

		uint8_t chip_ref, chip_rev;

		result = i2c_write_read(data->i2c_bus, ST25DV_SYSTEM_AREA_ADDR,
					(uint8_t[]){
						(uint8_t)(ST25DV_REG_SYS_IC_REF >> 8),
						(uint8_t)ST25DV_REG_SYS_IC_REF,
					},
					2, &chip_ref, sizeof(chip_ref));
		if (result) {
			LOG_ERR("%s read chip id failed %d", st25dv_module_name, result);
			break;
		}
		result = -ENODEV;
		for (size_t i = 0; i < ARRAY_SIZE(st25dv_chip_ref); ++i) {
			if (chip_ref == st25dv_chip_ref[i]) {
				result = 0;
				break;
			}
		}
		if (result) {
			LOG_ERR("%s invalid chip id %02x", st25dv_module_name, chip_ref);
			break;
		}
		result = i2c_write_read(data->i2c_bus, ST25DV_SYSTEM_AREA_ADDR,
					(uint8_t[]){
						(uint8_t)(ST25DV_REG_SYS_IC_REV >> 8),
						(uint8_t)ST25DV_REG_SYS_IC_REV,
					},
					2, &chip_rev, sizeof(chip_rev));
		if (result) {
			LOG_ERR("%s read chip id failed %d", st25dv_module_name, result);
			break;
		}
		LOG_INF("%s chip id %02x rev %02x", st25dv_module_name, chip_ref, chip_rev);
	} while (0);

	return result;
}

static inline int st25dv_unlock_internal(struct st25dv_data *data, const uint8_t pass[8])
{
	int result;

	do {
		result = st25dv_is_locked(data);
		if (result < 0) {
			LOG_ERR("%s unlock read filed %d", st25dv_module_name, result);
			break;
		}
		if (!result) {
			LOG_DBG("%s already unlocked", st25dv_module_name);
			break;
		}

		uint8_t unlock_cmd[19];

		unlock_cmd[0] = (uint8_t)(ST25DV_REG_SYS_I2C_PWD >> 8);
		unlock_cmd[1] = (uint8_t)ST25DV_REG_SYS_I2C_PWD;
		memcpy(&unlock_cmd[2], pass, 8);  /* password */
		unlock_cmd[10] = 0x09;            /* validation code */
		memcpy(&unlock_cmd[11], pass, 8); /* password confirmation */
		result = i2c_write(data->i2c_bus, unlock_cmd, sizeof(unlock_cmd),
				   ST25DV_SYSTEM_AREA_ADDR);
		if (result) {
			LOG_ERR("%s unlock failed %d", st25dv_module_name, result);
			break;
		}
		result = st25dv_is_locked(data);
		if (result < 0) {
			LOG_ERR("%s unlock read filed %d", st25dv_module_name, result);
			break;
		}
		LOG_DBG("%s %s", st25dv_module_name, result ? "locked" : "unlocked");
	} while (0);

	return result;
}

static inline int st25dv_is_locked(struct st25dv_data *data)
{
	int result;

	do {
		uint8_t unlocked;

		result = i2c_write_read(data->i2c_bus, ST25DV_USER_MEMORY_ADDR,
					(uint8_t[]){
						(uint8_t)(ST25DV_REG_USR_SSM_ST_DYN >> 8),
						(uint8_t)ST25DV_REG_USR_SSM_ST_DYN,
					},
					2, &unlocked, sizeof(unlocked));
		if (result) {
			LOG_ERR("%s read unlock status failed %d", st25dv_module_name, result);
			break;
		}
		result = !(bool)unlocked;
	} while (0);

	return result;
}

static inline int st25dv_update_registers(struct st25dv_data *data)
{
	LOG_DBG("%s %s", st25dv_module_name, __func__);

	int result = 0;

	while (!result) {
		uint8_t gpo_1;

		result = i2c_write_read(data->i2c_bus, ST25DV_SYSTEM_AREA_ADDR,
					(uint8_t[]){
						(uint8_t)(ST25DV_REG_SYS_GPO_1 >> 8),
						(uint8_t)ST25DV_REG_SYS_GPO_1,
					},
					2, &gpo_1, sizeof(gpo_1));
		if (result) {
			LOG_ERR("%s gpo 1 read failed %d", st25dv_module_name, result);
			break;
		}
		if (gpo_1 == 0x71) { /* bit 0: GPO_EN = 1,
				      * bit 4: FIELD_CHANGE_EN = 1,
				      * bit 5: RF_PUT_MSG_EN = 1,
				      * bit 6: RF_GET_MSG_EN = 1
				      */
			LOG_DBG("%s gpo 1 mode already set", st25dv_module_name);
			break;
		}
		result = i2c_write(data->i2c_bus,
				   (uint8_t[]){
					   (uint8_t)(ST25DV_REG_SYS_GPO_1 >> 8),
					   (uint8_t)ST25DV_REG_SYS_GPO_1,
					   0x71,
					   /* bit 0: GPO_EN = 1,
					    * bit 4: FIELD_CHANGE_EN = 1,
					    * bit 5: RF_PUT_MSG_EN = 1,
					    * bit 6: RF_GET_MSG_EN = 1,
					    */
				   },
				   3, ST25DV_SYSTEM_AREA_ADDR);
		if (result) {
			LOG_ERR("%s gpo 1 mode set failed %d", st25dv_module_name, result);
			break;
		}
		k_msleep(ST25DV_FLASH_UPDATE_TIME_MS);
		break;
	};

	while (!result) {
		uint8_t gpo_2;

		result = i2c_write_read(data->i2c_bus, ST25DV_SYSTEM_AREA_ADDR,
					(uint8_t[]){
						(uint8_t)(ST25DV_REG_SYS_GPO_2 >> 8),
						(uint8_t)ST25DV_REG_SYS_GPO_2,
					},
					2, &gpo_2, sizeof(gpo_2));
		if (result) {
			LOG_ERR("%s gpo 2 read failed %d", st25dv_module_name, result);
			break;
		}
		if (gpo_2 == 0x0c) { /* default value (pulse 413.95uS) */
			LOG_DBG("%s gpo 2 mode already set", st25dv_module_name);
			break;
		}
		result = i2c_write(data->i2c_bus,
				   (uint8_t[]){
					   (uint8_t)(ST25DV_REG_SYS_GPO_2 >> 8),
					   (uint8_t)ST25DV_REG_SYS_GPO_2,
					   0x0c, /* default value (pulse 413.95uS) */
				   },
				   3, ST25DV_SYSTEM_AREA_ADDR);
		if (result) {
			LOG_ERR("%s gpo 2 mode set failed %d", st25dv_module_name, result);
			break;
		}
		k_msleep(ST25DV_FLASH_UPDATE_TIME_MS);
		break;
	};

	while (!result) {
		uint8_t mb_mode;

		result = i2c_write_read(data->i2c_bus, ST25DV_SYSTEM_AREA_ADDR,
					(uint8_t[]){
						(uint8_t)(ST25DV_REG_SYS_MB_MODE >> 8),
						(uint8_t)ST25DV_REG_SYS_MB_MODE,
					},
					2, &mb_mode, sizeof(mb_mode));
		if (result) {
			LOG_ERR("%s mailbox mode read failed %d", st25dv_module_name, result);
			break;
		}
		if (mb_mode & 0x01) { /* bit 0: MB_EN = 1 */
			LOG_DBG("%s mailbox mode already enabled", st25dv_module_name);
			break;
		}
		result = i2c_write(data->i2c_bus,
				   (uint8_t[]){
					   (uint8_t)(ST25DV_REG_SYS_MB_MODE >> 8),
					   (uint8_t)ST25DV_REG_SYS_MB_MODE,
					   0x01, /* bit 0: MB_EN = 1 */
				   },
				   3, ST25DV_SYSTEM_AREA_ADDR);
		if (result) {
			LOG_ERR("%s mailbox mode set failed %d", st25dv_module_name, result);
			break;
		}
		k_msleep(ST25DV_FLASH_UPDATE_TIME_MS);
		break;
	};

	while (!result) {
		uint8_t rf_state;

		result = i2c_write_read(data->i2c_bus, ST25DV_SYSTEM_AREA_ADDR,
					(uint8_t[]){
						(uint8_t)(ST25DV_REG_SYS_RF_MNGT >> 8),
						(uint8_t)ST25DV_REG_SYS_RF_MNGT,
					},
					2, &rf_state, sizeof(rf_state));
		if (result) {
			LOG_ERR("%s radio state read failed %d", st25dv_module_name, result);
			break;
		}
		if (!rf_state) { /* bit 0: RF_DISABLE = 1, bit 1: RF_SLEEP = 1 */
			LOG_DBG("%s radio state already set", st25dv_module_name);
			break;
		}
		result = i2c_write(data->i2c_bus,
				   (uint8_t[]){
					   (uint8_t)(ST25DV_REG_SYS_RF_MNGT >> 8),
					   (uint8_t)ST25DV_REG_SYS_RF_MNGT,
					   0x00, /* bit 0: RF_DISABLE = 1, bit 1: RF_SLEEP = 1 */
				   },
				   3, ST25DV_SYSTEM_AREA_ADDR);
		if (result) {
			LOG_ERR("%s radio state set failed %d", st25dv_module_name, result);
			break;
		}
		k_msleep(ST25DV_FLASH_UPDATE_TIME_MS);
		break;
	};

	return result;
}

static inline int st25dv_reset(struct st25dv_data *data)
{
	int result;

	do {
		LOG_DBG("%s %s", st25dv_module_name, __func__);
		result = i2c_write(data->i2c_bus,
				   (uint8_t[]){
					   (uint8_t)(ST25DV_REG_USR_RF_MNGT_DYN >> 8),
					   (uint8_t)ST25DV_REG_USR_RF_MNGT_DYN,
					   0x02, /* bit 1: RF_SLEEP = 1 */
				   },
				   3, ST25DV_USER_MEMORY_ADDR);
		if (result) {
			LOG_ERR("%s reset radio state failed %d", st25dv_module_name, result);
			break;
		}
		result = i2c_write(data->i2c_bus,
				   (uint8_t[]){
					   (uint8_t)(ST25DV_REG_USR_GPO_DYN >> 8),
					   (uint8_t)ST25DV_REG_USR_GPO_DYN,
					   0x00, /* bit 7: GPO_EN = 1 */
				   },
				   3, ST25DV_USER_MEMORY_ADDR);
		if (result) {
			LOG_ERR("%s clear isr config failed %d", st25dv_module_name, result);
			break;
		}
		result = i2c_write(data->i2c_bus,
				   (uint8_t[]){
					   (uint8_t)(ST25DV_REG_USR_MB_CTRL_DYN >> 8),
					   (uint8_t)ST25DV_REG_USR_MB_CTRL_DYN,
					   0x00, /* bit 0: MB_EN = 1 */
				   },
				   3, ST25DV_USER_MEMORY_ADDR);
		if (result) {
			LOG_ERR("%s mailbox switching off failed %d", st25dv_module_name, result);
			break;
		}
	} while (0);

	return result;
}

static inline int st25dv_type5_tag_precondition(struct st25dv_data *data, size_t *mem_len,
						size_t *tag_len)
{
	int result = -ENODEV;

	do {
		LOG_DBG("%s %s", st25dv_module_name, __func__);
		if (!data->inited) {
			LOG_ERR("%s mailbox mode active", st25dv_module_name);
			break;
		}

		uint8_t mem_size_lsb, mem_size_msb;

		result = i2c_write_read(data->i2c_bus, ST25DV_SYSTEM_AREA_ADDR,
					(uint8_t[]){
						(uint8_t)(ST25DV_REG_SYS_MEM_SIZE_LSB >> 8),
						(uint8_t)ST25DV_REG_SYS_MEM_SIZE_LSB,
					},
					2, &mem_size_lsb, sizeof(mem_size_lsb));
		if (result) {
			LOG_ERR("%s read memory size lsb failed %d", st25dv_module_name, result);
			break;
		}
		result = i2c_write_read(data->i2c_bus, ST25DV_SYSTEM_AREA_ADDR,
					(uint8_t[]){
						(uint8_t)(ST25DV_REG_SYS_MEM_SIZE_MSB >> 8),
						(uint8_t)ST25DV_REG_SYS_MEM_SIZE_MSB,
					},
					2, &mem_size_msb, sizeof(mem_size_msb));
		if (result) {
			LOG_ERR("%s read memory size msb failed %d", st25dv_module_name, result);
			break;
		}
		*mem_len = ((((size_t)mem_size_msb << 8) | (mem_size_lsb)) + 1) << 2;
		LOG_INF("%s memory size %u bytes", st25dv_module_name, *mem_len);

		uint8_t header_len_data;

		result = i2c_write_read(data->i2c_bus, ST25DV_USER_MEMORY_ADDR,
					(uint8_t[]){
						0x00,
						0x02,
					},
					2, /* zero 8 bytes CC , nonzero 4 bytes CC */
					&header_len_data, sizeof(header_len_data));
		if (result) {
			LOG_ERR("%s read cc length failed %d", st25dv_module_name, result);
			break;
		}
		*tag_len = !header_len_data ? 8 : 4;

		uint8_t tag_len_data[3];

		result = i2c_write_read(data->i2c_bus, ST25DV_USER_MEMORY_ADDR,
					(uint8_t[]){
						0x00,
						(uint8_t)((*tag_len) + 1),
					},
					2, /* tlv starts at cc length, so tlv length at offset 1 */
					tag_len_data, sizeof(tag_len_data));
		if (result) {
			LOG_ERR("%s read tag length failed %d", st25dv_module_name, result);
			break;
		}
		*tag_len += tag_len_data[0] == 0xff ? 4 : 2; /* tag and length */
		*tag_len += tag_len_data[0] == 0xff
				    ? ((size_t)(tag_len_data[1] << 8) | tag_len_data[2])
				    : tag_len_data[0];
		*tag_len = MIN(*tag_len, *mem_len); /* probably malformed tag data */
		LOG_INF("%s tag size %u bytes", st25dv_module_name, *tag_len);
	} while (0);

	return result;
}

static inline int st25dv_mode_get_internal(struct st25dv_data *data)
{
	int result;

	do {
		LOG_DBG("%s %s", st25dv_module_name, __func__);

		uint8_t radio_state;

		result = i2c_write_read(data->i2c_bus, ST25DV_USER_MEMORY_ADDR,
					(uint8_t[]){
						(uint8_t)(ST25DV_REG_USR_RF_MNGT_DYN >> 8),
						(uint8_t)ST25DV_REG_USR_RF_MNGT_DYN,
					},
					2, &radio_state, sizeof(radio_state));
		if (result) {
			LOG_ERR("%s radio state read failed %d", st25dv_module_name, result);
			break;
		}
		if (radio_state & 0x02) { /* bit 1: RF_SLEEP = 1 */
			result = ST25DV_MODE_OFF;
			break;
		}

		uint8_t ctrl_reg;

		result = i2c_write_read(data->i2c_bus, ST25DV_USER_MEMORY_ADDR,
					(uint8_t[]){
						(uint8_t)(ST25DV_REG_USR_MB_CTRL_DYN >> 8),
						(uint8_t)ST25DV_REG_USR_MB_CTRL_DYN,
					},
					2, &ctrl_reg, sizeof(ctrl_reg));
		if (result) {
			LOG_ERR("%s mailbox status failed %d", st25dv_module_name, result);
			break;
		}
		if (ctrl_reg & 0x01) { /* bit 0: MB_EN = 1 */
			result = ST25DV_MODE_MAILBOX;
			break;
		}
		result = ST25DV_MODE_TAG;
	} while (0);

	return result;
}

static inline int st25dv_mode_set_internal(struct st25dv_data *data, enum st25dv_mode mode)
{
	int result = -EINVAL;

	LOG_DBG("%s %s", st25dv_module_name, __func__);
	switch (mode) {
	case ST25DV_MODE_OFF:
		result = i2c_write(data->i2c_bus,
				   (uint8_t[]){
					   (uint8_t)(ST25DV_REG_USR_RF_MNGT_DYN >> 8),
					   (uint8_t)ST25DV_REG_USR_RF_MNGT_DYN,
					   0x02, /* bit 1: RF_SLEEP = 1 */
				   },
				   3, ST25DV_USER_MEMORY_ADDR);
		if (result) {
			LOG_ERR("%s disable radio failed %d", st25dv_module_name, result);
			break;
		}
		LOG_DBG("%s mode off", st25dv_module_name);
		break;
	case ST25DV_MODE_TAG:
		result = i2c_write(data->i2c_bus,
				   (uint8_t[]){
					   (uint8_t)(ST25DV_REG_USR_RF_MNGT_DYN >> 8),
					   (uint8_t)ST25DV_REG_USR_RF_MNGT_DYN,
					   0x00, /* bit 1: RF_SLEEP = 1 */
				   },
				   3, ST25DV_USER_MEMORY_ADDR);
		if (result) {
			LOG_ERR("%s enable radio failed %d", st25dv_module_name, result);
			break;
		}
		result = i2c_write(data->i2c_bus,
				   (uint8_t[]){
					   (uint8_t)(ST25DV_REG_USR_MB_CTRL_DYN >> 8),
					   (uint8_t)ST25DV_REG_USR_MB_CTRL_DYN,
					   0x00, /* bit 0: MB_EN = 1 */
				   },
				   3, ST25DV_USER_MEMORY_ADDR);
		if (result) {
			LOG_ERR("%s disable mailbox failed %d", st25dv_module_name, result);
			break;
		}
		LOG_DBG("%s mode tag", st25dv_module_name);
		break;
	case ST25DV_MODE_MAILBOX:
		result = i2c_write(data->i2c_bus,
				   (uint8_t[]){
					   (uint8_t)(ST25DV_REG_USR_RF_MNGT_DYN >> 8),
					   (uint8_t)ST25DV_REG_USR_RF_MNGT_DYN,
					   0x00, /* bit 1: RF_SLEEP = 1 */
				   },
				   3, ST25DV_USER_MEMORY_ADDR);
		if (result) {
			LOG_ERR("%s enable radio failed %d", st25dv_module_name, result);
			break;
		}
		result = i2c_write(data->i2c_bus,
				   (uint8_t[]){
					   (uint8_t)(ST25DV_REG_USR_MB_CTRL_DYN >> 8),
					   (uint8_t)ST25DV_REG_USR_MB_CTRL_DYN,
					   0x01, /* bit 0: MB_EN = 1 */
				   },
				   3, ST25DV_USER_MEMORY_ADDR);
		if (result) {
			LOG_ERR("%s enable mailbox failed %d", st25dv_module_name, result);
			break;
		}
		LOG_DBG("%s mode mailbox", st25dv_module_name);
		break;
	default:
		LOG_ERR("%s invalid mode %u", st25dv_module_name, mode);
	}

	return result;
}

static void st25dv_isr_handler(const struct device *port, struct gpio_callback *cb,
			       gpio_port_pins_t pins)
{
	struct st25dv_data *data = CONTAINER_OF(cb, struct st25dv_data, gpio_cb);

	/* LOG_DBG("%s %s", st25dv_module_name, __func__); */

	int result = k_work_submit_to_queue(&st25dv_workq, &data->isr_work);

	if (result < 0) {
		LOG_ERR("%s failed to submit isr work %d", st25dv_module_name, result);
	}
}

static void st25dv_isr_work_handler(struct k_work *item)
{
	struct st25dv_data *data = CONTAINER_OF(item, struct st25dv_data, isr_work);
	int result = 0;
	uint8_t it_status;

	/* LOG_DBG("%s %s", st25dv_module_name, __func__); */
	IF_ENABLED(CONFIG_ST25DV_MULTITHREAD_SUPPORT,
		((void)k_mutex_lock(&data->protector, K_FOREVER);)
	)
	while (!result) {
		result = i2c_write_read(data->i2c_bus, ST25DV_USER_MEMORY_ADDR,
					(uint8_t[]){
						(uint8_t)(ST25DV_REG_USR_IT_STS_DYN >> 8),
						(uint8_t)ST25DV_REG_USR_IT_STS_DYN,
					},
					2, &it_status, sizeof(it_status));
		if (result) {
			LOG_ERR("%s isr status read failed %d", st25dv_module_name, result);
			break;
		}
		break;
	}

	while (!result && it_status & 0x08) { /* bit 3: FIELD_FALLING = 1 */
		LOG_DBG("%s rf field falling", st25dv_module_name);
		if (data->event_handler) {
			data->event_handler(ST25DV_EVENT_FIELD_OFF, data->event_ctx, NULL, 0);
		}
		break;
	}

	while (!result && it_status & 0x10) { /* bit 4: FIELD_RISING = 1 */
		LOG_DBG("%s rf field rising", st25dv_module_name);
		if (data->event_handler) {
			data->event_handler(ST25DV_EVENT_FIELD_ON, data->event_ctx, NULL, 0);
		}
		break;
	}

	while (!result && it_status & 0x40) { /* bit 6: RF_GET_MSG = 1 */
		LOG_DBG("%s data sent", st25dv_module_name);
		if (data->event_handler) {
			data->event_handler(ST25DV_EVENT_DATA_SENT, data->event_ctx, NULL, 0);
		}
		break;
	}

	while (!result && it_status & 0x20) { /* bit 5: RF_PUT_MSG = 1 */
		LOG_DBG("%s data received", st25dv_module_name);

		int r;
		uint8_t buf_len;

		r = i2c_write_read(data->i2c_bus, ST25DV_USER_MEMORY_ADDR,
				   (uint8_t[]){
					   (uint8_t)(ST25DV_REG_USR_MB_LEN_DYN >> 8),
					   (uint8_t)ST25DV_REG_USR_MB_LEN_DYN,
				   },
				   2, &buf_len, sizeof(buf_len));
		if (r) {
			LOG_ERR("%s mailbox read length failed %d", st25dv_module_name, r);
			break;
		}

		uint8_t mbox_data[(size_t)buf_len + 1];

		r = i2c_write_read(data->i2c_bus, ST25DV_USER_MEMORY_ADDR,
				   (uint8_t[]){
					   (uint8_t)(ST25DV_REG_USR_MB_DATA_DYN >> 8),
					   (uint8_t)ST25DV_REG_USR_MB_DATA_DYN,
				   },
				   2, mbox_data, sizeof(mbox_data));
		if (r) {
			LOG_ERR("%s mailbox read failed %d", st25dv_module_name, r);
			break;
		}
		LOG_HEXDUMP_DBG(mbox_data, sizeof(mbox_data), st25dv_module_name);
		if (data->event_handler) {
			data->event_handler(ST25DV_EVENT_DATA_RECEIVED, data->event_ctx, mbox_data,
					    sizeof(mbox_data));
		}
		break;
	}

	while (!result && it_status & 0x87) { /* other interrupts */
		LOG_WRN("%s unhandled interrupt %02x", st25dv_module_name, it_status & 0x9F);
		break;
	}
	IF_ENABLED(CONFIG_ST25DV_MULTITHREAD_SUPPORT,
		((void)k_mutex_unlock(&data->protector);)
	)
}

static int st25dv_init_workq(void)
{
	static K_THREAD_STACK_DEFINE(st25dv_workq_stack, CONFIG_ST25DV_WORKER_STACK_SIZE);

	LOG_DBG("%s %s", st25dv_module_name, __func__);
	k_work_queue_init(&st25dv_workq);
	k_work_queue_start(&st25dv_workq, st25dv_workq_stack,
			   K_THREAD_STACK_SIZEOF(st25dv_workq_stack), CONFIG_ST25DV_WORKER_PRIORITY,
			   NULL);
	return 0;
}

SYS_INIT(st25dv_init_workq, POST_KERNEL, CONFIG_ST25DV_WORKER_INIT_PRIO);

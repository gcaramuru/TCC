/*
 * SPDX-FileCopyrightText: 2022 Kyunghwan Kwon <k@mononn.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef LIBMCU_BQ25180_OVERRIDES_H
#define LIBMCU_BQ25180_OVERRIDES_H

#if defined(__cplusplus)
extern "C"
{
#endif

#include <stdint.h>
#include <stddef.h>
    #include <zephyr/device.h>
#define BQ25180_NODE DT_NODELABEL(bq25180)
    /**
     * @brief Read a register value via I2C
     *
     * @param[in] addr device address
     * @param[in] reg register address to read from
     * @param[in] buf buffer to get the value in
     * @param[in] bufsize size of @ref buf
     *
     * @return The number of bytes read on success. Otherwise a negative integer
     *         error code
     *
     * @note This function should be implemented for the specific platform or
     *       board. The default one does nothing being linked weak.
     */
    int bq25180_read(uint8_t addr, uint8_t reg, void *buf, size_t bufsize)
    {
        // const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);
        static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(BQ25180_NODE);

        if (!device_is_ready(dev_i2c.bus))
        {
            printk("I2C bus %s is not ready!\n\r", dev_i2c.bus->name);
            return;
        }

        uint8_t write_data[1] = {reg};
        uint8_t read_data[bufsize];
        struct i2c_msg msgs[2];

        msgs[0].buf = write_data;
        msgs[0].len = 1;
        msgs[0].flags = I2C_MSG_WRITE;

        msgs[1].buf = read_data;
        msgs[1].len = bufsize;
        msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

        if (i2c_transfer(dev_i2c.bus, &msgs[0], 2, addr) < 0)
        {
            printk("Error: Unable to write and read data over I2C\n");
            return -1;
        }

        memcpy(buf, read_data, bufsize);

        return 0; // Success
    };

    /**
     * @brief Write a value to a register via I2C
     *
     * @param[in] addr device address
     * @param[in] reg register address to write to
     * @param[in] data data to be written in @ref reg
     * @param[in] data_len length of @ref data
     *
     * @return The number of bytes written on success. Otherwise a negative integer
     *         error code
     *
     * @note This function should be implemented for the specific platform or
     *       board. The default one does nothing being linked weak.
     */
    int bq25180_write(uint8_t addr, uint8_t reg, const void *data, size_t data_len)
    {
        static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(BQ25180_NODE);

        if (!device_is_ready(dev_i2c.bus))
        {
            printk("I2C bus %s is not ready!\n\r", dev_i2c.bus->name);
            return;
        }
        uint8_t i2c_address = addr;
        uint8_t buf[data_len + 1];
        buf[0] = reg;

        if (data_len > 0)
        {
            memcpy(&buf[1], data, data_len);
        }

        struct i2c_msg msg;
        msg.buf = buf;
        msg.len = data_len + 1;
        msg.flags = I2C_MSG_WRITE;

        if (i2c_transfer(dev_i2c.bus, &msg, 1, i2c_address) < 0)
        {
            printk("Error: Unable to write to registers\n");
            return -1;
        }

        printk("Write operation successful\n");

        return 0; // Sucesso
    };

#if defined(__cplusplus)
}
#endif

#endif /* LIBMCU_BQ25180_OVERRIDES_H */
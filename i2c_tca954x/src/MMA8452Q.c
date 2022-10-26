/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <drivers/i2c.h>
#include <usb/usb_device.h>
#include <stdio.h>

#define MUX_ADDR 0x70
#define ACCEL_ADDR1 0x1C
#define ACCEL_ADDR2 0x1D

enum MMA8452Q_Register {
	STATUS = 0x00,
	OUT_X_MSB = 0x01,
	OUT_X_LSB = 0x02,
	OUT_Y_MSB = 0x03,
	OUT_Y_LSB = 0x04,
	OUT_Z_MSB = 0x05,
	OUT_Z_LSB = 0x06,
	SYSMOD = 0x0B,
	INT_SOURCE = 0x0C,
	WHO_AM_I = 0x0D,
	XYZ_DATA_CFG = 0x0E,
	HP_FILTER_CUTOFF = 0x0F,
	PL_STATUS = 0x10,
	PL_CFG = 0x11,
	PL_COUNT = 0x12,
	PL_BF_ZCOMP = 0x13,
	P_L_THS_REG = 0x14,
	FF_MT_CFG = 0x15,
	FF_MT_SRC = 0x16,
	FF_MT_THS = 0x17,
	FF_MT_COUNT = 0x18,
	TRANSIENT_CFG = 0x1D,
	TRANSIENT_SRC = 0x1E,
	TRANSIENT_THS = 0x1F,
	TRANSIENT_COUNT = 0x20,
	PULSE_CFG = 0x21,
	PULSE_SRC = 0x22,
	PULSE_THSX = 0x23,
	PULSE_THSY = 0x24,
	PULSE_THSZ = 0x25,
	PULSE_TMLT = 0x26,
	PULSE_LTCY = 0x27,
	PULSE_WIND = 0x28,
	ASLP_COUNT = 0x29,
	CTRL_REG1 = 0x2A,
	CTRL_REG2 = 0x2B,
	CTRL_REG3 = 0x2C,
	CTRL_REG4 = 0x2D,
	CTRL_REG5 = 0x2E,
	OFF_X = 0x2F,
	OFF_Y = 0x30,
	OFF_Z = 0x31
};

enum MMA8452Q_Scale
{
	SCALE_2G = 2,
	SCALE_4G = 4,
	SCALE_8G = 8
}; // Possible full-scale settings

enum MMA8452Q_ODR
{
	ODR_800,
	ODR_400,
	ODR_200,
	ODR_100,
	ODR_50,
	ODR_12,
	ODR_6,
	ODR_1
}; // possible data rates


enum TCA9548A_CHANNELS {
    MUX_CHANNEL_0 = 0x01,
    MUX_CHANNEL_1 = 0x02,
    MUX_CHANNEL_2 = 0x04,
    MUX_CHANNEL_3 = 0x08,
    MUX_CHANNEL_4 = 0x10,
    MUX_CHANNEL_5 = 0x20,
    MUX_CHANNEL_6 = 0x40,
    MUX_CHANNEL_7 = 0x80,
};

uint8_t read_accel_byte(const struct device *i2c_dev, uint8_t dev_address, uint8_t register_address) {
    // printk("read_accel_byte\n");
    uint8_t read_data;
    int ret = i2c_write_read(i2c_dev, dev_address, &register_address, 1, &read_data, 1);
    if (ret != 0) {
        printk("debug 1\n");
    }
    return read_data;
}

void read_accel_data(const struct device *i2c_dev, uint8_t dev_address, uint8_t register_address, uint8_t *read_data) {
    // printk("read_accel_data\n");
    int ret = i2c_write_read(i2c_dev, dev_address, &register_address, 1, read_data, 6);
    if (ret != 0) {
        printk("debug 2\n");
    }
}

void accel_set_standby(const struct device *i2c_dev, uint8_t accel_address){
    printk("accel_set_standby\n");
    uint8_t c = read_accel_byte(i2c_dev, accel_address, CTRL_REG1);
    uint8_t *write_data = {CTRL_REG1, (c & ~(0x01))};
    int ret = i2c_reg_write_byte(i2c_dev, accel_address, CTRL_REG1, (c & ~(0x01)));
    if (ret != 0) {
        printk("debug 3\n");
    }
}

void accel_set_active(const struct device *i2c_dev, uint8_t accel_address){
    printk("accel_set_active\n");
    uint8_t c = read_accel_byte(i2c_dev, accel_address, CTRL_REG1);
    uint8_t *write_data = {CTRL_REG1, (c | 0x01)};
    int ret = i2c_reg_write_byte(i2c_dev, accel_address, CTRL_REG1, (c | 0x01));
    if (ret != 0) {
        printk("debug 4\n");
    }
}

void accel_set_scale(const struct device *i2c_dev, uint8_t accel_address, int scale) {
    // accel_set_standby(i2c_dev, accel_address);
    printk("accel_set_scale\n");
    printk("SCALE: %d\n", scale);
    uint8_t cfg = read_accel_byte(i2c_dev, accel_address, XYZ_DATA_CFG);

    cfg &= 0xFC;

    cfg |= (scale >> 2);

    uint8_t *write_data = {XYZ_DATA_CFG, cfg};
    int ret = i2c_reg_write_byte(i2c_dev, accel_address, XYZ_DATA_CFG, cfg);
    if (ret != 0) {
        printk("debug 5\n");
    }
}

void accel_set_odr(const struct device *i2c_dev, uint8_t accel_address, int ODR) {
    printk("accel_set_odr\n");
    printk("ODR: %d\n", ODR);
    uint8_t c = read_accel_byte(i2c_dev, accel_address, CTRL_REG1);
    printk("old = %x\n", c);

    c &= 0xC7;
    c |= (ODR << 3);

    uint8_t *write_data = {CTRL_REG1, c};
    int ret = i2c_reg_write_byte(i2c_dev, accel_address, CTRL_REG1, c);
    if (ret != 0) {
        printk("debug 6\n");
    }


    c = read_accel_byte(i2c_dev, accel_address, CTRL_REG1);
    printk("new = %x\n", c);
}

void init_accel(const struct device *i2c_dev, uint8_t dev_address, int scale, int odr) {

    // before adjusting register must be in standy by mode
    accel_set_standby(i2c_dev, dev_address);

    accel_set_scale(i2c_dev, dev_address, scale);
    accel_set_odr(i2c_dev, dev_address, odr);

    // set back to active mode
    accel_set_active(i2c_dev, dev_address);
}

void mux_change_channel(const struct device *i2c_dev, uint8_t channel_cfg) {
    int ret;

    ret = i2c_write(i2c_dev, &channel_cfg, 1, MUX_ADDR);

    if (ret != 0) {
        printk("Mux channel/s change to 0x%x failed.\n", channel_cfg);
    }
}



void main()
{
    usb_enable(NULL);

    k_msleep(5000);
	const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

    bool dev_ready = device_is_ready(i2c_dev);

    uint8_t write_data = 0x02;
    uint8_t read_data = 0x00;

    int ret = i2c_write(i2c_dev, &write_data, 1, MUX_ADDR);

    if (ret != 0) {
        printk("debug channel change failed basic\n");
    }

    ret = i2c_read(i2c_dev, &read_data, 1, MUX_ADDR);

    if (ret != 0) {
        printk("debug channel read failed basic\n");
    }

    // uint8_t *accel_data ;
    // accel_data = malloc(sizeof(uint8_t) * 6);

    uint8_t accel_data[6] = {0};

    uint8_t data[] = {OUT_X_MSB, OUT_X_LSB, OUT_Y_LSB, OUT_X_MSB, OUT_Z_LSB, OUT_Z_MSB};
    int count = 0;


    uint8_t mux_channels[8] = {MUX_CHANNEL_0, MUX_CHANNEL_1, MUX_CHANNEL_2, MUX_CHANNEL_3, MUX_CHANNEL_4, MUX_CHANNEL_5, MUX_CHANNEL_6, MUX_CHANNEL_7};
    
    for (int i = 0; i < 8; i++) {
        mux_change_channel(i2c_dev, mux_channels[i]);
        init_accel(i2c_dev, ACCEL_ADDR1, SCALE_2G, ODR_800);
        init_accel(i2c_dev, ACCEL_ADDR2, SCALE_2G, ODR_800);
    }

    // mux_change_channel(i2c_dev, mux_channels[0]);
    // init_accel(i2c_dev, ACCEL_ADDR1, SCALE_2G, ODR_800);

    while(1) {
        printk("%d start of data collected:\n", count++);
        read_data = read_accel_byte(i2c_dev, ACCEL_ADDR1, STATUS);
        for (int i = 0; i < 8; i++) {
            mux_change_channel(i2c_dev, mux_channels[0]);
            read_accel_data(i2c_dev, ACCEL_ADDR1, OUT_X_MSB, accel_data);
            if ((read_data & 0x08) == 0x08) {

                short x = ((short)accel_data[0] << 8 | accel_data[1]) >> 4;
                short y = ((short)accel_data[2] << 8 | accel_data[3]) >> 4;
                short z = ((short)accel_data[4] << 8 | accel_data[5]) >> 4;

                char test[50] = {0};
                if (x > 2047) {
                    x -= 4096;
                }

                if (y > 2047) {
                    y -= 4096;
                }

                if (z > 2047) {
                    z -= 4096;
                }
                
                float test123 = ((float)x * 0.00039)*100;
                
                printk("channel:%d dev:1 x:%d, y:%d, z:%d\n", i, x, y, z);
                // printk("dev:1 x:%d, y:%d, z:%d\n", x, y, z);
            }

            read_accel_data(i2c_dev, ACCEL_ADDR2, OUT_X_MSB, accel_data);
            if ((read_data & 0x08) == 0x08) {

                short x = ((short)accel_data[0] << 8 | accel_data[1]) >> 4;
                short y = ((short)accel_data[2] << 8 | accel_data[3]) >> 4;
                short z = ((short)accel_data[4] << 8 | accel_data[5]) >> 4;

                char test[50] = {0};
                if (x > 2047) {
                    x -= 4096;
                }

                if (y > 2047) {
                    y -= 4096;
                }

                if (z > 2047) {
                    z -= 4096;
                }
                
                float test123 = ((float)x * 0.00039)*100;
                
                printk("channel:%d dev:2 x:%d, y:%d, z:%d\n", i, x, y, z);
            }
        }

        k_msleep(1000);
    }
        
}

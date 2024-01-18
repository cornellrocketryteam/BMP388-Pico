#include "bmp388.hpp"
#include <cstdio>
#include <cmath>
#include <cstring>

BMP388::BMP388(i2c_inst_t *i2c_type) {
    i2c = i2c_type;

    uint8_t dev_addr = BMP388_ADDR;

    device.intf = BMP3_I2C_INTF;
    device.read = &i2c_read;
    device.write = &i2c_write;
    device.dummy_byte = 0;
    device.delay_us = &delay_usec;
    device.intf_ptr = &dev_addr;
}

bool BMP388::begin() {
    int8_t ret = BMP3_OK;

    ret = bmp3_init(&device);
    if (ret != BMP3_OK) {
        printf("Error: Could not initialize altimeter\n");
        return false;
    }

    return true;
}

bool BMP388::read_pressure(double *pressure) {
    int8_t ret;
    uint16_t settings_sel = 0;
    struct bmp3_settings settings = { 0 };
    struct bmp3_status status = { { 0 } };
    struct bmp3_data data;

    settings.int_settings.drdy_en = BMP3_ENABLE;
    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;

    settings.odr_filter.press_os = BMP3_OVERSAMPLING_2X;
    settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
    settings.odr_filter.odr = BMP3_ODR_100_HZ;

    settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR |
                   BMP3_SEL_DRDY_EN;

    ret = bmp3_set_sensor_settings(settings_sel, &settings, &device);
    if (ret != BMP3_OK) {
        return false;
    }

    settings.op_mode = BMP3_MODE_NORMAL;

    ret = bmp3_set_op_mode(&settings, &device);
    if (ret != BMP3_OK) {
        return false;
    }

    ret = bmp3_get_status(&status, &device);
    if (ret != BMP3_OK) {
        return false;
    }

    ret = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &device);
    if (ret != BMP3_OK) {
        return false;
    }
    *pressure = data.pressure / 100;
    return true;
}

bool BMP388::read_altitude(double *altitude, double sea_level_pressure) {
    double pressure = 0.0;
    int ret = read_pressure(&pressure);
    *altitude = 44330.0 * (1.0 - pow(pressure / sea_level_pressure, 0.1903));
    return ret;
}

BMP3_INTF_RET_TYPE BMP388::i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    int ret;
    ret = i2c_write_blocking(i2c0, BMP388_ADDR, &reg_addr, 1, true);
    if (ret < 1) {
        return 1;
    }
    ret = i2c_read_blocking(i2c0, BMP388_ADDR, reg_data, len, false);
    if (ret < 1) {
        return 1;
    }
    return BMP3_INTF_RET_SUCCESS;
}

BMP3_INTF_RET_TYPE BMP388::i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    int ret;
    uint8_t buf[len + 1];
    buf[0] = reg_addr;

    memcpy(&buf[1], reg_data, len);
    ret = i2c_write_blocking(i2c0, BMP388_ADDR, buf, len + 1, false);
    if (ret < 1) {
        return 1;
    }

    return BMP3_INTF_RET_SUCCESS;
}

void BMP388::delay_usec(uint32_t us, void *intf_ptr) {
    sleep_us(us);
}
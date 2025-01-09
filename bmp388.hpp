#ifndef BMP388_HPP
#define BMP388_HPP

#include "bmp3/bmp3.h"
#include "bmp3/bmp3_defs.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define BMP388_ADDR (0x77)

#define BMP388_BYTE_TIMEOUT_US (1500)

/**
 * Representation of the BMP388 sensor.
 * Uses the Bosch Sensortec BMP3 API under the hood.
 */
class BMP388 {
public:
    /**
     * Initializes an BMP388 object on an I2C bus.
     * @param i2c_type The I2C bus that this sensor is on
     */
    BMP388(i2c_inst_t *i2c_type);

    /**
     * Attempts to establish a connection with the sensor and initiates a software reset.
     * @return True on successful connection, false otherwise
     */
    bool begin();

    /**
     * Reads a pressure value in hectopascals.
     * @param pressure The resulting pressure
     * @return True on successful read, false otherwise
     */
    bool read_pressure(float *pressure);

     /**
     * Reads an altitude value in meters.
     * @param pressure The resulting altitude
     * @param temperature The resulting temperature
     * @param sea_level_pressure Sea level pressure, in hectopascals, necessary for relative altitudes
     * @return True on successful read, false otherwise
     */
    bool read_data(float *altitude, float *temperature, float sea_level_pressure);

private:
    /**
     * The I2C bus
     */
    i2c_inst_t *i2c;

    /**
     * BMP3 device structures
     */
    struct bmp3_dev device;
    struct bmp3_status status = {{0}};
    struct bmp3_data data;

    /**
     * The current pressure reading
     */
    float pressure;

    /**
     * The current temperature reading
     */
    float temp;

    static BMP3_INTF_RET_TYPE i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
    static BMP3_INTF_RET_TYPE i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
    static void delay_usec(uint32_t us, void *intf_ptr);
};

#endif // BMP388_HPP
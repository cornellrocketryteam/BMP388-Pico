#ifndef BMP388_HPP
#define BMP388_HPP

#include "bmp3/bmp3.h"
#include "bmp3/bmp3_defs.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define BMP388_ADDR (0x77)

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
     * @param sea_level_pressure Sea level pressure, in hectopascals, necessary for relative altitudes
     * @return True on successful read, false otherwise
     */
    bool read_altitude(float *altitude, float sea_level_pressure);

private:
    /**
     * Return value for BMP3 library calls.
     */
    int8_t ret = BMP3_OK;

    /**
     * The I2C bus.
     */
    i2c_inst_t *i2c;

    /**
     * BMP3 device structure.
     */
    struct bmp3_dev device;

    static BMP3_INTF_RET_TYPE i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
    static BMP3_INTF_RET_TYPE i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
    static void delay_usec(uint32_t us, void *intf_ptr);
};

#endif // BMP388_HPP
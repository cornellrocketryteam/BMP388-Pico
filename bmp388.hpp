#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "bmp3/bmp3.h"
#include "bmp3/bmp3_defs.h"

#ifndef BMP388_
#define BMP388_

#define BMP388_ADDR (0x76)

class BMP388 {
public:
    BMP388(i2c_inst_t *i2c_type);
    bool begin();
    bool read_pressure(double *pressure);
    bool read_altitude(double *altitude, double sea_level_pressure);

private:
    i2c_inst_t *i2c;
    struct bmp3_dev device;

    static BMP3_INTF_RET_TYPE i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
    static BMP3_INTF_RET_TYPE i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
    static void delay_usec(uint32_t us, void *intf_ptr);
};

#endif
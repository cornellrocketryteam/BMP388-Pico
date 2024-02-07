#include "../bmp388.hpp"
#include "tusb.h"
#include <cstdio>

#define I2C_PORT i2c0
#define I2C_SDA 12
#define I2C_SCL 13

#define SEA_LEVEL_PRESSURE_HPA (1013.25)

BMP388 altimeter(I2C_PORT);

int main() {
    stdio_init_all();

    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    while (!tud_cdc_connected()) {
        sleep_ms(500);
    }
    printf("Connected\n");

    while (!altimeter.begin()) {
        printf("Altimeter failed to initialize\n");
        sleep_ms(1000);
    }

    float pressure, altitude;
    bool ret;

    while (true) {
        ret = altimeter.read_pressure(&pressure);
        if (!ret) {
            printf("Altimeter failed to read pressure\n");
        }

        ret = altimeter.read_altitude(&altitude, SEA_LEVEL_PRESSURE_HPA);
        if (!ret) {
            printf("Altimeter failed to read altitude\n");
        }

        printf("Pressure: %.3f\n", pressure);
        printf("Altitude: %.3f\n", altitude);
        sleep_ms(1000);
    }

    return 0;
}
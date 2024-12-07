#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "libraries/BME280_SensorAPI-master/bme280.h"

//TODO: Finish implementation

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9
#define BME280_ADDR 0x76

// == DEBUG LOOP ==
// this will just print something for some thime, it can be used to debug things
void debug_print(){
    for (int i = 0; i < 15; i++) {
        printf("IT WORKS\n");
        printf("%d\n", i);
        fflush(stdout);
        sleep_ms(1000);
    }
}
// =========================================



//reads bme280 data
void read_bme_data(struct bme280_dev *dev) {
    struct bme280_data comp_data;
    int8_t rs = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);

    if (rs == BME280_OK) {
        printf("Temperature: %.2f°C\n", comp_data.temperature / 100.0);
    } else {
        printf("BME280 initialization failed with error code: %d\n", rs);
    }
}


int main() {
    stdio_init_all();

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c

    // init for bme280 sensor
    int8_t rslt;
    struct bme280_dev dev;
    struct bme280_settings settings;

    // dev.intf_ptr = BME280_ADDR;  // Set the I2C address

    int8_t rs = bme280_init(&dev);
    if (rs != BME280_OK) {
        printf("BME 280 initialization failed!\n");
    //    return -1;
    }

    printf("BME280 sensor initialized successfully!\n");

    while (true) {
        read_bme_data(&dev);
        sleep_ms(1000);
    }
}

#include "BMP390.h"
#include "i2c.h"
#include <exception>
#include <cstdio>
#include <thread>
#include <chrono>

/* I2C interface and device provided by my custom platform-independent implementation */
static I2C_Interface i2c_interface;
static I2C_DeviceBase i2c_device(i2c_interface, 0x77); // BMP390 default I2C address is 0x77
static BMP390_I2C_Interface bmp390_i2c(i2c_device);
static BMP390_Base bmp390(bmp390_i2c);

int main(int argc, char *argv[])
{
    i2c_interface.Init(argc > 1 ? argv[1] : "/dev/i2c-0");

    int res = 0;

    try {
        bmp390.Reset(); // Reset the BMP390 to ensure it's in a known state before initialization
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait for 100 ms after reset
        res = bmp390.Init();
        if (!res) {
            bmp390.SetOversampling(BMP390_Base::Oversampling::X4, BMP390_Base::Oversampling::X4); // Set oversampling for pressure and temperature
            bmp390.SetDataRate(0x03); // Set data rate to 50 Hz (0x03 corresponds to 50 Hz in BMP390)
        } else {
            fprintf(stderr, "Failed to initialize BMP390: error code %d\n", res);
            return res;
        }
    } catch (const std::exception &e) {
        fprintf(stderr, "Failed to initialize BMP390: %s\n", e.what());
        return 1;
    }

    /* Read and print raw sensor data */
    for (int i = 0; i < 10; ++i) {
        auto real_data = bmp390.WaitForData();
        printf("Pressure: %.2f Pa, Temperature: %.2f °C, Altitude: %.2f m\n", real_data.pressure, real_data.temperature, real_data.GetAltitude());
    }

    return res;
}

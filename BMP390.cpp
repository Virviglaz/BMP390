
/*
 * This file is provided under a MIT license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * MIT License
 *
 * Copyright (c) 2026 Pavel Nadein
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * BMP390 C++ driver implementation header.
 *
 * Contact Information:
 * Pavel Nadein <pavelnadein@gmail.com>
 */

#include "BMP390.h"
#include <errno.h>
#include <cstring>
#include <stdexcept>
#include <cmath>

#define BMP390_CMD 0x7E
#define BMP390_CHIP_ID 0x00
#define BMP390_PWR_CTRL 0x1B
#define BMP390_OSR 0x1C
#define BMP390_ODR 0x1D
#define BMP388_ID 0x50
#define BMP390_ID 0x60

#ifndef BIT
#define BIT(n) (1U << (n))
#endif

void BMP390_InterfaceBase::WriteReg(uint8_t reg, uint8_t value)
{
    Write(&reg, 1, &value, 1);
}

uint8_t BMP390_InterfaceBase::ReadReg(uint8_t reg)
{
    uint8_t value;
    Read(&reg, 1, &value, 1);
    return value;
}

void BMP390_I2C_Interface::Write(const uint8_t *reg_addr,
                                 size_t reg_addr_size,
                                 const uint8_t *data,
                                 size_t length)
{
    device_.Write(reg_addr, reg_addr_size, data, length);
}

void BMP390_I2C_Interface::Read(const uint8_t *reg_addr,
                                size_t reg_addr_size,
                                uint8_t *data,
                                size_t length)
{
    device_.Read(reg_addr, reg_addr_size, data, length);
}

void BMP390_SPI_Interface::Write(const uint8_t *reg_addr,
                                 size_t reg_addr_size,
                                 const uint8_t *data,
                                 size_t length)
{
    if (length != 1)
        throw std::invalid_argument("BMP390_SPI_Interface::Write: length must be 1 for single register write");
    uint8_t buffer[2] = { static_cast<uint8_t>(reg_addr[0] & 0x7F), data[0] }; // Clear MSB for write operation
    device_.Transfer(buffer, nullptr, sizeof(buffer)); // Send register address and data
}

void BMP390_SPI_Interface::Read(const uint8_t *reg_addr,
                                size_t reg_addr_size,
                                uint8_t *data,
                                size_t length)
{
    if (length > 100)
        throw std::invalid_argument("BMP390_SPI_Interface::Read: length must not exceed 100 bytes for single register read");
    uint8_t buffer[102] = { static_cast<uint8_t>(reg_addr[0] | 0x80), 0x00 };
    device_.Transfer(buffer, data, sizeof(buffer)); // Send register address and read data
    memcpy(data, buffer + 2, length); // Copy the read buffer to the temporary buffer
}

void BMP390_Base::Reset()
{
    ifs_.WriteReg(BMP390_CMD, 0xB6); // Reset command
}

int BMP390_Base::Init()
{
    /* Check the chip ID */
    uint8_t chip_id = ifs_.ReadReg(BMP390_CHIP_ID);
    if (chip_id != BMP388_ID && chip_id != BMP390_ID) {
        return -ENODEV; // Device not found or incorrect chip ID
    }
    current_sensor_type_ = (chip_id == BMP388_ID) ? SensorType::BMP388 : SensorType::BMP390;

    /* Configure the sensor settings */
    ifs_.WriteReg(BMP390_PWR_CTRL, BIT(0) | BIT(1) | BIT(4) | BIT(5)); // Enable pressure and temperature measurements

    cal_params_ = ReadCalibrationParams();

    return 0;
}

void BMP390_Base::SetOversampling(BMP390_Base::Oversampling osr_p, BMP390_Base::Oversampling osr_t)
{
    uint8_t osr_value = (static_cast<uint8_t>(osr_t) << 3) | static_cast<uint8_t>(osr_p);
    ifs_.WriteReg(BMP390_OSR, osr_value);
}

void BMP390_Base::SetDataRate(uint8_t odr)
{
    ifs_.WriteReg(BMP390_ODR, odr & 0x0F); // Set output data rate (ODR) with lower 4 bits
}

BMP390_Base::Result BMP390_Base::ReadData()
{
    #pragma pack(push, 1)
    class RawData {
    public:
        uint8_t pressure[3]; // 24-bit raw pressure data
        uint8_t temperature[3]; // 24-bit raw temperature data
    };
    #pragma pack(pop)

    RawData raw_data;
    uint8_t reg_addr = 0x04; // Starting register address for pressure and temperature data
    ifs_.Read(&reg_addr, 1, reinterpret_cast<uint8_t*>(&raw_data), sizeof(raw_data)); // Read raw data

    uint32_t raw_pressure = (static_cast<uint32_t>(raw_data.pressure[2]) << 16) |
                            (static_cast<uint32_t>(raw_data.pressure[1]) << 8) |
                            static_cast<uint32_t>(raw_data.pressure[0]);

    uint32_t raw_temperature = (static_cast<uint32_t>(raw_data.temperature[2]) << 16) |
                               (static_cast<uint32_t>(raw_data.temperature[1]) << 8) |
                               static_cast<uint32_t>(raw_data.temperature[0]);


    Result result;
    double partial_data1 = static_cast<double>(raw_temperature) - cal_params_.t1;
    double partial_data2 = partial_data1 * cal_params_.t2;
    result.temperature = partial_data2 + (partial_data1 * partial_data1) * cal_params_.t3;

    double comp_temp = result.temperature;
    partial_data1 = cal_params_.p6 * comp_temp;
    partial_data2 = cal_params_.p7 * (comp_temp * comp_temp);
    double partial_data3 = cal_params_.p8 * (comp_temp * comp_temp * comp_temp);
    double partial_out1  = cal_params_.p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = cal_params_.p2 * comp_temp;
    partial_data2 = cal_params_.p3 * (comp_temp * comp_temp);
    partial_data3 = cal_params_.p4 * (comp_temp * comp_temp * comp_temp);
    double partial_out2 = static_cast<double>(raw_pressure) * 
                          (cal_params_.p1 + partial_data1 + partial_data2 + partial_data3);

    const double raw_pressure_d = static_cast<double>(raw_pressure);
    partial_data1 = raw_pressure_d * raw_pressure_d;
    partial_data2 = cal_params_.p9 + cal_params_.p10 * comp_temp;
    partial_data3 = partial_data1 * partial_data2;
    double partial_out3 = partial_data3 +
                          (raw_pressure_d * partial_data1) * cal_params_.p11;
    
    result.pressure = partial_out1 + partial_out2 + partial_out3;

    return result;
}

BMP390_Base::Result BMP390_Base::WaitForData()
{
    while (!IsDataReady()) { }
    return ReadData();
}

bool BMP390_Base::IsDataReady()
{
    uint8_t status = ifs_.ReadReg(0x03); // Read the status register
    return (status & BIT(5)) && (status & BIT(6));
}

double BMP390_Base::Result::GetAltitude(double sea_level_pressure) const
{
            // Calculate altitude in meters using the barometric formula
            return (1.0 - pow(pressure / sea_level_pressure, 0.190284)) * 145366.45 * 0.3048; // Convert feet to meters
}

BMP390_Base::calibration_params BMP390_Base::ReadCalibrationParams()
{
    uint8_t reg_buf[21];
    uint8_t reg_addr = 0x31; // Starting register address for calibration parameters
    ifs_.Read(&reg_addr, 1, reg_buf, sizeof(reg_buf)); // Read calibration registers

    auto u16le = [](uint8_t lsb, uint8_t msb) -> uint16_t {
        return (static_cast<uint16_t>(msb) << 8) | static_cast<uint16_t>(lsb);
    };

    auto s16le = [&](uint8_t lsb, uint8_t msb) -> int16_t {
        return static_cast<int16_t>(u16le(lsb, msb));
    };

    calibration_params params;
    uint16_t T1 = u16le(reg_buf[0], reg_buf[1]);
    uint16_t T2 = u16le(reg_buf[2], reg_buf[3]);
    int8_t T3 = static_cast<int8_t>(reg_buf[4]);

    int16_t P1 = s16le(reg_buf[5], reg_buf[6]);
    int16_t P2 = s16le(reg_buf[7], reg_buf[8]);
    int8_t P3 = static_cast<int8_t>(reg_buf[9]);
    int8_t P4 = static_cast<int8_t>(reg_buf[10]);
    uint16_t P5 = u16le(reg_buf[11], reg_buf[12]);
    uint16_t P6 = u16le(reg_buf[13], reg_buf[14]);
    int8_t P7 = static_cast<int8_t>(reg_buf[15]);
    int8_t P8 = static_cast<int8_t>(reg_buf[16]);
    int16_t P9 = s16le(reg_buf[17], reg_buf[18]);
    int8_t P10 = static_cast<int8_t>(reg_buf[19]);
    int8_t P11 = static_cast<int8_t>(reg_buf[20]);

    if (current_sensor_type_ == SensorType::BMP388) {
        // BMP388 has different calibration parameter scaling
       params.t1 = static_cast<float>(T1) / 0.00390625f;          // T1 / 2^-8  (или T1 * 256)
        params.t2 = static_cast<float>(T2) / 1073741824.0f;        // T2 / 2^30
        params.t3 = static_cast<float>(T3) / 281474976710656.0f;   // T3 / 2^48

        params.p1 = (static_cast<float>(P1) - 16384.0f) / 1048576.0f;
        params.p2 = (static_cast<float>(P2) - 16384.0f) / 536870912.0f;
        
        params.p3 = static_cast<float>(P3) / 4294967296.0f;         // P3 / 2^32
        params.p4 = static_cast<float>(P4) / 137438953472.0f;       // P4 / 2^37
        params.p5 = static_cast<float>(P5) / 0.125f;                // P5 / 2^-3  (или P5 * 8)
        params.p6 = static_cast<float>(P6) / 64.0f;                 // P6 / 2^6
        params.p7 = static_cast<float>(P7) / 256.0f;                // P7 / 2^8
        params.p8 = static_cast<float>(P8) / 32768.0f;              // P8 / 2^15
        params.p9 = static_cast<float>(P9) / 281474976710656.0f;    // P9 / 2^48
        params.p10 = static_cast<float>(P10) / 281474976710656.0f;  // P10 / 2^48
        params.p11 = static_cast<float>(P11) / 36893488147419103232.0f; // P11 / 2^65
    } else {
        params.t1 = static_cast<float>(T1) / 0.00390625f;        // T1 * 2^8
        params.t2 = static_cast<float>(T2) / 1073741824.0f;      // T2 / 2^30
        params.t3 = static_cast<float>(T3) / 281474976710656.0f; // T3 / 2^48

        params.p1 = (static_cast<float>(P1) - 16384.0f) / 1048576.0f;   // (P1 - 2^14) / 2^20
        params.p2 = (static_cast<float>(P2) - 16384.0f) / 536870912.0f; // (P2 - 2^14) / 2^29
        params.p3 = static_cast<float>(P3) / 4294967296.0f;             // P3 / 2^32
        params.p4 = static_cast<float>(P4) / 137438953472.0f;           // P4 / 2^37
        params.p5 = static_cast<float>(P5) / 0.125f;                    // P5 * 2^3
        params.p6 = static_cast<float>(P6) / 64.0f;                     // P6 / 2^6
        params.p7 = static_cast<float>(P7) / 256.0f;                    // P7 / 2^8
        params.p8 = static_cast<float>(P8) / 32768.0f;                  // P8 / 2^15
        params.p9 = static_cast<float>(P9) / 281474976710656.0f;        // P9 / 2^48
        params.p10 = static_cast<float>(P10) / 281474976710656.0f;      // P10 / 2^48
        params.p11 = static_cast<float>(P11) / 36893488147419103232.0f; // P11 / 2^65
    }

    return params;
}

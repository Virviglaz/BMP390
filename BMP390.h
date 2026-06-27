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

#ifndef BMP390_H
#define BMP390_H

#ifndef __cplusplus
#error "This header requires C++11 or higher"
#endif

#include "devices.h"

class BMP390_InterfaceBase
{
public:
    explicit BMP390_InterfaceBase() = default;
    virtual ~BMP390_InterfaceBase() = default;

    /* I2C/SPI interface */
    virtual void Write(const uint8_t *reg_addr,
                       size_t reg_addr_size,
                       const uint8_t *data,
                       size_t length) = 0;
    virtual void Read(const uint8_t *reg_addr,
                      size_t reg_addr_size,
                      uint8_t *data,
                      size_t length) = 0;

    void WriteReg(uint8_t reg, uint8_t value);
    uint8_t ReadReg(uint8_t reg);
};

class BMP390_I2C_Interface : public BMP390_InterfaceBase
{
public:
    explicit BMP390_I2C_Interface(I2C_DeviceBase& device) : device_(device) {}
    ~BMP390_I2C_Interface() override = default;

    void Write(const uint8_t *reg_addr,
               size_t reg_addr_size,
               const uint8_t *data,
               size_t length) override;

    void Read(const uint8_t *reg_addr,
              size_t reg_addr_size,
              uint8_t *data,
              size_t length) override;
private:
    I2C_DeviceBase& device_;
};

class BMP390_SPI_Interface : public BMP390_InterfaceBase
{
public:
    explicit BMP390_SPI_Interface(SPI_DeviceBase& device) : device_(device) {}
    ~BMP390_SPI_Interface() override = default;

    void Write(const uint8_t *reg_addr,
               size_t reg_addr_size,
               const uint8_t *data,
               size_t length) override;

    void Read(const uint8_t *reg_addr,
              size_t reg_addr_size,
              uint8_t *data,
              size_t length) override;
private:
    SPI_DeviceBase& device_;
};

class BMP390_Base
{
public:
    explicit BMP390_Base(BMP390_InterfaceBase& ifs) : ifs_(ifs) {}
    virtual ~BMP390_Base() = default;

    void Reset();
    int Init();

    enum class Oversampling : uint8_t {
        SKIPPED = 0,
        X1 = 1,
        X2 = 2,
        X4 = 3,
        X8 = 4,
        X16 = 5
    };

    void SetOversampling(Oversampling osr_p, Oversampling osr_t);

    void SetDataRate(uint8_t odr);

    struct Result {
        double pressure;
        double temperature;
        double GetmmHg() const { return pressure * 0.00750062; } // Convert Pa to mmHg
        double GetinHg() const { return pressure * 0.0002953; } // Convert Pa to inHg
        double GetkPa() const { return pressure * 0.001; } // Convert Pa to kPa
        double GetCelsius() const { return temperature; } // Temperature in Celsius
        double GetFahrenheit() const { return (temperature * 9.0 / 5.0) + 32.0; } // Convert Celsius to Fahrenheit
        double GetKelvin() const { return temperature + 273.15; } // Convert Celsius to Kelvin
        double GetAltitude(double sea_level_pressure = 101325.0) const;
    };
    Result ReadData();
    Result WaitForData();

    virtual bool IsDataReady();
protected:
    BMP390_InterfaceBase& ifs_;

    enum class SensorType {
        BMP388,
        BMP390
    };

    SensorType current_sensor_type_;

    struct calibration_params
    {
        float t1, t2, t3;
        float p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
    };

    calibration_params cal_params_;
    calibration_params ReadCalibrationParams();
};

#endif /* BMP390_H */

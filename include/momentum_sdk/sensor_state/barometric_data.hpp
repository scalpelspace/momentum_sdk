/**
 * \file barometric_data.hpp
 * \brief
 *    Barometric sensor data state management and synchronization
 * \details
 *    Provides thread-safe classes for managing barometric pressure and
 *    temperature data.
 * \author
 *    ScalpelSpace (info@scalpelspace.com)
 * \version 0.1
 * \date 2025-07-23
 * \copyright
 *    MIT License
 *    
 *    Copyright (c) 2025 Scalpelspace
 *    
 *    Permission is hereby granted, free of charge, to any person obtaining a copy
 *    of this software and associated documentation files (the "Software"), to deal
 *    in the Software without restriction, including without limitation the rights
 *    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *    copies of the Software, and to permit persons to whom the Software is
 *    furnished to do so, subject to the following conditions:
 *    
 *    The above copyright notice and this permission notice shall be included in all
 *    copies or substantial portions of the Software.
 *    
 *    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *    SOFTWARE.
 */

#ifndef MOMENTUM_BAROMETRIC_DATA_HPP
#define MOMENTUM_BAROMETRIC_DATA_HPP

#include <chrono>
#include <mutex>
#include "../protocol/barometric_messages.hpp"

namespace MomentumSDK {
    namespace SensorState {

        /**
         * \brief Structure containing barometric sensor readings
         * \details
         *    Holds atmospheric pressure and temperature measurements,
         *    along with timestamp and validity flag for sensor data.
         */
        struct BarometricMsg {
            double pressure_pa;  // Pressure in Pascal
            double Temp_c;      // Temperature in Celsius
            std::uint8_t BarometricState; // Sensor state (0: Normal, 1: Warning, 2: Error, 3: Critical)
            std::chrono::steady_clock::time_point TimeStamp; // Timestamp of the sensor data
            bool IsValid;
        };

        /**
         * \brief Thread-safe barometric sensor data manager
         * \details
         *    Manages barometric pressure and temperature data with thread safety.
         *    Provides methods to update and retrieve sensor measurements,
         *    check data validity, and monitor data age. Uses mutex
         *    protection for all data access operations.
         */
        class BarometricData {
            public: 

                /**
                 * \brief Updates the barometric sensor readings
                 * \details
                 *    Updates pressure and temperature readings from the barometric
                 *    sensor message. Updates timestamp and marks data as valid.
                 * \param baro Barometric message containing pressure and temperature
                 * \note Thread-safe operation using mutex protection
                 */
                void updateRead(const Protocol::Barometric& baro){
                    std::lock_guard<std::mutex> lock(data_mutex_);

                    reading_.pressure_pa = baro.getPressure();
                    reading_.Temp_c = baro.getTemperature(); 
                    reading_.BarometricState = baro.getBarometricState();

                    reading_.TimeStamp = std::chrono::steady_clock::now();
                    reading_.IsValid = true;
                }

                /**
                 * \brief Retrieves the current barometric sensor readings
                 * \return Copy of the current barometric measurements
                 * \note Thread-safe operation using mutex protection
                 */
                BarometricMsg getReading() const {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    return reading_;
                }

                /**
                 * \brief Checks if barometric data is valid and recent
                 * \return true if sensor data is valid and less than 5 seconds old
                 * \note Thread-safe operation using mutex protection
                 */
                bool Status() const {
                    std::lock_guard<std::mutex> lock(data_mutex_);

                    auto now = std::chrono::steady_clock::now();
                    auto data_age = std::chrono::duration_cast<std::chrono::seconds>(now - reading_.TimeStamp);

                    return reading_.IsValid && data_age.count() < 5;
                }

                /**
                 * \brief Gets the age of the barometric sensor data
                 * \return Age of the sensor readings in milliseconds
                 * \note Thread-safe operation using mutex protection
                 */
                std::chrono::milliseconds getDataAge() const {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    auto now = std::chrono::steady_clock::now();           
                    return std::chrono::duration_cast<std::chrono::milliseconds>(now - reading_.TimeStamp);
                }

            private:
            
                mutable std::mutex data_mutex_;
                BarometricMsg reading_{0.0, 0.0, 0, std::chrono::steady_clock::now(), false};
                static constexpr std::chrono::seconds MAX_AGE_{2}; // Maximum age for valid data
        };
    }

}

#endif // MOMENTUM_BAROMETRIC_DATA_HPP
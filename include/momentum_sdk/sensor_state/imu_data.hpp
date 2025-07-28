/**
 * \file imu_data.hpp
 * \brief
 *    IMU sensor data state management and synchronization
 * \details
 *    Provides thread-safe classes for managing IMU sensor data including
 *    quaternion orientation, gyroscope rates, accelerometer measurements,
 *    linear acceleration, and gravity vectors.
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

#ifndef MOMENTUM_IMU_DATA_HPP
#define MOMENTUM_IMU_DATA_HPP

#include <chrono>
#include <mutex>
#include <cmath>

#include "../protocol/imu_messages.hpp"

namespace MomentumSDK {
    namespace SensorState {

        /**
         * \brief Structure containing complete IMU sensor readings
         * \details
         *    Holds comprehensive IMU measurements including:
         *    - Raw accelerometer data (accel_x/y/z) in m/s^2
         *    - Gyroscope angular rates (gyro_x/y/z) in degrees/s
         *    - Orientation quaternion (quat_w/x/y/z)
         *    - Gravity vector components (gravity_x/y/z) in m/s^2
         *    - Linear acceleration (linear_accel_x/y/z) in m/s^2
         */
        struct IMUReading {
            double Accel_x, Accel_y, Accel_z;      // Acceleration in m/s^2

            double Gyro_x, Gyro_y, Gyro_z;         // Angular rates in deg/s

            double Quat_w, Quat_x, Quat_y, Quat_z; // Orientation quaternion

            double Gravity_x, Gravity_y, Gravity_z; // Gravity vector in m/s^2

            double LinearAccel_x, LinearAccel_y, LinearAccel_z; // Linear acceleration in m/s^2

            std::chrono::steady_clock::time_point Timestamp; // Timestamp of the IMU data
            bool IsValid;

        };

        /**
         * \brief Thread-safe IMU data manager
         * \details
         *    Manages comprehensive IMU sensor data with thread safety.
         *    Provides methods to update and retrieve quaternion orientation,
         *    gyroscope, accelerometer, linear acceleration, and gravity
         *    measurements. Uses mutex protection for all data access operations.
         */
        class ImuData {
            public: 

                /**
                 * \brief Updates the quaternion orientation data
                 * \details
                 *    Updates all four quaternion components (w, x, y, z) from
                 *    the IMU orientation message. Updates timestamp and marks
                 *    data as valid.
                 * \param imu1 IMU message containing quaternion orientation
                 * \note Thread-safe operation using mutex protection
                 */
                void updateQuat(const Protocol::IMU_1& imu1){
                    std::lock_guard<std::mutex> lock(data_mutex_);

                    Reading_.Quat_w = imu1.getQuaternionW();
                    Reading_.Quat_x = imu1.getQuaternionX();
                    Reading_.Quat_y = imu1.getQuaternionY();
                    Reading_.Quat_z = imu1.getQuaternionZ();

                    Reading_.Timestamp = std::chrono::steady_clock::now();
                    Reading_.IsValid = true;
                }

                /**
                 * \brief Updates the gyroscope angular rate data
                 * \details
                 *    Updates angular rates around all three axes (x, y, z) from
                 *    the IMU gyroscope message. Updates timestamp and marks
                 *    data as valid.
                 * \param imu2 IMU message containing gyroscope measurements
                 * \note Thread-safe operation using mutex protection
                 */
                void updateGyro(const Protocol::IMU_2& imu2){
                    std::lock_guard<std::mutex> lock(data_mutex_);

                    Reading_.Gyro_x = imu2.getGyroX();
                    Reading_.Gyro_y = imu2.getGyroY();
                    Reading_.Gyro_z = imu2.getGyroZ();

                    Reading_.Timestamp = std::chrono::steady_clock::now();
                    Reading_.IsValid = true;
                }

                /**
                 * \brief Updates the accelerometer data
                 * \details
                 *    Updates acceleration measurements for all three axes (x, y, z)
                 *    from the IMU accelerometer message. Updates timestamp and
                 *    marks data as valid.
                 * \param imu3 IMU message containing accelerometer measurements
                 * \note Thread-safe operation using mutex protection
                 */
                void updateAccel(const Protocol::IMU_3& imu3){
                    std::lock_guard<std::mutex> lock(data_mutex_);

                    Reading_.Accel_x = imu3.getAccelX();
                    Reading_.Accel_y = imu3.getAccelY();
                    Reading_.Accel_z = imu3.getAccelZ();

                    Reading_.Timestamp = std::chrono::steady_clock::now();
                    Reading_.IsValid = true;
                }

                /**
                 * \brief Updates the linear acceleration data
                 * \details
                 *    Updates linear acceleration (gravity compensated) for all
                 *    three axes (x, y, z) from the IMU message. Updates timestamp
                 *    and marks data as valid.
                 * \param imu4 IMU message containing linear acceleration data
                 * \note Thread-safe operation using mutex protection
                 */
                void updateLinearAccel(const Protocol::IMU_4& imu4){
                    std::lock_guard<std::mutex> lock(data_mutex_);

                    Reading_.LinearAccel_x = imu4.getLinAccelX();
                    Reading_.LinearAccel_y = imu4.getLinAccelY();
                    Reading_.LinearAccel_z = imu4.getLinAccelZ();

                    Reading_.Timestamp = std::chrono::steady_clock::now();
                    Reading_.IsValid = true;
                }

                /**
                 * \brief Updates the gravity vector data
                 * \details
                 *    Updates gravity vector components for all three axes (x, y, z)
                 *    from the IMU message. Updates timestamp and marks data as valid.
                 * \param imu5 IMU message containing gravity vector measurements
                 * \note Thread-safe operation using mutex protection
                 */
                void updateGravity(const Protocol::IMU_5& imu5){
                    std::lock_guard<std::mutex> lock(data_mutex_);

                    Reading_.Gravity_x = imu5.getGravityX();
                    Reading_.Gravity_y = imu5.getGravityY();
                    Reading_.Gravity_z = imu5.getGravityZ();

                    Reading_.Timestamp = std::chrono::steady_clock::now();
                    Reading_.IsValid = true;
                }

                /**
                 * \brief Retrieves the current IMU sensor readings
                 * \return Copy of the current IMU measurements structure
                 * \note Thread-safe operation using mutex protection
                 */
                IMUReading getRead() const {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    return Reading_;
                }

                /**
                 * \brief Checks if IMU data is valid and recent
                 * \return true if sensor data is valid and less than 5 seconds old
                 * \note Thread-safe operation using mutex protection
                 */
                bool Status() const {
                    std::lock_guard<std::mutex> lock(data_mutex_);

                    auto now = std::chrono::steady_clock::now();
                    auto DataAge = std::chrono::duration_cast<std::chrono::seconds>(now - Reading_.Timestamp);

                    return Reading_.IsValid && DataAge.count() < 5;
                }

                /**
                 * \brief Gets the age of the IMU sensor data
                 * \return Age of the sensor readings in milliseconds
                 * \note Thread-safe operation using mutex protection
                 */
                std::chrono::milliseconds getDataAge() const {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    auto now = std::chrono::steady_clock::now();
                    return std::chrono::duration_cast<std::chrono::milliseconds>(now - Reading_.Timestamp);
                }

            private:
            
                mutable std::mutex data_mutex_;
                IMUReading Reading_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, std::chrono::steady_clock::now(), false};
                static constexpr std::chrono::seconds MAX_AGE_{2};
        };
    }

}

#endif // MOMENTUM_IMU_DATA_HPP
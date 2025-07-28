/**
 * \file imu_messages.hpp
 * \brief
 *    IMU message parsers for CAN communication
 * \details
 *    Provides specialized message parser classes for handling IMU sensor Data
 *    transmitted over CAN bus.
 * \author
 *    ScalpelSpace (info@scalpelspace.com)
 * \version 0.1
 * \date 2025-07-22
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

#ifndef MOMENTUM_IMU_MSG_HPP
#define MOMENTUM_IMU_MSG_HPP
#pragma once

#include <array>
#include "message_parser.hpp"

namespace MomentumSDK {
    namespace Protocol {
        
        /**
         * \brief Parser for IMU quaternion orientation Data (CAN ID: 262)
         * \details
         *    Handles parsing of IMU orientation Data represented as a quaternion.
         *    The quaternion provides a complete 3D orientation representation
         *    avoiding gimbal lock issues. Each component has high-precision
         *    fixed-point representation.
         */
        class IMU_1 : public MessageParser {
            public:

                static constexpr std::uint32_t ID = 262;
                static constexpr std::uint8_t Length = 8;

                explicit IMU_1(const std::array<std::uint8_t, 8>& Data)
                : MessageParser(ID, Length, Data) {}

                /**
                 * \brief Get the X component of the orientation quaternion
                 * \return X component in range [-1.0, 1.0]
                 * \note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                double getQuaternionX() const {
                    const can_signal_t* signal = findSignal(ID, "quaternion_x");
                    return signal ? static_cast<double>(decode_signal(signal, getData().data())) : 0.0;
                }

                /**
                 * \brief Get the Y component of the orientation quaternion
                 * \return Y component in range [-1.0, 1.0]
                 * \note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                double getQuaternionY() const {
                    const can_signal_t* signal = findSignal(ID, "quaternion_y");
                    return signal ? static_cast<double>(decode_signal(signal, getData().data())) : 0.0;
                }

                /**
                 * \brief Get the Z component of the orientation quaternion
                 * \return Z component in range [-1.0, 1.0]
                 * \note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                double getQuaternionZ() const {
                    const can_signal_t* signal = findSignal(ID, "quaternion_z");
                    return signal ? static_cast<double>(decode_signal(signal, getData().data())) : 0.0;
                }

                /**
                 * \brief Get the W (scalar) component of the orientation quaternion
                 * \return W component in range [-1.0, 1.0]
                 * \note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                double getQuaternionW() const {
                    const can_signal_t* signal = findSignal(ID, "quaternion_w");
                    return signal ? static_cast<double>(decode_signal(signal, getData().data())) : 0.0;
                }

        }; 

        /**
         * \brief Parser for IMU gyroscope Data (CAN ID: 263)
         * \details
         *    Handles parsing of IMU angular velocity measurements from the
         *    gyroscope. Data is provided for all three axes with high
         *    precision and wide dynamic range.
         */
        class IMU_2 : public MessageParser {
            public:

                static constexpr std::uint32_t ID = 263;
                static constexpr std::uint8_t  Length = 8;

                explicit IMU_2(const std::array<std::uint8_t, 8>& Data)
                : MessageParser(ID, Length, Data) {}

                /**
                 * \brief Get the angular velocity around X axis
                 * \return Angular velocity in degrees per second, range [-2000.0, 2000.0]
                 * \note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                double getGyroX() const {
                    const can_signal_t* signal = findSignal(ID, "gyro_x");
                    return signal ? static_cast<double>(decode_signal(signal, getData().data())) : 0.0;
                }

                /**
                 * \brief Get the angular velocity around Y axis
                 * \return Angular velocity in degrees per second, range [-2000.0, 2000.0]
                 * \note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                double getGyroY() const {
                    const can_signal_t* signal = findSignal(ID, "gyro_y");
                    return signal ? static_cast<double>(decode_signal(signal, getData().data())) : 0.0;
                }

                /**
                 * \brief Get the angular velocity around Z axis
                 * \return Angular velocity in degrees per second, range [-2000.0, 2000.0]
                 * \note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                double getGyroZ() const {
                    const can_signal_t* signal = findSignal(ID, "gyro_z");
                    return signal ? static_cast<double>(decode_signal(signal, getData().data())) : 0.0;
                }
        };

        /**
         * \brief Parser for IMU accelerometer Data (CAN ID: 264)
         * \details
         *    Handles parsing of IMU acceleration measurements from the
         *    accelerometer. Provides total acceleration including gravity
         *    for all three axes with high precision.
         */
        class IMU_3 : public MessageParser {
            public:
                
                static constexpr std::uint32_t ID = 264;
                static constexpr std::uint8_t  Length = 8;

                explicit IMU_3(const std::array<std::uint8_t, 8>& Data)
                : MessageParser(ID, Length, Data) {}

                /**
                 * \brief Get the acceleration along X axis
                 * \return Acceleration in m/s², range [-156.9, 156.9]
                 * \note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                double getAccelX() const {
                    const can_signal_t* signal = findSignal(ID, "accel_x");
                    return signal ? static_cast<double>(decode_signal(signal, getData().data())) : 0.0;
                }

                /**
                 * \brief Get the acceleration along Y axis
                 * \return Acceleration in m/s², range [-156.9, 156.9]
                 * \note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                double getAccelY() const {
                    const can_signal_t* signal = findSignal(ID, "accel_y");
                    return signal ? static_cast<double>(decode_signal(signal, getData().data())) : 0.0;
                }

                /**
                 * \brief Get the acceleration along Z axis
                 * \return Acceleration in m/s², range [-156.9, 156.9]
                 * \note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                double getAccelZ() const {
                    const can_signal_t* signal = findSignal(ID, "accel_z");
                    return signal ? static_cast<double>(decode_signal(signal, getData().data())) : 0.0;
                }
        };

        /**
         * \brief Parser for IMU linear acceleration Data (CAN ID: 265)
         * \details
         *    Handles parsing of IMU linear acceleration measurements with
         *    gravity component removed. Provides pure motion acceleration
         *    for all three axes.
         */
        class IMU_4 : public MessageParser {
            public:

                static constexpr std::uint32_t ID = 265;
                static constexpr std::uint8_t  Length = 8;

                explicit IMU_4(const std::array<std::uint8_t, 8>& Data)
                : MessageParser(ID, Length, Data) {}

                /**
                 * \brief Get the linear acceleration along X axis (gravity removed)
                 * \return Linear acceleration in m/s², range [-156.9, 156.9]
                 * \note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                double getLinAccelX() const {
                    const can_signal_t* signal = findSignal(ID, "lin_accel_x");
                    return signal ? static_cast<double>(decode_signal(signal, getData().data())) : 0.0;
                }

                /**
                 * \brief Get the linear acceleration along Y axis (gravity removed)
                 * \return Linear acceleration in m/s², range [-156.9, 156.9]
                 * \note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                double getLinAccelY() const {
                    const can_signal_t* signal = findSignal(ID, "lin_accel_y");
                    return signal ? static_cast<double>(decode_signal(signal, getData().data())) : 0.0;
                }

                /**
                 * \brief Get the linear acceleration along Z axis (gravity removed)
                 * \return Linear acceleration in m/s², range [-156.9, 156.9]
                 * \note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                double getLinAccelZ() const {
                    const can_signal_t* signal = findSignal(ID, "lin_accel_z");
                    return signal ? static_cast<double>(decode_signal(signal, getData().data())) : 0.0;
                }
        };

        /**
         * \brief Parser for IMU gravity vector Data (CAN ID: 272)
         * \details
         *    Handles parsing of the estimated gravity vector components.
         *    This represents the direction of gravity in the sensor's
         *    frame of reference with high precision measurements.
         */
        class IMU_5 : public MessageParser {
            public:

                static constexpr std::uint32_t ID = 272;
                static constexpr std::uint8_t  Length = 8;

                explicit IMU_5(const std::array<std::uint8_t, 8>& Data)
                : MessageParser(ID, Length, Data) {}

                /**
                 * \brief Get the gravity vector component along X axis
                 * \return Gravity in m/s², centered around -9.81
                 * \note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                double getGravityX() const {
                    const can_signal_t* signal = findSignal(ID, "gravity_x");
                    return signal ? static_cast<double>(decode_signal(signal, getData().data())) : 0.0;
                }

                /**
                 * \brief Get the gravity vector component along Y axis
                 * \return Gravity in m/s², centered around -9.81
                 * \note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                double getGravityY() const {
                    const can_signal_t* signal = findSignal(ID, "gravity_y");
                    return signal ? static_cast<double>(decode_signal(signal, getData().data())) : 0.0;
                }

                /**
                 * \brief Get the gravity vector component along Z axis
                 * \return Gravity in m/s², centered around -9.81
                 * \note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                double getGravityZ() const {
                    const can_signal_t* signal = findSignal(ID, "gravity_z");
                    return signal ? static_cast<double>(decode_signal(signal, getData().data())) : 0.0;
                }
        };
    }

}

#endif // MOMENTUM_IMU_MSG_HPP
/**
 * \file message_parser.cpp
 * \brief
 *    CAN message parsing utilities and ID mapping
 * \details
 *    Provides utility functions for mapping CAN IDs to message types and
 *    parsing CAN frames into appropriate protocol message objects for 
 *    Momentum board.
 * \author
 *    ScalpelSpace (info@scalpelspace.com)
 * \version 0.1
 * \date 2025-07-25
 * \copyright
 *    MIT License
 * 
 * Copyright (c) 2025 Scalpelspace
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

#include "../../include/momentum_sdk/protocol/message_parser.hpp"
#include "../../include/momentum_sdk/protocol/gps_messages.hpp"
#include "../../include/momentum_sdk/protocol/imu_messages.hpp"
#include "../../include/momentum_sdk/protocol/barometric_messages.hpp"
#include "../../include/momentum_sdk/protocol/system_messages.hpp"
#include "../../include/momentum_sdk/can/can_frame.hpp"

#include <unordered_map>
#include <string>
#include <stdexcept>

namespace MomentumSDK {
    namespace Protocol {

        /**
         * \brief Enumeration of supported message types in the Momentum protocol
         */
        enum class MessageType {
            SYSTEM_STATE = 257,    // System state message
            BAROMETRIC = 258,      // Barometric sensor data
            GPS_1 = 259,          // GPS latitude/longitude
            GPS_2 = 260,          // GPS speed/course/satellites/HDOP
            GPS_3 = 261,          // GPS altitude/geoid separation/state
            IMU_1 = 262,          // IMU quaternion orientation
            IMU_2 = 263,          // IMU gyroscope data
            IMU_3 = 264,          // IMU accelerometer data
            IMU_4 = 265,          // IMU linear acceleration
            IMU_5 = 272,          // IMU gravity vector
            COMMAND = 513,        // Command/control messages
            RTC = 600,            // Real-time clock data
            UNKNOWN = 0           // Unknown message type
        };

        /**
         * \brief Maps a CAN ID to its corresponding message type
         * \param can_id The CAN identifier to map
         * \return The corresponding MessageType enumeration value
         */
        MessageType getMessageType(std::uint32_t can_id) noexcept {
            static const std::unordered_map<std::uint32_t, MessageType> id_map = {
                {257, MessageType::SYSTEM_STATE},
                {258, MessageType::BAROMETRIC},
                {259, MessageType::GPS_1},
                {260, MessageType::GPS_2},
                {261, MessageType::GPS_3},
                {262, MessageType::IMU_1},
                {263, MessageType::IMU_2},
                {264, MessageType::IMU_3},
                {265, MessageType::IMU_4},
                {272, MessageType::IMU_5},
                {513, MessageType::COMMAND},
                {600, MessageType::RTC}
            };
            
            auto it = id_map.find(can_id);
            return (it != id_map.end()) ? it->second : MessageType::UNKNOWN;
        }

        /**
         * \brief Gets a human-readable string description of a message type
         * \param type The MessageType to describe
         * \return String description of the message type
         */
        std::string getMessageTypeName(MessageType type) noexcept {
            switch (type) {
                case MessageType::SYSTEM_STATE: return "System State";
                case MessageType::BAROMETRIC:   return "Barometric Sensor";
                case MessageType::GPS_1:        return "GPS Position";
                case MessageType::GPS_2:        return "GPS Navigation";
                case MessageType::GPS_3:        return "GPS Altitude";
                case MessageType::IMU_1:        return "IMU Quaternion";
                case MessageType::IMU_2:        return "IMU Gyroscope";
                case MessageType::IMU_3:        return "IMU Accelerometer";
                case MessageType::IMU_4:        return "IMU Linear Acceleration";
                case MessageType::IMU_5:        return "IMU Gravity Vector";
                case MessageType::COMMAND:      return "Command Message";
                case MessageType::RTC:          return "Real-Time Clock";
                case MessageType::UNKNOWN:
                default:                        return "Unknown Message";
            }
        }

        /**
         * \brief Gets a human-readable string description of a CAN ID
         * \param can_id The CAN identifier to describe
         * \return String description of the CAN ID
         */
        std::string getMessageTypeName(std::uint32_t can_id) noexcept {
            return getMessageTypeName(getMessageType(can_id));
        }

        /**
         * \brief Checks if a CAN ID corresponds to a GPS message
         * \param can_id The CAN identifier to check
         * \return true if the ID is for a GPS message, false otherwise
         */
        bool isGpsMessage(std::uint32_t can_id) noexcept {
            MessageType type = getMessageType(can_id);
            return (type == MessageType::GPS_1 || 
                    type == MessageType::GPS_2 || 
                    type == MessageType::GPS_3);
        }

        /**
         * \brief Checks if a CAN ID corresponds to an IMU message
         * \param can_id The CAN identifier to check
         * \return true if the ID is for an IMU message, false otherwise
         */
        bool isImuMessage(std::uint32_t can_id) noexcept {
            MessageType type = getMessageType(can_id);
            return (type == MessageType::IMU_1 || 
                    type == MessageType::IMU_2 || 
                    type == MessageType::IMU_3 || 
                    type == MessageType::IMU_4 || 
                    type == MessageType::IMU_5);
        }

        /**
         * \brief Validates that a CAN frame has the expected data length for its message type
         * \param frame The CAN frame to validate
         * \return true if the frame has valid data length, false otherwise
         */
        bool isValidFrameLength(const CAN::CANFrame& frame) noexcept {
            
            auto frame_data = frame.getFrame();
            return frame_data.size() == 8;
        }

        /**
         * \brief Gets the expected update rate for a given message type
         * \param type The MessageType to query
         * \return Expected update rate in Hz, or 0 if unknown
         */
        double getExpectedUpdateRate(MessageType type) noexcept {
            switch (type) {
                case MessageType::GPS_1:
                case MessageType::GPS_2:
                case MessageType::GPS_3:        return 10.0;  // 10 Hz GPS updates
                case MessageType::IMU_1:
                case MessageType::IMU_2:
                case MessageType::IMU_3:
                case MessageType::IMU_4:
                case MessageType::IMU_5:        return 100.0; // 100 Hz IMU updates
                case MessageType::BAROMETRIC:   return 20.0;  // 20 Hz barometric updates
                case MessageType::SYSTEM_STATE: return 1.0;   // 1 Hz system state updates
                case MessageType::RTC:          return 1.0;   // 1 Hz RTC updates
                case MessageType::COMMAND:      return 0.0;   // Event-driven
                case MessageType::UNKNOWN:
                default:                        return 0.0;   // Unknown
            }
        }

        /**
         * \brief Gets the expected update rate for a given CAN ID
         * \param can_id The CAN identifier to query
         * \return Expected update rate in Hz, or 0 if unknown
         */
        double getExpectedUpdateRate(std::uint32_t can_id) noexcept {
            return getExpectedUpdateRate(getMessageType(can_id));
        }

    }
} 
/**
 * \file system_messages.hpp
 * \brief
 *    System-related message parsers for CAN communication
 * \details
 *    Provides specialized message parser classes for handling system-related data
 *    transmitted over CAN bus.
 * \author
 *    ScalpelSpace (info@scalpelspace.com)
 * \version 0.1
 * \date 2025-07-24
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

#ifndef MOMENTUM_SYSTEM_MSG_HPP
#define MOMENTUM_SYSTEM_MSG_HPP
#pragma once

#include <array>
#include "message_parser.hpp"

namespace MomentumSDK {
    namespace Protocol {

        /**
         * @brief Parser for system state messages
         * 
         * The State class handles CAN messages containing system state information.
         * It uses CAN ID 257 and operates on 8-byte messages.
         */
        class State : public MessageParser {
            public:

                static constexpr std::uint32_t ID = 257;      //CAN message identifier
                static constexpr std::uint8_t Length = 8;     //Message length in bytes

                explicit State(const std::array<std::uint8_t, 8>& Data)
                : MessageParser(ID, Length, Data) {}

                /**
                 * \brief Get the system state
                 * \return System state as a string
                 * \note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                std::uint8_t getState() const {
                    const can_signal_t* signal = findSignal(ID, "system_state");
                    return signal ? static_cast<std::uint8_t>(decode_signal(signal, getData().data())) : 0;
                }
        };

        /**
         * @brief Parser for Real-Time Clock (RTC) messages
         * 
         * The RTC class handles CAN messages containing real-time clock information.
         * It uses CAN ID 600 and operates on 8-byte messages that contain:
         * - RTC state
         * - Date (year, month, day, weekday)
         * - Time (hour, minute, second)
         */
        class RTC : public MessageParser {
            public:

                static constexpr std::uint32_t ID = 600;     
                static constexpr std::uint8_t Length = 8;    

                /**
                 * @brief Construct a new RTC message parser
                 * @param Data The 8-byte array containing the CAN message data
                 */
                explicit RTC(const std::array<std::uint8_t, 8>& Data)
                : MessageParser(ID, Length, Data) {}

                
                /**
                 * @brief Get the RTC state
                 * @return Current state of the RTC system
                 * @note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                std::uint8_t getRtcState() const {
                    const can_signal_t* signal = findSignal(ID, "rtc_state");
                    return signal ? static_cast<std::uint8_t>(decode_signal(signal, getData().data())) : 0;
                }

                /**
                 * @brief Get the year from RTC
                 * @return Current year (offset from 2000)
                 * @note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                std::uint16_t getRtcYear() const {
                    const can_signal_t* signal = findSignal(ID, "rtc_year");
                    return signal ? static_cast<std::uint16_t>(decode_signal(signal, getData().data())) : 2000;
                }

                /**
                 * @brief Get the month from RTC
                 * @return Current month (1-12)
                 * @note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                std::uint8_t getRtcMonth() const {
                    const can_signal_t* signal = findSignal(ID, "rtc_month");
                    return signal ? static_cast<std::uint8_t>(decode_signal(signal, getData().data())) : 1;
                }

                /**
                 * @brief Get the day of month from RTC
                 * @return Current day (1-31)
                 * @note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                std::uint8_t getRtcDay() const {
                    const can_signal_t* signal = findSignal(ID, "rtc_day");
                    return signal ? static_cast<std::uint8_t>(decode_signal(signal, getData().data())) : 1;
                }

                /**
                 * @brief Get the day of week from RTC
                 * @return Current weekday (0-6, where 0 is Sunday)
                 * @note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                std::uint8_t getRtcWeekDay() const {
                    const can_signal_t* signal = findSignal(ID, "rtc_weekday");
                    return signal ? static_cast<std::uint8_t>(decode_signal(signal, getData().data())) : 1;
                }

                /**
                 * @brief Get the hour from RTC
                 * @return Current hour (0-23)
                 * @note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                std::uint8_t getRtcHour() const {
                    const can_signal_t* signal = findSignal(ID, "rtc_hour");
                    return signal ? static_cast<std::uint8_t>(decode_signal(signal, getData().data())) : 0;
                }

                /**
                 * @brief Get the minute from RTC
                 * @return Current minute (0-59)
                 * @note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                std::uint8_t getRtcMinute() const {
                    const can_signal_t* signal = findSignal(ID, "rtc_minute");
                    return signal ? static_cast<std::uint8_t>(decode_signal(signal, getData().data())) : 0;
                }

                /**
                 * @brief Get the second from RTC
                 * @return Current second (0-59)
                 * @note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                std::uint8_t getRtcSecond() const {
                    const can_signal_t* signal = findSignal(ID, "rtc_second");
                    return signal ? static_cast<std::uint8_t>(decode_signal(signal, getData().data())) : 0;
                }

        };
    }
}

#endif // MOMENTUM_SYSTEM_MSG_HPP
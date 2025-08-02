/**
 * \file barometric_messages.hpp
 * \brief
 *    Barometric pressure sensor message parser for CAN communication
 * \details
 *    Provides specialized message parser class for handling barometric sensor data
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

#ifndef MOMENTUM_BAROMETRIC_MSG_HPP
#define MOMENTUM_BAROMETRIC_MSG_HPP
#pragma once

#include <array>
#include "message_parser.hpp"

namespace MomentumSDK {
    namespace Protocol {

        /**
         * \brief Parser for barometric sensor data (CAN ID: 258)
         * \details
         *    Handles parsing of barometric sensor measurements including
         *    atmospheric pressure, temperature, and sensor state information.
         *    All measurements use fixed-point representation for high precision.
         */
        class Barometric : public MessageParser {
            public:

                static constexpr std::uint32_t ID = 258;
                static constexpr std::uint8_t Length = 8;

                explicit Barometric(const std::array<std::uint8_t, 8>& Data)
                : MessageParser(ID, Length, Data) {}

                /**
                 * \brief Get the atmospheric pressure measurement
                 * \return Pressure in hectopascals (hPa), range [30000.0, 24339514.8897]
                 * \note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                double getPressure() const {
                    const can_signal_t* signal = findSignal(ID, "pressure");
                    return signal ? static_cast<double>(decode_signal(signal, getData().data())) : 0.0;
                }

                /**
                 * \brief Get the temperature measurement
                 * \return Temperature in degrees Celsius, range [-40.0, 85.0]
                 * \note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                double getTemperature() const {
                    const can_signal_t* signal = findSignal(ID, "temperature");
                    return signal ? static_cast<double>(decode_signal(signal, getData().data())) : 0.0;
                }

                /**
                 * \brief Get the barometric sensor state
                 * \return Sensor state (0: Normal, 1: Warning, 2: Error, 3: Critical)
                 * \note Uses DBC-defined signal parameters
                 */
                [[nodiscard]]
                std::uint8_t getBarometricState() const {
                    const can_signal_t* signal = findSignal(ID, "barometric_state");
                    return signal ? static_cast<std::uint8_t>(decode_signal(signal, getData().data())) : 0;
                }

        };
    }
}

#endif // MOMENTUM_BAROMETRIC_MSG_HPP
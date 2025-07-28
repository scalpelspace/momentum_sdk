/**
 * \file gps_messages.hpp
 * \brief
 *    GPS message parsers for CAN communication
 * \details
 *    Provides specialized message parser classes for handling GPS data transmitted
 *    over CAN bus. Includes parsers for position data (latitude/longitude) and
 *    navigation data (speed, course, satellites, etc.)
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

#ifndef MOMENTUM_GPS_MSG_HPP
#define MOMENTUM_GPS_MSG_HPP
#pragma once

#include <array>
#include "message_parser.hpp"

namespace MomentumSDK {
    namespace Protocol {
        
        /**
         * \brief Parser for primary GPS position data (CAN ID: 259)
         * \details
         *    Handles parsing of GPS position data containing latitude and longitude
         *    coordinates.
         */
        class  GPS_1 : public MessageParser { 
            public:

                static constexpr std::uint32_t ID = 259;
                static constexpr std::uint8_t Length = 8;

                /**
                 * \brief Constructs a GPS_1 position message parser
                 * \param Data Raw CAN message data to be parsed
                 */
                explicit GPS_1(const std::array<std::uint8_t, 8>& Data)
                : MessageParser(ID, Length, Data) {}

                /**
                 * \brief Get the latitude coordinate
                 * \return Latitude in degrees, range [-90.0, 90.0]
                 * \note Uses DBC-defined signal parameters for accurate scaling
                 */
                [[nodiscard]]
                double getLatitude() const {
                    const can_signal_t* signal = findSignal(ID, "latitude");
                    return signal ? static_cast<double>(decode_signal(signal, getData().data())) : 0.0;
                }

                /**
                 * \brief Get the longitude coordinate
                 * \return Longitude in degrees, range [-180.0, 180.0]
                 * \note Uses DBC-defined signal parameters for accurate scaling
                 */
                [[nodiscard]]
                double getLongitude() const {
                    const can_signal_t* signal = findSignal(ID, "longitude");
                    return signal ? static_cast<double>(decode_signal(signal, getData().data())) : 0.0;
                }
        };

        /**
         * \brief Parser for secondary GPS navigation data (CAN ID: 260)
         * \details
         *    Handles parsing of GPS navigation data including ground speed, course,
         *    position fix type, number of satellites in view, and horizontal
         *    dilution of precision (HDOP). 
         */
        class GPS_2 : public MessageParser {
            public: 

                static constexpr std::uint32_t ID = 260;
                static constexpr std::uint8_t Length = 8;

                /**
                 * \brief Constructs a GPS_2 navigation data parser
                 * \param Data Raw CAN message data to be parsed
                 */
                explicit GPS_2(const std::array<std::uint8_t, 8>& Data)
                : MessageParser(ID, Length, Data) {}

                /**
                 * \brief Get the ground speed
                 * \return Speed in meters per second with 0.01 resolution
                 */
                [[nodiscard]]
                double getSpeed() const {
                    const can_signal_t* signal = findSignal(ID, "speed");
                    return signal ? static_cast<double>(decode_signal(signal, getData().data())) : 0.0;
                }

                /**
                 * \brief Get the course over ground
                 * \return Course in degrees [0.0, 360.0) with 0.01 resolution
                 */
                [[nodiscard]]
                double getCourse() const {
                    const can_signal_t* signal = findSignal(ID, "course");
                    return signal ? static_cast<double>(decode_signal(signal, getData().data())) : 0.0;
                }

                /**
                 * \brief Get the GPS position fix type
                 * \return Fix type (0: No fix, 1: 2D fix, 2: 3D fix)
                 */
                [[nodiscard]]
                std::uint8_t getPositionFix() const {
                    const can_signal_t* signal = findSignal(ID, "position_fix");
                    return signal ? static_cast<std::uint8_t>(decode_signal(signal, getData().data())) : 0;
                }

                /**
                 * \brief Get the number of satellites used in the position solution
                 * \return Number of satellites in view
                 */
                [[nodiscard]]
                std::uint8_t getSatelliteCount() const {
                    const can_signal_t* signal = findSignal(ID, "satellite_count");
                    return signal ? static_cast<std::uint8_t>(decode_signal(signal, getData().data())) : 0;
                }

                /**
                 * \brief Get the Horizontal Dilution of Precision
                 * \return HDOP value with 0.01 resolution
                 * \note Lower values indicate better geometric precision
                 */
                [[nodiscard]]
                double getHDOP() const {
                    const can_signal_t* signal = findSignal(ID, "hdop");
                    return signal ? static_cast<double>(decode_signal(signal, getData().data())) : 0.0;
                }

        };
        
        /**
         * \brief Parser for tertiary GPS altitude and state data (CAN ID: 261)
         * \details
         *    Handles parsing of GPS altitude data, geoid separation, and
         *    GPS receiver state information. All altitude measurements use
         *    fixed-point representation for high precision.
         */
        class GPS_3 : public MessageParser {
            public: 

                static constexpr std::uint32_t ID = 261;
                static constexpr std::uint8_t Length = 8;

                /**
                 * \brief Constructs a GPS_3 altitude and state data parser
                 * \param Data Raw CAN message data to be parsed
                 */
                explicit GPS_3(const std::array<std::uint8_t, 8>& Data)
                : MessageParser(ID, Length, Data) {}

                /**
                 * \brief Get the altitude above mean sea level
                 * \return Altitude in meters with 0.01 resolution
                 * \note Range starts at -150m to accommodate measurements below sea level
                 */
                [[nodiscard]]
                double getAltitude() const {
                    const can_signal_t* signal = findSignal(ID, "altitude");
                    return signal ? static_cast<double>(decode_signal(signal, getData().data())) : 0.0;
                }

                /**
                 * \brief Get the geoid separation (difference between WGS-84 ellipsoid and mean sea level)
                 * \return Separation in meters with 0.01 resolution
                 * \note Positive values indicate the geoid is above the WGS-84 ellipsoid
                 */
                [[nodiscard]]
                double getGeoidSeparation() const {
                    const can_signal_t* signal = findSignal(ID, "geoid_separation");
                    return signal ? static_cast<double>(decode_signal(signal, getData().data())) : 0.0;
                }

                /**
                 * \brief Get the current GPS receiver state
                 * \return State value (0: Not initialized, 1: Normal operation, 
                 *         2: Warning state, 3: Critical error)
                 */
                [[nodiscard]]
                std::uint8_t getGpsState() const {
                    const can_signal_t* signal = findSignal(ID, "gps_state");
                    return signal ? static_cast<std::uint8_t>(decode_signal(signal, getData().data())) : 0;
                }
     
        };
    }
}

#endif // MOMENTUM_GPS_MSG_HPP
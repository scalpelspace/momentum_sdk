/**
 * \file gps_data.hpp
 * \brief
 *    GPS data state management and synchronization
 * \details
 *    Provides thread-safe classes for managing GPS position and velocity data.
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

#ifndef MOMENTUM_GPS_DATA_HPP
#define MOMENTUM_GPS_DATA_HPP

#include <chrono>
#include <mutex>
#include "../protocol/gps_messages.hpp"

namespace MomentumSDK {
    namespace SensorState {

        /**
         * \brief Structure containing GPS position information
         * \details
         *    Holds complete 3D position data including latitude, longitude,
         *    and altitude, along with timestamp and validity flag.
         */
        struct GpsPosition {
            double Latitude_deg;  // Latitude in degrees
            double Longitude_deg; // Longitude in degrees
            double Altitude_m;  // Altitude in meters
            double GeoidSeparation; // Geoid separation in meters
            std::chrono::steady_clock::time_point TimeStamp; // Timestamp of the GPS data
            bool IsValid;

        };

        /**
         * \brief Structure containing GPS velocity information
         * \details
         *    Holds ground speed and course information along with
         *    timestamp and validity flag for velocity measurements.
         */
        struct GpsVelocity {
            double Speed_ms;  // Speed in meters per second
            double Course_deg; // Course in degrees
            std::chrono::steady_clock::time_point TimeStamp; // Timestamp of the GPS data
            bool IsValid;
        };

        struct GpsUtilities {
            std::uint8_t PositionFix;
            std::uint8_t SatelliteCount;
            double Hdop;
            std::uint8_t GpsState;
            std::chrono::steady_clock::time_point TimeStamp; // Timestamp of the GPS utilities data
            bool IsValid;
        };

        /**
         * \brief Thread-safe GPS data manager
         * \details
         *    Manages GPS position and velocity data with thread safety.
         *    Provides methods to update and retrieve GPS measurements,
         *    check data validity, and monitor data age. Uses mutex
         *    protection for all data access operations.
         */
        class GpsData {
            public: 

                /**
                 * \brief Updates the GPS position data
                 * \details
                 *    Updates latitude and longitude from GPS_1 message and
                 *    altitude from GPS_3 message. Updates timestamp and
                 *    marks data as valid.
                 * \param gps1 GPS message containing latitude/longitude
                 * \param gps3 GPS message containing altitude
                 * \note Thread-safe operation using mutex protection
                 */
                void updatePosition(const Protocol::GPS_1& gps1, const Protocol::GPS_3& gps3){
                    std::lock_guard<std::mutex> lock(data_mutex_);

                    Position_.Latitude_deg = gps1.getLatitude();
                    Position_.Longitude_deg = gps1.getLongitude();
                    
                    Position_.Altitude_m = gps3.getAltitude();

                    Position_.TimeStamp = std::chrono::steady_clock::now();
                    Position_.IsValid = true;
                }

                /**
                 * \brief Updates the GPS velocity data
                 * \details
                 *    Updates speed and course information from GPS_2 message.
                 *    Updates timestamp and marks data as valid.
                 * \param gps2 GPS message containing speed and course data
                 * \note Thread-safe operation using mutex protection
                 */
                void updateVelocity(const Protocol::GPS_2& gps2){
                    std::lock_guard<std::mutex> lock(data_mutex_);

                    Velocity_.Speed_ms = gps2.getSpeed();
                    Velocity_.Course_deg = gps2.getCourse();

                    Velocity_.TimeStamp = std::chrono::steady_clock::now();
                    Velocity_.IsValid = true;
                }

                /**
                 * \brief Updates the GPS utilities data
                 * \details
                 *    Updates position fix, satellite count, HDOP from GPS_2 message
                 *    and GPS state from GPS_3 message. Updates timestamp and
                 *    marks data as valid.
                 * \param gps2 GPS message containing fix, satellites, and HDOP data
                 * \param gps3 GPS message containing GPS state
                 * \note Thread-safe operation using mutex protection
                 */
                void updateUtilities(const Protocol::GPS_2& gps2, const Protocol::GPS_3& gps3){
                    std::lock_guard<std::mutex> lock(data_mutex_);

                    Utilities_.PositionFix = gps2.getPositionFix();
                    Utilities_.SatelliteCount = gps2.getSatelliteCount();
                    Utilities_.Hdop = gps2.getHDOP();
                    Utilities_.GpsState = gps3.getGpsState();

                    Utilities_.TimeStamp = std::chrono::steady_clock::now();
                    Utilities_.IsValid = true;
                }

                /**
                 * \brief Retrieves the current GPS position data
                 * \return Copy of the current GPS position structure
                 * \note Thread-safe operation using mutex protection
                 */
                GpsPosition getPosition() const {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    return Position_;
                }

                /**
                 * \brief Retrieves the current GPS velocity data
                 * \return Copy of the current GPS velocity structure
                 * \note Thread-safe operation using mutex protection
                 */
                GpsVelocity getVelocity() const {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    return Velocity_;
                }

                /**
                 * \brief Retrieves the current GPS utilities data
                 * \return Copy of the current GPS utilities structure containing
                 *         position fix, satellite count, HDOP, and GPS state
                 * \note Thread-safe operation using mutex protection
                 */
                GpsUtilities getUtilities() const {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    return Utilities_;
                }

                /**
                 * \brief Checks if GPS data is valid and recent
                 * \return true if both position and velocity data are valid
                 *         and less than 5 seconds old
                 * \note Thread-safe operation using mutex protection
                 */
                bool Status() const {
                    std::lock_guard<std::mutex> lock(data_mutex_);

                    auto now = std::chrono::steady_clock::now();
                    auto position_age = std::chrono::duration_cast<std::chrono::seconds>(now - Position_.TimeStamp);
                    auto velocity_age = std::chrono::duration_cast<std::chrono::seconds>(now - Velocity_.TimeStamp);
                    auto utilities_age = std::chrono::duration_cast<std::chrono::seconds>(now - Utilities_.TimeStamp);

                    return Position_.IsValid && Velocity_.IsValid && Utilities_.IsValid && position_age.count() < 5 && velocity_age.count() < 5
                           && utilities_age.count() < 5;
                }

                /**
                 * \brief Gets the age of the oldest GPS data
                 * \return Age of the oldest data (position or velocity) in milliseconds
                 * \note Thread-safe operation using mutex protection
                 */
                std::chrono::milliseconds getDataAge() const {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    auto now = std::chrono::steady_clock::now();
                    auto PositionAge = std::chrono::duration_cast<std::chrono::milliseconds>(now - Position_.TimeStamp);
                    auto VelocityAge = std::chrono::duration_cast<std::chrono::milliseconds>(now - Velocity_.TimeStamp);

                    return std::max(PositionAge, VelocityAge);
                }

            private:
            
                mutable std::mutex data_mutex_;
                GpsPosition Position_{0.0, 0.0, 0.0, 0.0, std::chrono::steady_clock::now(), false};
                GpsVelocity Velocity_{0.0, 0.0, std::chrono::steady_clock::now(), false};
                GpsUtilities Utilities_{0, 0, 0.0, 0, std::chrono::steady_clock::now(), false};
                static constexpr std::chrono::seconds MAX_AGE_{2}; // Maximum age for valid data
        };
    }

}

#endif // MOMENTUM_GPS_DATA_HPP
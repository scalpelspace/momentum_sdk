/**
 * \file system_state.hpp
 * \brief
 *    System monitoring and state management for Momentum board.
 * \details
 *    Provides comprehensive system state management including status monitoring,
 *    sensor status tracking, real-time clock synchronization, and overall system
 *    diagnostics.
 * \author
 *    ScalpelSpace (info@scalpelspace.com)
 * \version 0.1
 * \date 2025-07-25
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

#ifndef MOMENTUM_SYSTEM_STATE_HPP
#define MOMENTUM_SYSTEM_STATE_HPP
#pragma once

#include <chrono>
#include <mutex>
#include <string>
#include <ctime>
#include <unordered_map>

#include "../protocol/system_messages.hpp"

namespace MomentumSDK {
    namespace SensorState{

        /**
         * \brief Enumeration of possible system states
         * \details
         *    Defines the hierarchical operational states of the ScalpelSpace Momentum
         *    system.
         *    
         *    State progression typically follows:
         *    Unknown -> when all sensors operational
         *    Normal  -> data aging or minor issues
         *    Warning -> sensor failures or CAN bus issues
         */
        enum class SystemState {
            Normal,   // All sensors operational
            Warning,  // Minor issues detected
            Error,    // Critical failures
            Unknown   // Initial state or indeterminate system condition
        };

        /**
         * \brief Comprehensive system status information
         * \details
         *    Contains complete system status information including current state,
         *    individual sensor status, timing information, and diagnostic messages.
         */
        struct SystemStatus {
            SystemState State;                                   
            std::chrono::steady_clock::time_point Timestamp;     
            std::string Message;                                  

            bool GpsStatus;                                       
            bool ImuStatus;                                      
            bool BarometricStatus;                               

            std::chrono::system_clock::time_point RtcTime;       
        };

        /**
         * \brief Thread-safe system state manager and status monitor
         * \details
         *    Manages comprehensive system status monitoring for the ScalpelSpace
         *    Momentum platform.
         */
        class SystemStateManager {
            public:

                /**
                 * \brief Constructs system state manager with default values
                 * \details
                 *    Initializes the system state manager with conservative defaults:
                 *    - System state set to Unknown
                 *    - All sensors marked as offline
                 *    - Timestamps set to current time
                 *    - Status message indicates initialization
                 */
                SystemStateManager() {

                    SystemStatus_.State = SystemState::Unknown;
                    SystemStatus_.Message = "System initializing!";
                    SystemStatus_.Timestamp = std::chrono::steady_clock::now();
                    SystemStatus_.GpsStatus = false;
                    SystemStatus_.ImuStatus = false;
                    SystemStatus_.BarometricStatus = false;
                    SystemStatus_.RtcTime = std::chrono::system_clock::now();

                }

                /**
                 * \brief Updates system state from hardware status message
                 * \details
                 *    Processes system state messages received from the Momentum
                 *    hardware platform.
                 * \param state_msg System state message from CAN bus
                 */
                void updateSystemState(const Protocol::State& state_msg) {
                    std::lock_guard<std::mutex> lock(state_mutex_);

                    SystemStatus_.State = static_cast<SystemState>(state_msg.getState());
                    SystemStatus_.Timestamp = std::chrono::steady_clock::now();

                    updateSystemStatus();
                }

                void updateRTC(const Protocol::RTC& rtc_msg) {
                    std::lock_guard<std::mutex> lock(state_mutex_);

                    auto year = rtc_msg.getRtcYear();
                    auto month = rtc_msg.getRtcMonth();
                    auto day = rtc_msg.getRtcDay();
                    auto hour = rtc_msg.getRtcHour();
                    auto minute = rtc_msg.getRtcMinute();
                    auto second = rtc_msg.getRtcSecond();

                    std::tm TimeInfo = {};
                    TimeInfo.tm_year = year - 1900;
                    TimeInfo.tm_mon = month - 1; 
                    TimeInfo.tm_mday = day;
                    TimeInfo.tm_hour = hour;
                    TimeInfo.tm_min = minute;
                    TimeInfo.tm_sec = second;

                    SystemStatus_.RtcTime = std::chrono::system_clock::from_time_t(std::mktime(&TimeInfo));
                    SystemStatus_.Timestamp = std::chrono::steady_clock::now();

                }

                SystemStatus getStatus() const {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    return SystemStatus_;
                }

                /**
                 * \brief Checks if the system status is operational
                 * \return true if system state is Normal, false otherwise
                 */
                bool Status() const {
                    return evaluateCurrentState() == SystemState::Normal;
                }

                SystemState evaluateCurrentState() const {
                    std::lock_guard<std::mutex> lock(state_mutex_);

                    if (!SystemStatus_.GpsStatus || !SystemStatus_.ImuStatus || !SystemStatus_.BarometricStatus) {
                        return SystemState::Error;
                    }

                    auto now = std::chrono::steady_clock::now();
                    auto DataAge = std::chrono::duration_cast<std::chrono::seconds>(now - SystemStatus_.Timestamp);

                    if (DataAge > std::chrono::seconds(5)) {
                        return SystemState::Warning;
                    }

                    if (SystemStatus_.State != SystemState::Normal) {
                        return SystemState::Warning;
                    }

                    return SystemState::Normal;
                }

                void setSensorStatus(const std::string& sensor, bool running) {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    
                    SensorStatusMap_[sensor] = running;
                    
                    if (sensor == "gps"){
                        SystemStatus_.GpsStatus = running;
                    } else if (sensor == "imu") {
                        SystemStatus_.ImuStatus = running;
                    } else if (sensor == "barometric") {
                        SystemStatus_.BarometricStatus = running;
                    }

                    updateSystemStatus();
                }
            
            private:

                mutable std::mutex state_mutex_;
                SystemStatus SystemStatus_;
                std::unordered_map<std::string, bool> SensorStatusMap_;

                void updateSystemStatus() {
                    auto new_status = evaluateCurrentState();
                    SystemStatus_.State = new_status;

                    switch (new_status) {
                        case SystemState::Normal:
                            SystemStatus_.Message = "Normal: momentum board running normal.";
                            break;
                        case SystemState::Warning:
                            SystemStatus_.Message = "Warning: check momentum board.";
                            break;
                        case SystemState::Error:
                            SystemStatus_.Message = "Error: Sensors or CAN bus from momentum board are offline.";
                            break;
                        default:
                            SystemStatus_.Message = "Unknown: momentum board state is unknown or initializing.";
                            break;
                    }
                }
        };
    }
}

#endif //MOMENTUM_STATE_HPP
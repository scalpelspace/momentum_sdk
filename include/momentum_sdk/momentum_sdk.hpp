/**
 * \file momentum_sdk.hpp
 * \brief
 *    Momentum SDK - Complete SDK include header
 * \details
 *    Convenience header that includes all components of Momentum SDK.
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

#ifndef MOMENTUM_SDK_HPP
#define MOMENTUM_SDK_HPP
#pragma once

#include "can/can_exceptions.hpp"
#include "can/can_frame.hpp"
#include "can/can_interface.hpp"
#include "can/can_utilities.hpp"

#include "protocol/message_parser.hpp"
#include "protocol/signal_utils.hpp"
#include "protocol/gps_messages.hpp"
#include "protocol/imu_messages.hpp"
#include "protocol/barometric_messages.hpp"
#include "protocol/system_messages.hpp"

#include "sensor_state/gps_data.hpp"
#include "sensor_state/imu_data.hpp"
#include "sensor_state/barometric_data.hpp"
#include "sensor_state/system_state.hpp"

#include <functional> 
#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <optional>
#include <condition_variable>

namespace MomentumSDK {
    
    // Forward declarations
    namespace CAN {
        class CANInterface;
        class CANFrame;
    }
    namespace Protocol {
        class GPS_3;
    }

    /**
     * @brief Error codes for Momentum operations
     */
    enum class MomentumError {
        None = 0,
        ConnectionFailed,
        DeviceNotFound,
        TimeoutError,
        DataStale,
        ConfigurationError,
        SystemError
    };
    
    // Version constants
    constexpr int VERSION_MAJOR = 1;
    constexpr int VERSION_MINOR = 0;
    constexpr int VERSION_PATCH = 0;
    
    /**
     * @brief Configuration structure for Momentum SDK
     */
    struct MomentumConfig {
        std::string can_interface = "can0";
        std::chrono::milliseconds data_timeout{1000};
        std::chrono::milliseconds connection_timeout{2000};
        bool auto_reconnect = true;
        bool enable_callbacks = true;
        size_t max_retry_attempts = 3;
    };
    
    /**
     * @brief Template for timestamped sensor readings
     */
    template<typename T>
    struct SensorReading {
        std::optional<T> value;
        std::chrono::system_clock::time_point timestamp;
        MomentumError status = MomentumError::None;
        
        bool isValid() const {
            return value.has_value() && status == MomentumError::None;
        }
        
        bool isFresh(std::chrono::milliseconds max_age) const {
            auto now = std::chrono::system_clock::now();
            return (now - timestamp) < max_age;
        }
    };

} // namespace MomentumSDK

#include "sensor_cache_manager.hpp"

namespace MomentumSDK {

    /**
     * @brief High-level wrapper class for Momentum board
     * 
     * The Momentum class provides a simple, thread-safe interface for accessing
     * sensor data from the Momentum board.
     */
    class Momentum {
    private:
        MomentumConfig config_;
        std::unique_ptr<CAN::CANInterface> can_interface_;
        std::unique_ptr<SensorState::GpsData> gps_data_;
        std::unique_ptr<SensorState::ImuData> imu_data_;
        std::unique_ptr<SensorState::BarometricData> barometric_data_;
        std::unique_ptr<SensorState::SystemStateManager> system_state_;
        
        std::unique_ptr<Protocol::GPS_3> gps3_data_;
        mutable std::mutex gps3_mutex_;
        
        std::atomic<bool> running_{false};
        std::atomic<bool> connected_{false};
        std::thread worker_thread_;
        mutable std::mutex data_mutex_;
        std::condition_variable data_cv_;
        
        SensorReading<SensorState::GpsPosition> cached_gps_position_;
        SensorReading<SensorState::GpsVelocity> cached_gps_velocity_;
        SensorReading<SensorState::IMUReading> cached_imu_data_;
        SensorReading<SensorState::BarometricMsg> cached_barometric_data_;
        
        // Callback function pointers
        void(*gps_position_callback_)(const SensorState::GpsPosition&) = nullptr;
        void(*gps_velocity_callback_)(const SensorState::GpsVelocity&) = nullptr;
        void(*imu_callback_)(const SensorState::IMUReading&) = nullptr;
        void(*barometric_callback_)(const SensorState::BarometricMsg&) = nullptr;
        void(*error_callback_)(MomentumError) = nullptr;

        // Internal methods for message processing
        void processMessages();
        void processCANFrame(const CAN::CANFrame& frame);
        bool isOperational() const;
        
        // Cache update methods
        void updateGpsPositionCache(const SensorState::GpsPosition& pos);
        void updateGpsVelocityCache(const SensorState::GpsVelocity& vel);
        void updateImuCache();
        void updateBarometricCache();
        void notifyError(MomentumError error);

    public:
        /**
         * @brief Construct Momentum with CAN interface name
         * @param can_interface Name of the CAN interface (e.g., "can0")
         */
        explicit Momentum(const std::string& can_interface = "can0");
        
        /**
         * @brief Construct Momentum with configuration
         * @param config Complete configuration structure
         */
        explicit Momentum(const MomentumConfig& config);
        
        /**
         * @brief Destructor - automatically disconnects
         */
        ~Momentum();
        
        /**
         * @brief Get SDK version string
         * @return Version string in format "major.minor.patch"
         */
        static std::string getVersion();
        
        /**
         * @brief Connect to CAN bus and start data processing
         * @return true if connection successful, false otherwise
         */
        bool connect();
        
        /**
         * @brief Disconnect from CAN bus and stop data processing
         */
        void disconnect();
        
        /**
         * @brief Check if currently connected to CAN bus
         * @return true if connected, false otherwise
         */
        bool isConnected() const;
        
        /**
         * @brief Get current configuration
         * @return Copy of current configuration
         */
        MomentumConfig getConfig() const;
        
        // Callback registration methods
        void onGpsPositionUpdate(void(*callback)(const SensorState::GpsPosition&));
        void onGpsVelocityUpdate(void(*callback)(const SensorState::GpsVelocity&));
        void onImuUpdate(void(*callback)(const SensorState::IMUReading&));
        void onBarometricUpdate(void(*callback)(const SensorState::BarometricMsg&));
        void onError(void(*callback)(MomentumError));
        
        // Data access methods
        std::optional<SensorState::GpsPosition> getGpsPosition() const;
        std::optional<SensorState::GpsVelocity> getGpsVelocity() const;
        std::optional<SensorState::IMUReading> getImuData() const;
        std::optional<SensorState::BarometricMsg> getBarometricData() const;
        
        /**
         * @brief Print current status to console
         */
        void printStatus() const;

    };

} // namespace MomentumSDK

#endif // MOMENTUM_SDK_HPP
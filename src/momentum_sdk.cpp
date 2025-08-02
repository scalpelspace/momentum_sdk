/**
 * \file momentum_sdk.cpp
 * \brief
 *    Implementation of the Momentum SDK main class
 * \details
 *    Provides implementation for CAN-based sensor data access and management.
 * \author
 *    ScalpelSpace (info@scalpelspace.com)
 * \version 1.0.0
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

#include "momentum_sdk/momentum_sdk.hpp"
#include <iostream>

namespace MomentumSDK {
    
    // Constructor - Initialize Momentum with CAN interface name
    Momentum::Momentum(const std::string& can_interface)
        : config_{can_interface}, running_(false), connected_(false) {

        // Create all the sensor data managers
        can_interface_ = std::make_unique<CAN::CANInterface>(can_interface);
        gps_data_ = std::make_unique<SensorState::GpsData>();
        imu_data_ = std::make_unique<SensorState::ImuData>();
        barometric_data_ = std::make_unique<SensorState::BarometricData>();
        system_state_ = std::make_unique<SensorState::SystemStateManager>();
    }
    
    // Constructor - Initialize Momentum with full configuration
    Momentum::Momentum(const MomentumConfig& config)
        : config_(config), running_(false), connected_(false) {

        // Create all the sensor data managers with custom config
        can_interface_ = std::make_unique<CAN::CANInterface>(config.can_interface);
        gps_data_ = std::make_unique<SensorState::GpsData>();
        imu_data_ = std::make_unique<SensorState::ImuData>();
        barometric_data_ = std::make_unique<SensorState::BarometricData>();
        system_state_ = std::make_unique<SensorState::SystemStateManager>();
    }

    // Destructor
    Momentum::~Momentum() {
        disconnect();
    }
    
    // Get SDK version as string (e.g., "1.0.0")
    std::string Momentum::getVersion() {
        return std::to_string(VERSION_MAJOR) + "." + 
               std::to_string(VERSION_MINOR) + "." + 
               std::to_string(VERSION_PATCH);
    }
    
    // Connect to CAN bus and start receiving sensor data
    bool Momentum::connect() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        try {

            // Open the CAN interface
            if (!can_interface_->open()) {
                notifyError(MomentumSDK::MomentumError::ConnectionFailed);
                return false;
            }
            
            // Mark as running and connected
            running_ = true;
            connected_ = true;
            
            // Start background thread to process incoming CAN messages
            worker_thread_ = std::thread(&Momentum::processMessages, this);
            
            return true;
            
        } catch (const std::exception& e) {
            notifyError(MomentumSDK::MomentumError::SystemError);
            return false;
        }
    }
    
    // Disconnect from CAN bus and stop processing
    void Momentum::disconnect() {

        // Stop the background processing
        running_ = false;
        connected_ = false;
        
        // Wait for worker thread to finish
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
        
        // Close CAN interface
        if (can_interface_) {
            can_interface_->close();
        }
    }
    
    // Check if currently connected to CAN bus
    bool Momentum::isConnected() const {
        return connected_.load();
    }
    
    // Get current configuration settings
    MomentumConfig Momentum::getConfig() const {
        return config_;
    }

    // Register callback function for GPS position updates
    void Momentum::onGpsPositionUpdate(void (*callback)(const MomentumSDK::SensorState::GpsPosition&)) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        gps_position_callback_ = callback;
    }
    
    // Register callback function for GPS velocity updates
    void Momentum::onGpsVelocityUpdate(void (*callback)(const MomentumSDK::SensorState::GpsVelocity&)) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        gps_velocity_callback_ = callback;
    }
    
    // Register callback function for IMU data updates
    void Momentum::onImuUpdate(void (*callback)(const MomentumSDK::SensorState::IMUReading&)) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        imu_callback_ = callback;
    }
    
    // Register callback function for barometric data updates
    void Momentum::onBarometricUpdate(void (*callback)(const MomentumSDK::SensorState::BarometricMsg&)) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        barometric_callback_ = callback;
    }
    
    // Register callback function for error notifications
    void Momentum::onError(void (*callback)(MomentumSDK::MomentumError)) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        error_callback_ = callback;
    }
    
    // Get current GPS position if data is fresh and valid
    std::optional<MomentumSDK::SensorState::GpsPosition> Momentum::getGpsPosition() const {
        return SensorCacheManager<MomentumSDK::SensorState::GpsPosition>::getCachedData(
            cached_gps_position_, config_, data_mutex_);
    }
    
    // Get current GPS velocity if data is fresh and valid
    std::optional<MomentumSDK::SensorState::GpsVelocity> Momentum::getGpsVelocity() const {
        return SensorCacheManager<MomentumSDK::SensorState::GpsVelocity>::getCachedData(
            cached_gps_velocity_, config_, data_mutex_);
    }
    
    // Get current IMU data if fresh and valid
    std::optional<MomentumSDK::SensorState::IMUReading> Momentum::getImuData() const {
        return SensorCacheManager<MomentumSDK::SensorState::IMUReading>::getCachedData(
            cached_imu_data_, config_, data_mutex_);
    }
    
    // Get current barometric data if fresh and valid
    std::optional<MomentumSDK::SensorState::BarometricMsg> Momentum::getBarometricData() const {
        return SensorCacheManager<MomentumSDK::SensorState::BarometricMsg>::getCachedData(
            cached_barometric_data_, config_, data_mutex_);
    }
    
    // Print current status and sensor data to console
    void Momentum::printStatus() const {
        std::cout << "\nMomentum v" << getVersion() << " Status:" << std::endl;
        std::cout << "Connected: " << (connected_.load() ? "Yes" : "No") << std::endl;
        
        // Show status of each sensor subsystem
        std::cout << "GPS Status: " << (gps_data_->Status() ? "Running" : "Unknown") << std::endl;
        std::cout << "IMU Status: " << (imu_data_->Status() ? "Running" : "Unknown") << std::endl;
        std::cout << "Barometric Status: " << (barometric_data_->Status() ? "Running" : "Unknown") << std::endl;
        std::cout << "System Status: " << (system_state_->Status() ? "Running" : "Unknown") << std::endl;
        
        // Show current sensor readings if available
        auto gps_pos = getGpsPosition();
        auto gps_vel = getGpsVelocity();
        
        if (gps_pos.has_value()) {
            std::cout << "GPS Position: " << gps_pos->Latitude_deg << ", " 
                      << gps_pos->Longitude_deg << ", " << gps_pos->Altitude_m << "m" << std::endl;
        }
        
        if (gps_vel.has_value()) {
            std::cout << "GPS Velocity: " << gps_vel->Speed_ms << " m/s, Course: " 
                      << gps_vel->Course_deg << " deg" << std::endl;
        }
    }

    // Background thread that continuously processes CAN messages
    void Momentum::processMessages() {
        while (running_.load()) {
            try {

                // Create empty frame to receive data
                MomentumSDK::CAN::CANFrame frame(0, {0,0,0,0,0,0,0,0});
                
                // Try to receive a frame with 100ms timeout
                if (can_interface_->receiveFrame(frame, 100)) {
                    processCANFrame(frame);
                }

            } catch (const std::exception& e) {
                
                // On error, notify and wait a bit before trying again
                notifyError(MomentumSDK::MomentumError::SystemError);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }

    // Process a single CAN frame based on its ID
    void Momentum::processCANFrame(const MomentumSDK::CAN::CANFrame& frame) {
        switch (frame.getID()) {
            case 257: {
                // System state message
                MomentumSDK::Protocol::State state_parser(frame.getFrame());
                system_state_->updateSystemState(state_parser);
                break;
            }
                
            case 258: {
                // Barometric sensor data
                MomentumSDK::Protocol::Barometric barometric_parser(frame.getFrame());
                barometric_data_->updateRead(barometric_parser);
                updateBarometricCache();
                break;
            }
                
            case 259: {
                // GPS position data (needs GPS_3 for complete info)
                MomentumSDK::Protocol::GPS_1 gps1_parser(frame.getFrame());
                std::lock_guard<std::mutex> lock(gps3_mutex_);
                if (gps3_data_) {
                    gps_data_->updatePosition(gps1_parser, *gps3_data_);
                    updateGpsPositionCache(gps_data_->getPosition());
                }
                break;
            }
                
            case 260: {
                // GPS velocity data
                MomentumSDK::Protocol::GPS_2 gps2_parser(frame.getFrame());
                gps_data_->updateVelocity(gps2_parser);
                updateGpsVelocityCache(gps_data_->getVelocity());
                
                std::lock_guard<std::mutex> lock(gps3_mutex_);
                if (gps3_data_) {
                    gps_data_->updateUtilities(gps2_parser, *gps3_data_);
                }
                break;
            }
                
            case 261: {
                // GPS utilities data (altitude, etc.)
                std::lock_guard<std::mutex> lock(gps3_mutex_);
                gps3_data_ = std::make_unique<MomentumSDK::Protocol::GPS_3>(frame.getFrame());
                break;
            }
                
            case 262: {
                // IMU quaternion data
                MomentumSDK::Protocol::IMU_1 imu1_parser(frame.getFrame());
                imu_data_->updateQuat(imu1_parser);
                updateImuCache();
                break;
            }
                
            case 263: {
                // IMU gyroscope data
                MomentumSDK::Protocol::IMU_2 imu2_parser(frame.getFrame());
                imu_data_->updateGyro(imu2_parser);
                updateImuCache();
                break;
            }
                
            case 264: {
                // IMU accelerometer data
                MomentumSDK::Protocol::IMU_3 imu3_parser(frame.getFrame());
                imu_data_->updateAccel(imu3_parser);
                updateImuCache();
                break;
            }
                
            case 265: {
                // IMU linear acceleration data
                MomentumSDK::Protocol::IMU_4 imu4_parser(frame.getFrame());
                imu_data_->updateLinearAccel(imu4_parser);
                updateImuCache();
                break;
            }
        }
    }
    
    // Check if all sensors are working properly
    bool Momentum::isOperational() const {
        return connected_.load() &&
               gps_data_->Status() && 
               imu_data_->Status() && 
               barometric_data_->Status() &&
               system_state_->Status();
    }

    // Update GPS position cache and call callback if registered
    void Momentum::updateGpsPositionCache(const SensorState::GpsPosition& pos) {
        SensorCacheManager<SensorState::GpsPosition>::updateCache(
            cached_gps_position_, pos, gps_position_callback_, config_, data_mutex_);
    }
    
    // Update GPS velocity cache and call callback if registered
    void Momentum::updateGpsVelocityCache(const SensorState::GpsVelocity& vel) {
        SensorCacheManager<SensorState::GpsVelocity>::updateCache(
            cached_gps_velocity_, vel, gps_velocity_callback_, config_, data_mutex_);
    }
    
    // Update IMU data cache and call callback if registered
    void Momentum::updateImuCache() {
        auto imu_read = imu_data_->getRead();
        SensorCacheManager<SensorState::IMUReading>::updateCache(
            cached_imu_data_, imu_read, imu_callback_, config_, data_mutex_);
    }
    
    // Update barometric data cache and call callback if registered
    void Momentum::updateBarometricCache() {
        auto baro_read = barometric_data_->getReading();
        SensorCacheManager<SensorState::BarometricMsg>::updateCache(
            cached_barometric_data_, baro_read, barometric_callback_, config_, data_mutex_);
    }
    
    // Send error notification to user callback if registered
    void Momentum::notifyError(MomentumSDK::MomentumError error) {
        if (config_.enable_callbacks && error_callback_) {
            error_callback_(error);
        }
    }

}
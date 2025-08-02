
/**
 * Basic example showing how to connect to a Momentum sensor and read data.
 * 
 * This program connects to a CAN interface, reads GPS, IMU, and barometric sensor data,
 * and prints the readings to the console every second. Press Ctrl+C to stop.
 * 
 * Usage:
 *     ./momentum_basic [can_interface]
 *     
 *     or 
 *     
 *     ./momentum_basic (default: "can0")
 *     
 * Example:
 *     ./momentum_basic can0
 *     ./momentum_basic
 */

#include "momentum_sdk/momentum_sdk.hpp"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>

class MomentumRead {
private:
    MomentumSDK::MomentumConfig config_;
    MomentumSDK::Momentum momentum_;
    std::atomic<bool> running_;
    std::thread status_thread_;

public:
    // Class to handle connection to Momentum board and print the data
    // acquired from sensors through CAN bus but with poll timing instead
    // of real-time.
    explicit MomentumRead(const std::string& can_interface = "can0")
        : momentum_(config_), running_(false) {
        
        // Set up the connection configuration for the Momentum board
        config_.can_interface = can_interface;
        config_.enable_callbacks = false;
    }

    // Function to try to connect to Momentum board
    bool connection() {
        std::cout << "Connecting to momentum: " << config_.can_interface << "..." << std::endl;
        
        if (momentum_.connect()) {
            std::cout << "Connected" << std::endl;
            running_ = true;
            return true;
        } else {
            std::cout << "Connection failed" << std::endl;
            return false;
        }
    }

    // Function to close all communication with Momentum board
    void disconnect() {
        std::cout << "Disconnecting from momentum..." << std::endl;
        running_ = false;
        
        if (status_thread_.joinable()) {
            status_thread_.join();
        }
        
        momentum_.disconnect();
    }

    // Function to print all the data received from Momentum board from GPS,
    // IMU, Barometric sensors.
    void dataPrint() {
        // Check if we have GPS position data available
        auto gps_pos = momentum_.getGpsPosition();
        if (gps_pos.has_value()) {
            const auto& pos = gps_pos.value();
            std::cout << std::fixed << std::setprecision(6);
            std::cout << "GPS Position: (" << pos.Latitude_deg << ", " 
                      << pos.Longitude_deg << ", " << pos.Altitude_m << ")" << std::endl;
        } else {
            std::cout << "No GPS Position data" << std::endl;
        }

        // Get all the IMU sensor readings - this includes accelerometer,
        // gyroscope, quaternion orientation, and linear acceleration
        auto imu_data = momentum_.getImuData();
        if (imu_data.has_value()) {
            const auto& imu = imu_data.value();
            
            // Print each sensor's reading
            std::cout << std::fixed << std::setprecision(3);
            std::cout << "Accelerometer (x,y,z): (" 
                      << imu.Accel_x << ", " << imu.Accel_y << ", " << imu.Accel_z << ")" << std::endl;
            std::cout << "Gyroscope (x,y,z): (" 
                      << imu.Gyro_x << ", " << imu.Gyro_y << ", " << imu.Gyro_z << ")" << std::endl;
            
            std::cout << std::fixed << std::setprecision(4);
            std::cout << "Quaternion (w,x,y,z): (" 
                      << imu.Quat_w << ", " 
                      << imu.Quat_x << ", " 
                      << imu.Quat_y << ", " 
                      << imu.Quat_z << ")" << std::endl;
            
            std::cout << std::fixed << std::setprecision(3);
            std::cout << "Linear Accel (x,y,z): (" 
                      << imu.LinearAccel_x << ", " 
                      << imu.LinearAccel_y << ", " 
                      << imu.LinearAccel_z << ")" << std::endl;
        } else {
            std::cout << "No IMU data" << std::endl;
        }

        // Check for atmospheric pressure and temperature readings
        auto baro_data = momentum_.getBarometricData();
        if (baro_data.has_value()) {
            const auto& baro = baro_data.value();
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "Barometric Data: (pressure: " << baro.pressure_pa 
                      << " Pa, temperature: " << baro.Temp_c << " C)" << std::endl;
        } else {
            std::cout << "No Barometric Data" << std::endl;
        }
    }

    // Main loop function
    bool run() {
        if (!connection()) {
            return false;
        }

        // Set up signal handlers
        std::signal(SIGINT, [](int) {
            std::cout << "\nKeyboard interrupt" << std::endl;
            std::exit(0);
        });

        // Start a background thread that prints data every second
        status_thread_ = std::thread([this]() {
            while (running_) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                if (running_) {
                    dataPrint();
                }
            }
        });

        // Loop to check if communication is still active
        try {
            while (running_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                if (!momentum_.isConnected()) {
                    std::cout << "Connection lost" << std::endl;
                    break;
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Exception: " << e.what() << std::endl;
        }

        disconnect();
        return true;
    }
};

int main(int argc, char* argv[]) {
    // A different CAN interface name could be input through command line
    // default: "can0"
    
    std::string can_interface = "can0";
    if (argc > 1) {
        can_interface = argv[1];
    }

    MomentumRead momentum(can_interface);

    try {
        bool success = momentum.run();
        return success ? 0 : 1;
    } catch (const std::exception& e) {
        std::cerr << "Something went wrong: " << e.what() << std::endl;
        return 1;
    }
}





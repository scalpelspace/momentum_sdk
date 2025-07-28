/**
 * \file momentum_bindings.cpp
 * \brief
 *    Python bindings for Momentum board
 * \details
 *    Provides Python interface using pybind11 for CAN-based sensor data access.
 * \author
 *    ScalpelSpace (info@scalpelspace.com)
 * \version 1.0.0
 * \date 2025-07-25
 * \copyright
 *    MIT License
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

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4127)
#endif

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>
#include <pybind11/functional.h>

// Include Momentum SDK header
#include "../include/momentum_sdk/momentum_sdk.hpp"

#ifdef _MSC_VER
#pragma warning(pop)
#endif

namespace py = pybind11;
using namespace MomentumSDK;

PYBIND11_MODULE(momentum_sdk, m) {
    m.doc() = "Momentum - CAN-based sensor data interface";
    
    // Module version and info
    m.attr("__version__") = "1.0.0";
    m.attr("__author__") = "ScalpelSpace";
    
    // Enums
    py::enum_<MomentumError>(m, "MomentumError", "Error codes for Momentum operations")
        .value("NONE", MomentumError::None, "No error")
        .value("CONNECTION_FAILED", MomentumError::ConnectionFailed, "Failed to connect to CAN interface")
        .value("DEVICE_NOT_FOUND", MomentumError::DeviceNotFound, "Momentum device not found")
        .value("TIMEOUT_ERROR", MomentumError::TimeoutError, "Operation timed out")
        .value("DATA_STALE", MomentumError::DataStale, "Sensor data is stale")
        .value("CONFIGURATION_ERROR", MomentumError::ConfigurationError, "Configuration error")
        .value("SYSTEM_ERROR", MomentumError::SystemError, "System error")
        .export_values();
    
    // Configuration structure
    py::class_<MomentumConfig>(m, "MomentumConfig", "Config settings for Momentum", py::dynamic_attr())
        .def(py::init<>(), "Create default config")
        .def_property("can_interface", 
            [](const MomentumConfig& self) -> std::string { return self.can_interface; },
            [](MomentumConfig& self, const std::string& val) { self.can_interface = val; },
            "CAN interface name (e.g. 'can0')")
        .def_property("data_timeout", 
            [](const MomentumConfig& self) -> std::chrono::milliseconds { return self.data_timeout; },
            [](MomentumConfig& self, const std::chrono::milliseconds& val) { self.data_timeout = val; },
            "Data timeout in milliseconds")
        .def_property("connection_timeout", 
            [](const MomentumConfig& self) -> std::chrono::milliseconds { return self.connection_timeout; },
            [](MomentumConfig& self, const std::chrono::milliseconds& val) { self.connection_timeout = val; },
            "Connection timeout in milliseconds")
        .def_property("auto_reconnect", 
            [](const MomentumConfig& self) -> bool { return self.auto_reconnect; },
            [](MomentumConfig& self, bool val) { self.auto_reconnect = val; },
            "Enable auto reconnection")
        .def_property("enable_callbacks", 
            [](const MomentumConfig& self) -> bool { return self.enable_callbacks; },
            [](MomentumConfig& self, bool val) { self.enable_callbacks = val; },
            "Enable callback notifications")
        .def_property("max_retry_attempts", 
            [](const MomentumConfig& self) -> int { return self.max_retry_attempts; },
            [](MomentumConfig& self, int val) { self.max_retry_attempts = val; },
            "Max retry attempts for operations")
        
        // Add __dir__ for better introspection
        .def("__dir__", [](const MomentumConfig&) -> std::vector<std::string> {
            return std::vector<std::string>{
                "can_interface", "data_timeout", "connection_timeout",
                "auto_reconnect", "enable_callbacks", "max_retry_attempts"
            };
        })
        
        .def("__repr__", [](const MomentumConfig& config) {
            return "<MomentumConfig(can_interface='" + config.can_interface + 
                   "', data_timeout=" + std::to_string(config.data_timeout.count()) + "ms)>";
        });
    
    // GPS Position structure
    py::class_<MomentumSDK::SensorState::GpsPosition>(m, "GpsPosition", "GPS position data", py::dynamic_attr())
        .def(py::init<>(), "Create empty GPS position")
        .def_property("latitude_deg", 
            [](const MomentumSDK::SensorState::GpsPosition& self) -> double { return self.Latitude_deg; },
            [](MomentumSDK::SensorState::GpsPosition& self, double val) { self.Latitude_deg = val; },
            "Latitude in degrees")
        .def_property("longitude_deg", 
            [](const MomentumSDK::SensorState::GpsPosition& self) -> double { return self.Longitude_deg; },
            [](MomentumSDK::SensorState::GpsPosition& self, double val) { self.Longitude_deg = val; },
            "Longitude in degrees")
        .def_property("altitude_m", 
            [](const MomentumSDK::SensorState::GpsPosition& self) -> double { return self.Altitude_m; },
            [](MomentumSDK::SensorState::GpsPosition& self, double val) { self.Altitude_m = val; },
            "Altitude in meters")
        .def_property("geoid_separation", 
            [](const MomentumSDK::SensorState::GpsPosition& self) -> double { return self.GeoidSeparation; },
            [](MomentumSDK::SensorState::GpsPosition& self, double val) { self.GeoidSeparation = val; },
            "Geoid separation in meters")
        .def_property("isValid", 
            [](const MomentumSDK::SensorState::GpsPosition& self) -> bool { return self.IsValid; },
            [](MomentumSDK::SensorState::GpsPosition& self, bool val) { self.IsValid = val; },
            "Data validity flag")
        
        // Convenience properties
        .def_property_readonly("position", 
            [](const MomentumSDK::SensorState::GpsPosition& pos) -> py::tuple {
                return py::make_tuple(pos.Latitude_deg, pos.Longitude_deg, pos.Altitude_m);
            }, 
            py::return_value_policy::copy,
            "Position as (lat, lon, alt) tuple")
        
        // Add __dir__ for better introspection
        .def("__dir__", [](const MomentumSDK::SensorState::GpsPosition&) -> std::vector<std::string> {
            return std::vector<std::string>{
                "latitude_deg", "longitude_deg", "altitude_m", 
                "geoid_separation", "isValid", "position"
            };
        })
        
        .def("__repr__", [](const MomentumSDK::SensorState::GpsPosition& pos) {
            return "<GpsPosition(valid=" + std::string(pos.IsValid ? "True" : "False") +
                   ", lat=" + std::to_string(pos.Latitude_deg) + 
                   ", lon=" + std::to_string(pos.Longitude_deg) + 
                   ", alt=" + std::to_string(pos.Altitude_m) + "m)>";
        });
    
    // GPS Velocity structure
    py::class_<MomentumSDK::SensorState::GpsVelocity>(m, "GpsVelocity", "GPS velocity data", py::dynamic_attr())
        .def(py::init<>(), "Create empty GPS velocity")
        .def_property("speed_ms", 
            [](const MomentumSDK::SensorState::GpsVelocity& self) -> double { return self.Speed_ms; },
            [](MomentumSDK::SensorState::GpsVelocity& self, double val) { self.Speed_ms = val; },
            "Speed in meters per second")
        .def_property("course_deg", 
            [](const MomentumSDK::SensorState::GpsVelocity& self) -> double { return self.Course_deg; },
            [](MomentumSDK::SensorState::GpsVelocity& self, double val) { self.Course_deg = val; },
            "Course in degrees")
        .def_property("isValid", 
            [](const MomentumSDK::SensorState::GpsVelocity& self) -> bool { return self.IsValid; },
            [](MomentumSDK::SensorState::GpsVelocity& self, bool val) { self.IsValid = val; },
            "Data validity flag")
        
        // Convenience properties
        .def_property_readonly("velocity", 
            [](const MomentumSDK::SensorState::GpsVelocity& vel) -> py::tuple {
                return py::make_tuple(vel.Speed_ms, vel.Course_deg);
            }, 
            py::return_value_policy::copy,
            "Velocity as (speed_m/s, course_deg) tuple")
        
        // Add __dir__ for better introspection
        .def("__dir__", [](const MomentumSDK::SensorState::GpsVelocity&) -> std::vector<std::string> {
            return std::vector<std::string>{
                "speed_ms", "course_deg", "isValid", "velocity"
            };
        })
        
        .def("__repr__", [](const MomentumSDK::SensorState::GpsVelocity& vel) {
            return "<GpsVelocity(valid=" + std::string(vel.IsValid ? "True" : "False") +
                   ", speed=" + std::to_string(vel.Speed_ms) + 
                   "m/s, course=" + std::to_string(vel.Course_deg) + " deg)>";
        });
    
    // IMU Data structure - improved with grouped access and explicit properties
    py::class_<MomentumSDK::SensorState::IMUReading>(m, "ImuData", "IMU sensor data", py::dynamic_attr())
        .def(py::init<>(), "Create empty IMU data")
        
        // Acceleration data (from IMU_3) - using explicit property syntax
        .def_property("accel_x", 
            [](const MomentumSDK::SensorState::IMUReading& self) -> double { return self.Accel_x; },
            [](MomentumSDK::SensorState::IMUReading& self, double val) { self.Accel_x = val; },
            "X acceleration (m/s^2)")
        .def_property("accel_y", 
            [](const MomentumSDK::SensorState::IMUReading& self) -> double { return self.Accel_y; },
            [](MomentumSDK::SensorState::IMUReading& self, double val) { self.Accel_y = val; },
            "Y acceleration (m/s^2)")
        .def_property("accel_z", 
            [](const MomentumSDK::SensorState::IMUReading& self) -> double { return self.Accel_z; },
            [](MomentumSDK::SensorState::IMUReading& self, double val) { self.Accel_z = val; },
            "Z acceleration (m/s^2)")
        
        // Gyroscope data (from IMU_2)
        .def_property("gyro_x", 
            [](const MomentumSDK::SensorState::IMUReading& self) -> double { return self.Gyro_x; },
            [](MomentumSDK::SensorState::IMUReading& self, double val) { self.Gyro_x = val; },
            "X angular rate (deg/s)")
        .def_property("gyro_y", 
            [](const MomentumSDK::SensorState::IMUReading& self) -> double { return self.Gyro_y; },
            [](MomentumSDK::SensorState::IMUReading& self, double val) { self.Gyro_y = val; },
            "Y angular rate (deg/s)")
        .def_property("gyro_z", 
            [](const MomentumSDK::SensorState::IMUReading& self) -> double { return self.Gyro_z; },
            [](MomentumSDK::SensorState::IMUReading& self, double val) { self.Gyro_z = val; },
            "Z angular rate (deg/s)")
        
        // Quaternion orientation (from IMU_1)
        .def_property("quat_w", 
            [](const MomentumSDK::SensorState::IMUReading& self) -> double { return self.Quat_w; },
            [](MomentumSDK::SensorState::IMUReading& self, double val) { self.Quat_w = val; },
            "Quaternion W")
        .def_property("quat_x", 
            [](const MomentumSDK::SensorState::IMUReading& self) -> double { return self.Quat_x; },
            [](MomentumSDK::SensorState::IMUReading& self, double val) { self.Quat_x = val; },
            "Quaternion X")
        .def_property("quat_y", 
            [](const MomentumSDK::SensorState::IMUReading& self) -> double { return self.Quat_y; },
            [](MomentumSDK::SensorState::IMUReading& self, double val) { self.Quat_y = val; },
            "Quaternion Y")
        .def_property("quat_z", 
            [](const MomentumSDK::SensorState::IMUReading& self) -> double { return self.Quat_z; },
            [](MomentumSDK::SensorState::IMUReading& self, double val) { self.Quat_z = val; },
            "Quaternion Z")
        
        // Gravity vector (from IMU_5)
        .def_property("gravity_x", 
            [](const MomentumSDK::SensorState::IMUReading& self) -> double { return self.Gravity_x; },
            [](MomentumSDK::SensorState::IMUReading& self, double val) { self.Gravity_x = val; },
            "X-axis gravity (m/s^2)")
        .def_property("gravity_y", 
            [](const MomentumSDK::SensorState::IMUReading& self) -> double { return self.Gravity_y; },
            [](MomentumSDK::SensorState::IMUReading& self, double val) { self.Gravity_y = val; },
            "Y-axis gravity (m/s^2)")
        .def_property("gravity_z", 
            [](const MomentumSDK::SensorState::IMUReading& self) -> double { return self.Gravity_z; },
            [](MomentumSDK::SensorState::IMUReading& self, double val) { self.Gravity_z = val; },
            "Z-axis gravity (m/s^2)")
        
        // Linear acceleration (from IMU_4)
        .def_property("linear_accel_x", 
            [](const MomentumSDK::SensorState::IMUReading& self) -> double { return self.LinearAccel_x; },
            [](MomentumSDK::SensorState::IMUReading& self, double val) { self.LinearAccel_x = val; },
            "X-axis linear acceleration (m/s^2)")
        .def_property("linear_accel_y", 
            [](const MomentumSDK::SensorState::IMUReading& self) -> double { return self.LinearAccel_y; },
            [](MomentumSDK::SensorState::IMUReading& self, double val) { self.LinearAccel_y = val; },
            "Y-axis linear acceleration (m/s^2)")
        .def_property("linear_accel_z", 
            [](const MomentumSDK::SensorState::IMUReading& self) -> double { return self.LinearAccel_z; },
            [](MomentumSDK::SensorState::IMUReading& self, double val) { self.LinearAccel_z = val; },
            "Z-axis linear acceleration (m/s^2)")
        
        .def_property("isValid", 
            [](const MomentumSDK::SensorState::IMUReading& self) -> bool { return self.IsValid; },
            [](MomentumSDK::SensorState::IMUReading& self, bool val) { self.IsValid = val; },
            "Data validity flag")
        
        // Convenience properties for grouped access with explicit return types
        .def_property_readonly("accel", 
            [](const MomentumSDK::SensorState::IMUReading& imu) -> py::tuple {
                return py::make_tuple(imu.Accel_x, imu.Accel_y, imu.Accel_z);
            }, 
            py::return_value_policy::copy,
            "Acceleration vector as (x, y, z) tuple")
        
        .def_property_readonly("gyro", 
            [](const MomentumSDK::SensorState::IMUReading& imu) -> py::tuple {
                return py::make_tuple(imu.Gyro_x, imu.Gyro_y, imu.Gyro_z);
            }, 
            py::return_value_policy::copy,
            "Gyroscope vector as (x, y, z) tuple")
        
        .def_property_readonly("quaternion", 
            [](const MomentumSDK::SensorState::IMUReading& imu) -> py::tuple {
                return py::make_tuple(imu.Quat_w, imu.Quat_x, imu.Quat_y, imu.Quat_z);
            }, 
            py::return_value_policy::copy,
            "Quaternion as (w, x, y, z) tuple")
        
        .def_property_readonly("gravity", 
            [](const MomentumSDK::SensorState::IMUReading& imu) -> py::tuple {
                return py::make_tuple(imu.Gravity_x, imu.Gravity_y, imu.Gravity_z);
            }, 
            py::return_value_policy::copy,
            "Gravity vector as (x, y, z) tuple")
        
        .def_property_readonly("linear_accel", 
            [](const MomentumSDK::SensorState::IMUReading& imu) -> py::tuple {
                return py::make_tuple(imu.LinearAccel_x, imu.LinearAccel_y, imu.LinearAccel_z);
            }, 
            py::return_value_policy::copy,
            "Linear acceleration vector as (x, y, z) tuple")
        
        // Add __dir__ for better introspection and autocomplete
        .def("__dir__", [](const MomentumSDK::SensorState::IMUReading&) -> std::vector<std::string> {
            return std::vector<std::string>{
                "accel_x", "accel_y", "accel_z", "accel",
                "gyro_x", "gyro_y", "gyro_z", "gyro",
                "quat_w", "quat_x", "quat_y", "quat_z", "quaternion",
                "gravity_x", "gravity_y", "gravity_z", "gravity",
                "linear_accel_x", "linear_accel_y", "linear_accel_z", "linear_accel",
                "isValid"
            };
        })
        
        .def("__repr__", [](const MomentumSDK::SensorState::IMUReading& imu) {
            return "<ImuData(valid=" + std::string(imu.IsValid ? "True" : "False") + 
                   ", accel=[" + std::to_string(imu.Accel_x) + "," + 
                   std::to_string(imu.Accel_y) + "," + std::to_string(imu.Accel_z) + 
                   "], quat=[" + std::to_string(imu.Quat_w) + "," + 
                   std::to_string(imu.Quat_x) + "," + std::to_string(imu.Quat_y) + "," + 
                   std::to_string(imu.Quat_z) + "])>";
        });
    
    // Barometric Data structure
    py::class_<MomentumSDK::SensorState::BarometricMsg>(m, "BarometricData", "Barometric sensor data", py::dynamic_attr())
        .def(py::init<>(), "Create empty barometric data")
        .def_property("pressure_pa", 
            [](const MomentumSDK::SensorState::BarometricMsg& self) -> double { return self.pressure_pa; },
            [](MomentumSDK::SensorState::BarometricMsg& self, double val) { self.pressure_pa = val; },
            "Pressure in Pascals")
        .def_property("temp_c", 
            [](const MomentumSDK::SensorState::BarometricMsg& self) -> double { return self.Temp_c; },
            [](MomentumSDK::SensorState::BarometricMsg& self, double val) { self.Temp_c = val; },
            "Temperature in Celsius")
        .def_property("barometricState", 
            [](const MomentumSDK::SensorState::BarometricMsg& self) -> int { return self.BarometricState; },
            [](MomentumSDK::SensorState::BarometricMsg& self, int val) { self.BarometricState = val; },
            "Sensor state (0: Normal, 1: Warning, 2: Error, 3: Critical)")
        .def_property("isValid", 
            [](const MomentumSDK::SensorState::BarometricMsg& self) -> bool { return self.IsValid; },
            [](MomentumSDK::SensorState::BarometricMsg& self, bool val) { self.IsValid = val; },
            "Data validity flag")
        
        // Convenience properties
        .def_property_readonly("atmospheric_data", 
            [](const MomentumSDK::SensorState::BarometricMsg& baro) -> py::tuple {
                return py::make_tuple(baro.pressure_pa, baro.Temp_c);
            }, 
            py::return_value_policy::copy,
            "Atmospheric data as (pressure_Pa, temp_C) tuple")
        
        // Add __dir__ for better introspection
        .def("__dir__", [](const MomentumSDK::SensorState::BarometricMsg&) -> std::vector<std::string> {
            return std::vector<std::string>{
                "pressure_pa", "temp_c", "barometricState", "isValid", "atmospheric_data"
            };
        })
        
        .def("__repr__", [](const MomentumSDK::SensorState::BarometricMsg& baro) {
            return "<BarometricData(valid=" + std::string(baro.IsValid ? "True" : "False") +
                   ", pressure=" + std::to_string(baro.pressure_pa) + 
                   "Pa, temp=" + std::to_string(baro.Temp_c) + "C, state=" + 
                   std::to_string(baro.BarometricState) + ")>";
        });
    
    // Main Momentum class
    py::class_<MomentumSDK::Momentum>(m, "Momentum", R"pbdoc(
        
        Main interface for Momentum board.
        
        Provides thread-safe access to GPS, IMU, and barometric sensors over CAN bus.
        Handles connection management and real-time data callbacks automatically.
        
        Example:
            import momentum_sdk as sm
            m = sm.Momentum("can0")
            m.connect()
            
            # Get sensor data
            gps = m.get_gps_position()
            if gps and gps.isValid:
                print(f"Position: {gps.latitude_deg}, {gps.longitude_deg}")
        )pbdoc")
        .def(py::init<const std::string&>(), 
             py::arg("can_interface") = "can0",
             "Create Momentum instance")
        .def(py::init<const MomentumConfig&>(),
             py::arg("config"),
             "Create Momentum instance with config")
        
        // Static methods
        .def_static("get_version", &MomentumSDK::Momentum::getVersion,
                   "Get SDK version")
        
        // Connection management
        .def("connect", &MomentumSDK::Momentum::connect,
             "Connect to CAN bus")
        .def("disconnect", &MomentumSDK::Momentum::disconnect,
             "Disconnect from CAN bus")
        .def("is_connected", &MomentumSDK::Momentum::isConnected,
             "Check connection status")
        
        // Configuration
        .def("get_config", &MomentumSDK::Momentum::getConfig,
             "Get current config")
        
        // Callback registration with explicit type hints
        .def("on_gps_position_update", &MomentumSDK::Momentum::onGpsPositionUpdate,
             py::arg("callback"),
             "Register GPS position callback\n\n"
             "Args:\n"
             "    callback: Function taking GpsPosition parameter")
        .def("on_gps_velocity_update", &MomentumSDK::Momentum::onGpsVelocityUpdate,
             py::arg("callback"),
             "Register GPS velocity callback\n\n"
             "Args:\n"
             "    callback: Function taking GpsVelocity parameter")
        .def("on_imu_update", &MomentumSDK::Momentum::onImuUpdate,
             py::arg("callback"),
             "Register IMU data callback\n\n"
             "Args:\n"
             "    callback: Function taking ImuData parameter")
        .def("on_barometric_update", &MomentumSDK::Momentum::onBarometricUpdate,
             py::arg("callback"),
             "Register barometric data callback\n\n"
             "Args:\n"
             "    callback: Function taking BarometricData parameter")
        .def("on_error", &MomentumSDK::Momentum::onError,
             py::arg("callback"),
             "Register error callback\n\n"
             "Args:\n"
             "    callback: Function taking MomentumError parameter")
        
        // Data access
        .def("get_gps_position", &MomentumSDK::Momentum::getGpsPosition,
             "Get current GPS position")
        .def("get_gps_velocity", &MomentumSDK::Momentum::getGpsVelocity,
             "Get current GPS velocity")
        .def("get_imu_data", &MomentumSDK::Momentum::getImuData,
             "Get current IMU data")
        .def("get_barometric_data", &MomentumSDK::Momentum::getBarometricData,
             "Get current barometric data")
        
        // Utility methods
        .def("print_status", &MomentumSDK::Momentum::printStatus,
             "Print status info")
        
        .def("__repr__", [](const MomentumSDK::Momentum& m) {
            return "<Momentum(connected=" + std::string(m.isConnected() ? "True" : "False") + 
                   ", version=" + MomentumSDK::Momentum::getVersion() + ")>";
        });
    
    // Module-level convenience functions
    m.def("create_momentum", 
          [](const std::string& can_interface = "can0") {
              return std::make_unique<MomentumSDK::Momentum>(can_interface);
          },
          py::arg("can_interface") = "can0",
          "Create a new Momentum instance");
    
    m.def("list_can_interfaces", 
          []() {
              return std::vector<std::string>{"can0", "can1", "vcan0", "vcan1"};
          },
          "List available CAN interfaces");
}

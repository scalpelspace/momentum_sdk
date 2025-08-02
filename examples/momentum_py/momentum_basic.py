"""
Basic example showing how to connect to a Momentum sensor and read data.

This script connects to a CAN interface, reads GPS, IMU, and barometric sensor data,
and prints the readings to the console every second. Press Ctrl+C to stop.

Usage:
    python3 mometum_basic.py [can_interface] 
    
    or 
    
    python3 momentum_basic.py if default:"can0"
    
Example:
    python3 mometum_basic.py can1

    python3 momentum_basic.py
"""

import momentum_sdk as momentum

import sys
import time
import signal
import threading


class MomentumRead:
    
    """Class to handle connection to Momentum board and print the data
    acquired from sensors through CAN bus but with poll timing instead
    of real-time."""

    def __init__(self, CANInterface: str = "can0"):
        
        """Set up the connection configuration for the Momentum board."""
        
        self.config = momentum.MomentumConfig()
        self.config.can_interface = CANInterface
        self.config.enable_callbacks = False

        self.momentum = momentum.Momentum(self.config)

        self.running = False


    def connection(self) -> bool:
        
        """Function to try to connect to Momentum board"""

        print(f"Connecting to momentum: {self.config.can_interface}...")

        if self.momentum.connect():
            print("Connected")
            self.running = True
            return True
        else: 
            print("Connection failed")
            return False
        
    def disconnect(self):
        
        """Function to close all communication with Momentum board"""

        print("Disconnecting from momentum...")
        self.momentum.disconnect()
        self.running = False

    def DataPrint(self):
        
        """Function to print all the data received from Momentum board from GPS,
        IMU, Barometric sensors."""
        
        # Check if we have GPS position data available
        gps_pos = self.momentum.get_gps_position()
        if gps_pos:
            pos = gps_pos.position
            print(f"GPS Position: ({pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f})")
        else:
            print("No GPS Position data")
        
        # Get all the IMU sensor readings - this includes accelerometer,
        # gyroscope, quaternion orientation, and linear acceleration
        imu_data = self.momentum.get_imu_data()
        if imu_data:
            accel = imu_data.accel
            gyro = imu_data.gyro
            quat = imu_data.quaternion
            lin_accel = imu_data.linear_accel
            
            # Print each sensor's reading
            print(f"Accelerometer (x,y,z): ({accel[0]:.3f}, {accel[1]:.3f}, {accel[2]:.3f})")
            print(f"Gyroscope (x,y,z): ({gyro[0]:.3f}, {gyro[1]:.3f}, {gyro[2]:.3f})")
            print(f"Quaternion (w,x,y,z): ({quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f})")
            print(f"Linear Accel (x,y,z): ({lin_accel[0]:.3f}, {lin_accel[1]:.3f}, {lin_accel[2]:.3f})")
            
        else:
            print("No IMU data")
        
        # Check for atmospheric pressure and temperature readings
        baro_data = self.momentum.get_barometric_data()
        
        if baro_data: 
            atm_data = baro_data.atmospheric_data
            print(f"Barometric Data: (pressure: {atm_data[0]:.2f} Pa, temperature: {atm_data[1]:.2f} C)")
        
        else:
            print("No Barometric Data")

    def run(self):
        
        """Main loop function"""

        if not self.connection():
            return False
        
        # Set up signal handlers
        def signal_handler(signum, frame):
            self.running = False

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        # Start a background thread that prints data every second
        def status():
            while self.running:
                time.sleep(1)
                if self.running:
                    self.DataPrint()
        status_thread = threading.Thread(target=status, daemon=True)
        status_thread.start()

        #Loop to check if communication is still active
        try:
            while self.running:
                time.sleep(0.1)

                if not self.momentum.is_connected():
                    print("Connection lost")
                    break
        except KeyboardInterrupt:
            print("\nKeyboard interrupt")
        
        finally:
            self.disconnect()

        return True

def main():
    
    """A different CAN interface name could be input throught terminal
    default: "can0" """

    CANInterface = "can0"
    if len(sys.argv) > 1:
        CANInterface = sys.argv[1]
    
    momentum = MomentumRead(CANInterface)

    try:
        succes = momentum.run()
        sys.exit(0 if succes else 1)
    except Exception as e:
        print(f"Something went wrong: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()

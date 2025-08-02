/**
 * \file can_interface.hpp
 * \brief
 *    C++ interface for SocketCAN
 * \details
 *    Provides a high-level C++ wrapper for the Linux SocketCAN interface.
 * \author
 *    ScalpelSpace (info@scalpelspace.com)
 * \version 0.1
 * \date 2025-07-20
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

#ifndef MOMENTUM_CAN_INTERFACE
#define MOMENTUM_CAN_INTERFACE
#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "../can/can_frame.hpp"

namespace MomentumSDK{
    namespace CAN{
    
      /**
       * \brief
       *    A C++ wrapper class for SocketCAN interfaces
       * \note
       *    This class is move-constructible and move-assignable but not copy-constructible 
       *    or copy-assignable due to socket resource management.
       */
      class CANInterface{
        public:

        /**
         * \brief
         *    Constructor for the CAN interface
         * \param[in] InterfaceName
         *    The name of the CAN interface, such as "can0", "slcan0"
         * \param[in] SendTimeout
         *    The timeout for sending a CAN frame
         * \param[in] ReceivedTimeout
         *    The timeout for receiving a CAN frame
         * \param[in] IsSignalErrors
         *    Whether to signal errors or not
         */
        CANInterface(std::string const& InterfaceName, std::chrono::microseconds const& SendTimeout = std::chrono::seconds(1),
                      std::chrono::microseconds const& ReceivedTimeout = std::chrono::seconds(1), bool const IsSignalErrors = true);
        CANInterface() = delete; // No empty CAN interface
        CANInterface(CANInterface const&) = delete; // No copy constructor
        CANInterface& operator = (CANInterface const&) = default; // No copy assignment
        CANInterface(CANInterface&&) = default; // No move constructor
        CANInterface& operator = (CANInterface&&) = default; // No move assignment
        ~CANInterface(); // No destructor

        /**
         * \brief
         *    Enables or disables the loopback mode for the CAN interface
         * \param[in] IsLoopback
         *    True to enable loopback mode, false to disable it
         */
        void setLoopback(bool const IsLoopback);

        /**
         * \brief
         *    Sets up filtering for received CAN frames
         * \param[in] CAN_IDs
         *    Vector of CAN IDs to filter for
         * \param[in] IsInvert
         *    If true, inverts the filter (accepts all except specified IDs)
         */
        void setRecvFilter(std::vector<std::uint32_t> const& CAN_IDs, bool const IsInvert = false);

        /**
         * \brief
         *    Sets the timeout duration for sending CAN frames
         * \param[in] SendTimeout
         *    The timeout duration in microseconds
         */
        void setSendTimeout(std::chrono::microseconds const& SendTimeout);

        /**
         * \brief
         *    Sets the timeout duration for receiving CAN frames
         * \param[in] ReceivedTimeout
         *    The timeout duration in microseconds
         */
        void setReceivedTimeout(std::chrono::microseconds const& ReceivedTimeout);

        /**
         * \brief
         *    Configures whether the interface should signal errors
         * \param[in] IsSignalErrors
         *    True to enable error signaling, false to disable it
         */
        void setSignalErrors(bool const IsSignalErrors);

        /**
         * \brief
         *    Writes a CAN frame to the interface using a CANFrame object
         * \param[in] Frame
         *    The CANFrame object containing the frame data to send
         */
        void writeFrame(CANFrame const& Frame);

        /**
         * \brief
         *    Writes a CAN frame to the interface using raw ID and data
         * \param[in] CAN_ID
         *    The CAN ID for the frame
         * \param[in] Frame
         *    Array of 8 bytes containing the frame data
         */
        void writeFrame(std::uint32_t const CAN_ID, std::array<std::uint8_t,8> const& Frame);

        bool readFrame(CANFrame& Frame, std::uint32_t const Timeout_ms = 0);

        template<class Rep, class Period>
        inline bool readFrame(CANFrame& Frame, std::chrono::duration<Rep, Period> const& Timeout) {
            auto Timeout_ms = std::chrono::duration_cast<std::chrono::milliseconds>(Timeout);
            return readFrame(Frame, static_cast<std::uint32_t>(Timeout_ms.count()));
        }

        template<class Rep, class Period>
        inline bool receiveFrame(CANFrame& Frame, std::chrono::duration<Rep, Period> const& Timeout) {
            return readFrame(Frame, Timeout);
        }

        inline bool receiveFrame(CANFrame& Frame, std::uint32_t const Timeout_ms = 0) {
            return readFrame(Frame, Timeout_ms);
        }

        /**
         * \brief
         *    Opens the CAN interface for communication
         * \return
         *    True if the interface was successfully opened, false otherwise
         */
        bool open();

        /**
         * \brief
         *    Closes the CAN interface
         * \note
         *    This function is noexcept and will not throw exceptions
         */
        void close() noexcept;

        protected:
          /**
           * \brief
           *    Initializes the SocketCAN interface
           * \param[in] InterfaceName
           *    The name of the CAN interface to initialize
           */
          void initSocketCAN(std::string const& InterfaceName);

          /**
           * \brief
           *    Safely closes the SocketCAN interface
           * \note
           *    This function is noexcept and will not throw exceptions
           */
          void closeSocketCAN() noexcept;

          std::string InterfaceName_;  
          int socket_;                  
      };
      
    }
}

#endif //MOMENTUM_CAN_INTERFACE
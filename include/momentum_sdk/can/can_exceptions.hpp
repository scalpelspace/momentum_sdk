/**
 * \file can_exceptions.hpp
 * \brie#include <string>
#include <stdexcept>

 namespace MomentumSDK{    Exception classes for CAN communication errors
 * \details
 *    Provides a hierarchy of exception classes for handling various
 *    CAN communication errors, including socket errors, bus problems,
 *    protocol violations, and hardware-related issues
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

 #ifndef MOMENTUM_CAN_EXCEPTIONS
 #define MOMENTUM_CAN_EXCEPTIONS
 #pragma once

 #include <stdexcept>
 #include <system_error>

 namespace MomentumSDK{
    namespace CAN{
      
        /**
         * \brief 
         *    Exception for Linux SocketCAN socket errors
         */
        class SocketException: public std::system_error{
            public:
             using std::system_error::system_error;
        };

        /**
         * \brief
         *    Base exception class for CAN-specific errors
         * \see
         *    https://github.com/linux-can/can-utils/blob/master/include/linux/can/error.h
         */
        class Exception: public std::runtime_error{
            public:
              using std::runtime_error::runtime_error;
        };

        /**
         * \brief
         *    Exception for CAN frame transmission timeouts
         */
        class TxTimeout: public Exception{
            public:
              using Exception::Exception;
        };

        /**
         * \brief
         *    Exception for CAN bus arbitration errors
         * \see
         *    https://www.cancapture.com/article/arbitration-lost-error-messages 
         */
        class LostArbitrationErr: public Exception{
            public:
              using Exception::Exception;
        };

        /**
         * \brief
         *    Exception for CAN bus controller problems
         */
        class ControllerProblemErr: public Exception{
            public:
              using Exception::Exception;
        };

        /**
         * \brief
         *    Exception for CAN bus protocol violations
         *    Check: https://www.csselectronics.com/pages/can-bus-errors-intro-tutorial
         */
        class ProtocolViolationErr: public Exception{
            public:
              using Exception::Exception;
        };

        /**
         * \brief
         *    Exception for CAN transceiver state errors
         */
        class TransceiverStateErr: public Exception{
            public:
              using Exception::Exception;
        };

        /**
         * \brief
         *    Exception for unacknowledged CAN messages
         */
        class NoAcknowledeErr: public Exception{
            public:
              using Exception::Exception;
        };

        /**
         * \brief
         *    Exception for CAN node bus-off state
         */
        class BusoffErr: public Exception{
            public:
              using Exception::Exception;
        };

        /**
         * \brief
         *    Exception for general CAN bus errors
         */
        class BusErr: public Exception{
            public:
              using Exception::Exception;
        };

        /**
         * \brief
         *    Exception for CAN controller restart events
         */
        class ControllerRestartErr: public Exception{
            public:
              using Exception::Exception;
        };

        /**
         * \brief
         *    General CAN exception alias for the base Exception class
         * \details
         *    Provides a convenient alias for general CAN-related errors.
         */
        using CANException = Exception;
    }
 }

#endif // MOMENTUM_CAN_EXCEPTIONS
/**
 * \file can_utilities.hpp
 * \brief
 *    Utility functions for CAN communication
 * \details
 *    Provides utilities for CAN frame output formatting and time conversion
 *    functions needed for CAN socket operations
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

#ifndef MOMENTUM_CAN_UTILITIES
#define MOMENTUM_PACE_CAN_UTILITIES

#include <chrono>
#include <ostream>
#include <ratio>

#include <linux/can.h>
#include <sys/time.h>

/**
 * \brief
 *    Output stream operator for CAN frames
 * \param[in,out] os
 *    The output stream to write to
 * \param[in] Frame
 *    The CAN frame to format
 * \return
 *    Reference to the output stream
 * \note
 *    This function is noexcept and will not throw exceptions
 */
std::ostream& operator << ( std::ostream& os, struct ::can_frame const& Frame) noexcept;

namespace MomentumSDK{
    namespace CAN{
        /**
         * \brief
         *    Converts a std::chrono duration to a timeval struct
         * \tparam Rep
         *    The representation type of the duration
         * \tparam Period
         *    The period type of the duration
         * \param[in] duration
         *    The duration to convert
         * \return
         *    A timeval struct representing the duration
         * \note
         *    This function is constexpr and noexcept
         */
        template <class Rep, class Period>
        [[nodiscard]]
        constexpr struct ::timeval toTimeval(std::chrono::duration<Rep, Period> const& Duration) noexcept {
            auto const microseconds { std::chrono::duration_cast<std::chrono::duration<Rep, std::micro>>(Duration)};
            struct ::timeval tv{};
            tv.tv_sec = static_cast<::time_t>(microseconds.count()/std::micro::den);
            tv.tv_usec = static_cast<long int>(microseconds.count())% std::micro::den;
            return tv;

        };
    }
    
}

#endif //MOMENTUM_CAN_UTILITIES
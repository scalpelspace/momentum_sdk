/**
 * \file can_frame.hpp
 * \brief
 *    CAN bus protocol frame representation
 * \details
 *    Provides a class for handling CAN frames, supporting 8-byte data frames
 *    with identifiers. Includes constexpr operations for compile-time frame 
 *    manipulation and efficient runtime performance
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

 #ifndef MOMENTUM_CAN_FRAME
 #define MOMENTUM_CAN_FRAME
 #pragma once

 #include <array>
 #include <cstdint>

 namespace MomentumSDK{
    namespace CAN{
        /**
         * \brief
         *    A class representing a CAN bus protocol frame
         */
        class CANFrame{
            public:
            /**
             * \brief
             *    Creates a new CAN frame
             * \param[in] CAN_ID
             *    The CAN frame identifier
             * \param[in] frame
             *    The 8-byte data frame
             */
            constexpr CANFrame(std::uint32_t const CAN_ID, std::array<std::uint8_t,8> frame) noexcept;
            CANFrame() = delete; //empty frame CANNOT be created
            CANFrame(CANFrame const&) = default; //A frame can be copied
            CANFrame& operator = (CANFrame const&) = default; // Assigning one frame to another is allowed
            CANFrame(CANFrame&&) = default; // A frame can be moved
            CANFrame& operator = (CANFrame&&) = default; // move-assign a frame is allowed

            /**
             * \brief 
             *    Gets the CAN frame identifier
             * \return
             *    The CAN frame ID
             */
            [[nodiscard]]
            constexpr std::uint32_t getID() const noexcept;

            /**
             * \brief
             *    Gets the CAN frame data
             * \return
             *    The 8-byte frame data
             */
            [[nodiscard]]
            constexpr std::array<std::uint8_t,8> const& getFrame() const noexcept;

        protected:
          std::uint32_t CAN_ID_;
          std::array<std::uint8_t,8> frame_;
        };

        constexpr CANFrame::CANFrame(std::uint32_t const CAN_ID, std::array<std::uint8_t,8> frame) noexcept
        : CAN_ID_{CAN_ID}, frame_{frame}{
            return;
        }

        constexpr std::uint32_t CANFrame::getID() const noexcept{
            return CAN_ID_;
        }

        constexpr std::array<std::uint8_t,8> const& CANFrame::getFrame() const noexcept{
            return frame_;
        }
    }
 }

 #endif //MOMENTUM_CAN_FRAME
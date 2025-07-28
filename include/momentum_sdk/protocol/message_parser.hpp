/**
 * \file message_parser.hpp
 * \brief
 *    Data container for CAN message handling
 * \details
 *    Provides data storage and basic access methods for CAN messages.
 *    Signal extraction is performed using momentum_driver functions.
 * \author
 *    ScalpelSpace (info@scalpelspace.com)
 * \version 0.1
 * \date 2025-07-22
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

#ifndef MOMENTUM_MESSAGE_PARSER_HPP
#define MOMENTUM_MESSAGE_PARSER_HPP
#pragma once

#include <array>
#include "signal_utils.hpp"

namespace MomentumSDK {
    namespace Protocol {

        /**
         * @brief A data container class for CAN message handling
         * 
         * MessageParser provides data storage and basic access methods for CAN messages.
         * Signal extraction is handled by momentum_driver functions for accuracy and consistency.
         */
        class MessageParser{
        public:
        /**
         * @brief Get the raw CAN message data
         * 
         * @return const reference to array containing the raw message data
         */
        [[nodiscard]]
        constexpr const std::array<std::uint8_t, 8>& getData() const noexcept {
            return Data_;
        }

        /**
         * @brief Get the CAN message ID
         * 
         * @return The message ID as a 32-bit unsigned integer
         */
        [[nodiscard]]
        constexpr std::uint32_t getID() const noexcept {
          return Id_;  
        }

        /**
         * @brief Get the length of the CAN message data
         * 
         * @return The length of the message data in bytes
         */
        [[nodiscard]]
        constexpr std::uint8_t getLength() const noexcept {
            return Length_;
        }
    protected:
        /**
         * @brief Construct a new Message Parser object
         * 
         * @param id The CAN message ID
         * @param length The length of the message data in bytes
         * @param data The raw message data (default empty array)
         */
        constexpr MessageParser(std::uint32_t id, std::uint8_t length, const std::array<std::uint8_t, 8>& data = {}) noexcept
        : Id_(id), Length_(length), Data_(data) {}

    private:
        std::uint32_t Id_;      
        std::uint8_t Length_;  
        std::array<std::uint8_t, 8> Data_;  
       };   
    }
}

#endif // MOMENTUM_MESSAGE_PARSER_HPP
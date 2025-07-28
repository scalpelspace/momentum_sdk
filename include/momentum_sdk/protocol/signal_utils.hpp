/**
 * \file signal_utils.hpp
 * \brief
 *    Common utilities for CAN signal processing
 * \details
 *    Provides shared utility functions for working with CAN signals across
 *    different message parsers.
 * \author
 *    ScalpelSpace (info@scalpelspace.com)
 * \version 0.1
 * \date 2025-07-26
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

#ifndef MOMENTUM_SIGNAL_UTILS_HPP
#define MOMENTUM_SIGNAL_UTILS_HPP
#pragma once

extern "C" {
    #include "momentum_driver.h"
}

#include <cstring>
#include <cstdint>

namespace MomentumSDK {
    namespace Protocol {
        
        /**
         * @brief Helper function to find a signal in a message by name
         * @param message_id The CAN message ID
         * @param signal_name The name of the signal to find
         * @return Pointer to the signal definition, or nullptr if not found
         */
        inline const can_signal_t* findSignal(uint32_t message_id, const char* signal_name) {
            for (int i = 0; i < dbc_message_count; ++i) {
                if (dbc_messages[i].message_id == message_id) {
                    for (uint8_t j = 0; j < dbc_messages[i].signal_count; ++j) {
                        if (strcmp(dbc_messages[i].signals[j].name, signal_name) == 0) {
                            return &dbc_messages[i].signals[j];
                        }
                    }
                    break;
                }
            }
            return nullptr;
        }
        
    } 
} 

#endif // MOMENTUM_SIGNAL_UTILS_HPP

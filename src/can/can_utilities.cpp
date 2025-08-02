/**
 * \file can_utilities.cpp
 * \brief
 *    Implementation of CAN utility functions
 * \details
 *    Provides implementations for CAN frame formatting and utility functions
 *    for SocketCAN operations
 * \author
 *    ScalpelSpace (info@scalpelspace.com)
 * \version 0.1
 * \date 2025-07-25
 * \copyright
 *    MIT License
 * 
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

#include "../../include/momentum_sdk/can/can_utilities.hpp"

#include <iomanip>
#include <sstream>

std::ostream& operator<<(std::ostream& os, struct ::can_frame const& frame) noexcept {
    try {
        // Format CAN ID
        os << "CAN ID: 0x" << std::hex << std::uppercase << std::setw(3) << std::setfill('0') 
           << frame.can_id << " ";
        
        // Format data length
        os << "DLC: " << std::dec << static_cast<int>(frame.can_dlc) << " ";
        
        // Format data bytes
        os << "Data: ";
        for (int i = 0; i < frame.can_dlc && i < 8; ++i) {
            os << std::hex << std::uppercase << std::setw(2) << std::setfill('0') 
               << static_cast<int>(frame.data[i]);
            if (i < frame.can_dlc - 1) {
                os << " ";
            }
        }
        
        // Reset stream formatting
        os << std::dec << std::nouppercase;
        
    } catch (...) {
        os << "CAN Frame (format error)";
    }
    
    return os;
}


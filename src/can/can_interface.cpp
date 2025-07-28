/**
 * \file can_interface.cpp
 * \brief
 *    SocketCAN implementation for CAN bus communication
 * \details
 *    Provides Linux SocketCAN implementation for the CAN_Interface class.
 *    Handles socket creation, configuration, and data transmission/reception.
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

#include "../../include/momentum_sdk/can/can_interface.hpp"
#include "../../include/momentum_sdk/can/can_exceptions.hpp"

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <cstring>
#include <poll.h>

namespace MomentumSDK {
    namespace CAN {

        CANInterface::CANInterface(std::string const& InterfaceName, 
                                    std::chrono::microseconds const& SendTimeout,
                                    std::chrono::microseconds const& ReceivedTimeout, 
                                    bool const IsSignalErrors)
            : InterfaceName_(InterfaceName), socket_(-1) {
            
            setSendTimeout(SendTimeout);
            setReceivedTimeout(ReceivedTimeout);
            setSignalErrors(IsSignalErrors);
        }

        CANInterface::~CANInterface() {
            closeSocketCAN();
        }

        bool CANInterface::open() {
            try {
                initSocketCAN(InterfaceName_);
                return true;
            } catch (const std::exception&) {
                return false;
            }
        }

        void CANInterface::close() noexcept {
            closeSocketCAN();
        }

        void CANInterface::setLoopback(bool const IsLoopback) {
            if (socket_ < 0) return;
            
            int loopback = IsLoopback ? 1 : 0;
            if (setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, 
                        &loopback, sizeof(loopback)) < 0) {
                throw MomentumSDK::CAN::CANException("Failed to set loopback mode");
            }
        }

        void CANInterface::setRecvFilter(std::vector<std::uint32_t> const& CAN_IDs, bool const IsInvert) {
            if (socket_ < 0 || CAN_IDs.empty()) return;
            
            std::vector<can_filter> filters;
            filters.reserve(CAN_IDs.size());
            
            for (auto id : CAN_IDs) {
                can_filter filter;
                filter.can_id = id;
                filter.can_mask = IsInvert ? 0 : CAN_SFF_MASK;
                filters.push_back(filter);
            }
            
            if (setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER,
                        filters.data(), filters.size() * sizeof(can_filter)) < 0) {
                throw MomentumSDK::CAN::CANException("Failed to set receive filter");
            }
        }

        void CANInterface::setSendTimeout(std::chrono::microseconds const& SendTimeout) {
            if (socket_ < 0) return;
            
            struct timeval tv;
            tv.tv_sec = std::chrono::duration_cast<std::chrono::seconds>(SendTimeout).count();
            tv.tv_usec = (SendTimeout.count() % 1000000);
            
            if (setsockopt(socket_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0) {
                throw MomentumSDK::CAN::CANException("Failed to set send timeout");
            }
        }

        void CANInterface::setReceivedTimeout(std::chrono::microseconds const& ReceivedTimeout) {
            if (socket_ < 0) return;
            
            struct timeval tv;
            tv.tv_sec = std::chrono::duration_cast<std::chrono::seconds>(ReceivedTimeout).count();
            tv.tv_usec = (ReceivedTimeout.count() % 1000000);
            
            if (setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
                throw MomentumSDK::CAN::CANException("Failed to set receive timeout");
            }
        }

        void CANInterface::setSignalErrors(bool const IsSignalErrors) {
            if (socket_ < 0) return;
            
            int err_mask = IsSignalErrors ? CAN_ERR_MASK : 0;
            if (setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER,
                        &err_mask, sizeof(err_mask)) < 0) {
                throw MomentumSDK::CAN::CANException("Failed to set error signaling");
            }
        }

        void CANInterface::writeFrame(CANFrame const& Frame) {
            if (socket_ < 0) {
                throw MomentumSDK::CAN::CANException("CAN interface not opened");
            }
            
            struct can_frame can_frame;
            can_frame.can_id = Frame.getID();
            can_frame.can_dlc = 8;
            
            auto frame_data = Frame.getFrame();
            std::memcpy(can_frame.data, frame_data.data(), 8);
            
            ssize_t bytes_sent = write(socket_, &can_frame, sizeof(can_frame));
            if (bytes_sent != sizeof(can_frame)) {
                throw MomentumSDK::CAN::CANException("Failed to send CAN frame");
            }
        }

        void CANInterface::writeFrame(std::uint32_t const CAN_ID, std::array<std::uint8_t,8> const& Frame) {
            CANFrame can_frame(CAN_ID, Frame);
            writeFrame(can_frame);
        }

        bool CANInterface::readFrame(CANFrame& Frame, std::uint32_t const Timeout_ms) {
            if (socket_ < 0) {
                return false;
            }
            
            // Use poll for timeout
            struct pollfd pfd;
            pfd.fd = socket_;
            pfd.events = POLLIN;
            
            int poll_result = poll(&pfd, 1, Timeout_ms);
            if (poll_result <= 0) {
                return false; // Timeout or error
            }
            
            struct can_frame can_frame;
            ssize_t bytes_read = read(socket_, &can_frame, sizeof(can_frame));
            
            if (bytes_read != sizeof(can_frame)) {
                return false;
            }
            
            // Convert to CANFrame format
            std::array<std::uint8_t, 8> data;
            std::memcpy(data.data(), can_frame.data, 8);
            
            // Create new frame and assign to the reference
            Frame = CANFrame(can_frame.can_id, data);
            
            return true;
        }

        void CANInterface::initSocketCAN(std::string const& InterfaceName) {
            // Create socket
            socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
            if (socket_ < 0) {
                throw MomentumSDK::CAN::CANException("Failed to create socket");
            }
            
            // Get interface index
            struct ifreq ifr;
            std::strncpy(ifr.ifr_name, InterfaceName.c_str(), IFNAMSIZ - 1);
            ifr.ifr_name[IFNAMSIZ - 1] = '\0';
            
            if (ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) {
                ::close(socket_);
                socket_ = -1;
                throw MomentumSDK::CAN::CANException("Failed to get interface index for " + InterfaceName);
            }
            
            // Bind socket
            struct sockaddr_can addr;
            std::memset(&addr, 0, sizeof(addr));
            addr.can_family = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;
            
            if (bind(socket_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
                ::close(socket_);
                socket_ = -1;
                throw MomentumSDK::CAN::CANException("Failed to bind socket to " + InterfaceName);
            }
        }

        void CANInterface::closeSocketCAN() noexcept {
            if (socket_ >= 0) {
                ::close(socket_);
                socket_ = -1;
            }
        }

    } 
} 
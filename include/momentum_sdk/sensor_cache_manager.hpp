/**
 * \file sensor_cache_manager.hpp
 * \brief
 *    Template-based cache manager
 * \details
 *    Provides unified cache update and retrieval for all sensor types.
 * \author
 *    ScalpelSpace (info@scalpelspace.com)
 * \version 1.0.0
 * \date 2025-07-26
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

#ifndef SENSOR_CACHE_MANAGER_HPP
#define SENSOR_CACHE_MANAGER_HPP

#include <mutex>
#include <functional>
#include <optional>
#include <chrono>

// Forward declarations for types defined in momentum_sdk.hpp
namespace MomentumSDK {
    enum class MomentumError;
    struct MomentumConfig;
}


namespace MomentumSDK {

    /**
     * \brief Template class to manage sensor data caching with unified logic
     * \tparam T The sensor data type (GpsPosition, GpsVelocity, ImuData, BarometricData)
     */
    template<typename T>
    class SensorCacheManager {
    public:
        /**
         * \brief Update cache with new sensor data and trigger callback
         * \param cache Reference to the cached data structure
         * \param new_data New sensor data to store
         * \param callback Optional callback function to notify on update
         * \param config Configuration settings for callbacks
         * \param mutex Mutex for thread-safe access
         */
        template<typename CacheType>
        static void updateCache(
            CacheType& cache,
            const T& new_data,
            const std::function<void(const T&)>& callback,
            const MomentumConfig& config,
            std::mutex& mutex) {
            
            std::lock_guard<std::mutex> lock(mutex);
            
            // Store new data with timestamp
            cache.value = new_data;
            cache.timestamp = std::chrono::system_clock::now();
            cache.status = MomentumSDK::MomentumError::None;
            
            // Call user callback 
            if (config.enable_callbacks && callback) {
                callback(new_data);
            }
        }

        /**
         * \brief Retrieve cached data if valid and fresh
         * \param cache Reference to the cached data structure
         * \param config Configuration settings for timeout validation
         * \param mutex Mutex for thread-safe access
         * \return Optional containing data if valid and fresh, nullopt otherwise
         */
        template<typename CacheType>
        static std::optional<T> getCachedData(
            const CacheType& cache,
            const MomentumConfig& config,
            std::mutex& mutex) {
            
            std::lock_guard<std::mutex> lock(mutex);
            
            // Return data only if valid and not too old
            if (!cache.isValid() || !cache.isFresh(config.data_timeout)) {
                return std::nullopt;
            }
            
            return cache.value;
        }
    };

}

#endif // SENSOR_CACHE_MANAGER_HPP

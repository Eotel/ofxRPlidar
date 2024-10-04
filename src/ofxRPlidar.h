/*
MIT LICENSE

Copyright 2017 nariakiiwatani

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
 * Modified by Eotel on 2024-09-28
 * - Added support for device types other than A2
 */

#pragma once

#include <string>
#include "ofThread.h"
#include "ofTypes.h"
#include "rplidar.h" //RPLiDAR standard sdk, all-in-one header
// __le defined in rplidar sdk but the same name is used in deque(libc++).
// it's seems not to be used in rplidar library at all so we undef it here.
#undef __le

#include "DoubleBuffer.h"
#include "ofSerial.h"

namespace rp::standalone::rplidar
{
    class RPlidarDriver;
}


namespace ofx::rplidar
{
    enum DeviceType
    {
        A1,
        A2M7,
        A2M8,
        A2M12,
        A3,
        S1,
        S2,
        S3,
        DEFAULT_TYPE = A1
    };

    namespace device
    {
        class GenericDevice final : ofThread
        {
        public:
            struct ScannedData
            {
                float angle;
                float distance;
                unsigned char quality;
                bool sync;
            };

            explicit GenericDevice(DeviceType type = ofx::rplidar::DEFAULT_TYPE);
            virtual ~GenericDevice();
            static std::vector<ofSerialDeviceInfo> getDeviceList();
            bool connect(const std::string& serial_path);
            bool reconnect();
            bool disconnect();
            bool isConnected() const;
            bool start(bool threaded = true);
            bool stop();
            void update();
            bool isFrameNew() const { return is_frame_new_; }
            static int getBaudRate(DeviceType type);

            std::vector<sl::LidarScanMode> scanModes;
            std::vector<ScannedData> scan(bool ascend = true);
            std::vector<ScannedData> getResult();
            std::string getSerialPath() const { return serial_path_; }
            std::string getSerialNumber() const;

        protected:
            std::string serial_path_;
            int baud_rate_;
            std::atomic<bool> has_new_frame_ = false;
            std::atomic<bool> is_frame_new_ = false;
            std::atomic<bool> is_scanning_ = false;
            std::chrono::steady_clock::time_point last_scan_time_;
            void threadedFunction() override;

            DoubleBuffer<std::vector<ScannedData>> result_;
            rp::standalone::rplidar::RPlidarDriver* driver_;
            rplidar_response_device_info_t device_info_{};
            rplidar_response_device_health_t health_info_{};
        };
    }
}

using ofxRPlidar = ofx::rplidar::device::GenericDevice;

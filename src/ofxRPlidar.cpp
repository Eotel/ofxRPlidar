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

#include "ofxRPlidar.h"
#include "ofLog.h"
#include "ofSerial.h"
#include "ofUtils.h"


using namespace rp::standalone::rplidar;
using namespace ofx::rplidar;
using namespace std;

namespace
{
    bool isDeviceRplidar(ofSerialDeviceInfo& device)
    {
#if defined(TARGET_WIN32)
        return ofIsStringInString(device.getDeviceName(), "Silicon Labs CP210x USB to UART Bridge");
#elif defined(TARGET_LINUX)
        return ofIsStringInString(device.getDeviceName(), "ttyUSB");
#else
        return ofIsStringInString(device.getDeviceName(), "tty.SLAB_USBtoUART");
#endif
    }
}

vector<ofSerialDeviceInfo> device::GenericDevice::getDeviceList()
{
    ofSerial serial;
    auto ret = serial.getDeviceList();
    ret.erase(remove_if(begin(ret), std::end(ret), [](ofSerialDeviceInfo& info)
    {
        return !isDeviceRplidar(info);
    }), std::end(ret));
    return ret;
}

device::GenericDevice::GenericDevice(const DeviceType type)
{
    driver_ = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);

    if (!driver_) ofLogError("RPLIDAR") << "insufficient memory, exit";

    baud_rate_ = getBaudRate(type);
}

device::GenericDevice::~GenericDevice()
{
    disconnect();
    if (driver_) RPlidarDriver::DisposeDriver(driver_);
}


bool device::GenericDevice::connect(const string& serial_path)
{
    serial_path_ = serial_path;
    if (IS_FAIL(driver_->connect(serial_path.c_str(), baud_rate_)))
    {
        ofLogError("RPLIDAR") << "Error, cannot bind to the specified serial port" << serial_path.c_str() << ".\n";
        return false;
    }

    // retrieving the device info
    ////////////////////////////////////////
    u_result op_result = driver_->getDeviceInfo(device_info_);

    if (IS_FAIL(op_result))
    {
        if (op_result == RESULT_OPERATION_TIMEOUT)
        {
            ofLogError("RPLIDAR") << "Error, operation time out.";
        }
        else
        {
            ofLogError("RPLIDAR") << "Error, unexpected error, code: " << op_result;
        }
        return false;
    }

    // print out the device serial number, firmware and hardware version number.
    string serial_number = getSerialNumber();
    ofLogVerbose("RPLIDAR") << "Serial Number: " << serial_number.c_str();

    ofLogVerbose("RPLIDAR") << "Version: " << RPLIDAR_SDK_VERSION;
    ofLogVerbose("RPLIDAR", "Firmware Ver: %d.%02d", device_info_.firmware_version >> 8,
                 device_info_.firmware_version & 0xFF);
    ofLogVerbose("RPLIDAR") << "Hardware Rev: " << ofToString(static_cast<int>(device_info_.hardware_version));


    // check the device health
    ////////////////////////////////////////
    op_result = driver_->getHealth(health_info_);
    if (IS_OK(op_result))
    {
        // the macro IS_OK is the preferred way to judge whether the operation is succeeded.
        const char* health_status = [](const _u8 status)
        {
            switch (status)
            {
            case RPLIDAR_STATUS_OK:
                return "OK.";
            case RPLIDAR_STATUS_WARNING:
                return "Warning.";
            case RPLIDAR_STATUS_ERROR:
                return "Error.";
            default:
                return "Unknown.";
            }
        }(health_info_.status);

        ofLogVerbose("RPLIDAR", "health status: %s", health_status);
        ofLogVerbose("RPLIDAR") << " (error code: )" << ofToString(health_info_.error_code);
    }
    else
    {
        ofLogError("RPLIDAR", "Error, cannot retrieve the lidar health code: %x", op_result);
        return false;
    }

    if (health_info_.status == RPLIDAR_STATUS_ERROR)
    {
        ofLogError("RPLIDAR") << "Error, rplidar internal error detected. Please reboot the device to retry.";
        // enable the following code if you want rplidar to be rebooted by software
        // drv->reset();
        return false;
    }

    return true;
}

bool device::GenericDevice::reconnect()
{
    return connect(serial_path_);
}

bool device::GenericDevice::disconnect()
{
    if (isConnected())
    {
        stop();
        driver_->disconnect();
        return true;
    }
    return false;
}

bool device::GenericDevice::isConnected() const
{
    return driver_ && driver_->isConnected();
}

bool device::GenericDevice::start(const bool threaded)
{
    if (isConnected()
        && !IS_FAIL(driver_->startMotor())
        && !IS_FAIL(driver_->startScan(true, true)))
    {
        if (threaded)
        {
            startThread();
        }
        return true;
    }
    return false;
}

bool device::GenericDevice::stop()
{
    if (isThreadRunning())
    {
        stopThread();
        waitForThread();
    }
    if (isConnected()
        && !IS_FAIL(driver_->stop())
        && !IS_FAIL(driver_->stopMotor()))
    {
        return true;
    }
    return false;
}

void device::GenericDevice::threadedFunction()
{
    while (isThreadRunning())
    {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_scan_time_).count() >= 66)
        {
            // ~15Hz
            result_.back() = scan();
            result_.swap();
            has_new_frame_ = true;
        }
        ofSleepMillis(1);
        if (!isConnected())
        {
            stopThread();
        }
    }
}

void device::GenericDevice::update()
{
    if (isThreadRunning())
    {
        is_frame_new_ = has_new_frame_.exchange(false);
    }
    else
    {
        result_.back() = scan();
        result_.swap();
        is_frame_new_ = true;
    }
}

int device::GenericDevice::getBaudRate(const DeviceType type)
{
    switch (type)
    {
    case A1:
    case A2M8:
        return 115200;
    case A2M7:
    case A2M12:
    case A3:
    case S1:
        return 256000;
    case S2:
    case S3:
        return 1000000;
    default:
        return 115200; // Default baud rate
    }
}


std::vector<device::GenericDevice::ScannedData> device::GenericDevice::getResult()
{
    if (isThreadRunning())
    {
        return result_.front();
    }
    return scan();
}

string device::GenericDevice::getSerialNumber() const
{
    string ret;
    for (unsigned char pos : device_info_.serialnum)
    {
        ret += ofToHex(pos);
    }
    return ret;
}

std::vector<device::GenericDevice::ScannedData> device::GenericDevice::scan(const bool ascend)
{
    std::vector<ScannedData> ret;
    is_scanning_ = true;
    auto start = std::chrono::steady_clock::now();

    rplidar_response_measurement_node_hq_t nodes[8192];
    size_t count = sizeof(nodes) / sizeof(rplidar_response_measurement_node_hq_t);

    while (is_scanning_ && std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count() < 1000)
    {
        u_result ans = driver_->grabScanDataHq(nodes, count);
        if (IS_OK(ans))
        {
            if (ascend) driver_->ascendScanData(nodes, count);

            ret.resize(count);
            for (int i = 0; i < count; ++i)
            {
                auto& [angle, distance, quality, sync] = ret[i];
                sync = nodes[i].flag;
                angle = static_cast<float>(nodes[i].angle_z_q14) * 90.f / (1 << 14);
                distance = static_cast<float>(nodes[i].dist_mm_q2) / (1 << 2);
                quality = nodes[i].quality;
            }
            is_scanning_ = false;
            last_scan_time_ = std::chrono::steady_clock::now();
            return ret;
        }
        else if (ans != RESULT_OPERATION_TIMEOUT)
        {
            ofLogWarning("RPLIDAR") << "Error occurred during scan, code: " << ans;
            break;
        }
    }

    is_scanning_ = false;
    if (ret.empty())
    {
        ofLogError("RPLIDAR") << "Failed to get scan data within timeout period";
    }
    return ret;
}

#include "haptics/HaplyInverse3Device.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>
#include <unistd.h>

#ifndef NO_HAPLY_HARDWARE_API
#include "HardwareAPI.h"
#endif

// Inverse3 continuous force limit per axis [N]
static constexpr float kMaxForcePerAxis = 3.3f;

// --------------------------------------------------------------------------
// Auto-detect serial port
// --------------------------------------------------------------------------
std::string HaplyInverse3Device::_autoDetectPort()
{
#ifndef NO_HAPLY_HARDWARE_API
    std::vector<std::string> ports = Haply::HardwareAPI::Devices::DeviceDetection::DetectInverse3s();
    if (!ports.empty())
    {
        std::cout << "[HaplyInverse3] Found device on " << ports[0] << std::endl;
        return ports[0];
    }
#endif
    return "";
}

// --------------------------------------------------------------------------
// Constructor
// --------------------------------------------------------------------------
HaplyInverse3Device::HaplyInverse3Device(const std::string& serial_port)
{
#ifndef NO_HAPLY_HARDWARE_API
    std::string port = serial_port;
    if (port.empty())
    {
        // Skip DetectInverse3s() to avoid double-wakeup — just scan ports directly
        const char* candidates[] = {
            "/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"
        };
        for (const char* p : candidates)
        {
            if (access(p, R_OK | W_OK) == 0)
            {
                port = p;
                std::cout << "[HaplyInverse3] Found serial port: " << port << std::endl;
                break;
            }
        }
    }
    if (port.empty())
    {
        std::cerr << "[HaplyInverse3] Inverse3 NOT found, falling back to mouse/keyboard" << std::endl;
        _connected = false;
        return;
    }

    // Retry initialization — the Inverse3 wakeup over Docker serial is flaky
    // and sometimes returns device_id 0 or zero position.
    constexpr int kMaxRetries = 5;
    for (int attempt = 1; attempt <= kMaxRetries; ++attempt)
    {
        try
        {
            auto* stream = new Haply::HardwareAPI::IO::SerialStream(port.c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            auto* device = new Haply::HardwareAPI::Devices::Inverse3(stream);
            auto info = device->DeviceWakeup();

            std::cout << "[HaplyInverse3] Attempt " << attempt
                      << ": device ID = " << info.device_id << std::endl;

            // Verify with a test read — send zero force and check position
            Haply::HardwareAPI::Devices::Inverse3::EndEffectorForceRequest req{};
            auto resp = device->EndEffectorForce(req);
            bool valid = (resp.position[0] != 0.0f || resp.position[1] != 0.0f
                          || resp.position[2] != 0.0f);

            if (valid)
            {
                std::cout << "[HaplyInverse3] Device woken up on " << port
                          << "  (device ID: " << info.device_id << ")" << std::endl;
                std::cout << "[HaplyInverse3] Initial position: ("
                          << resp.position[0] << ", " << resp.position[1] << ", "
                          << resp.position[2] << ")" << std::endl;
                _stream_handle = static_cast<void*>(stream);
                _device_handle = static_cast<void*>(device);
                _initial_position[0] = static_cast<Real>(resp.position[0]);
                _initial_position[1] = static_cast<Real>(resp.position[1]);
                _initial_position[2] = static_cast<Real>(resp.position[2]);
                _position = _initial_position;
                _connected = true;
                break;
            }

            // Invalid — clean up and retry
            std::cerr << "[HaplyInverse3] Attempt " << attempt
                      << " got zero position, retrying..." << std::endl;
            delete device;
            delete stream;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        catch (const std::exception& e)
        {
            std::cerr << "[HaplyInverse3] Attempt " << attempt
                      << " failed: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }

    if (!_connected)
    {
        std::cerr << "[HaplyInverse3] Failed to initialize device after "
                  << kMaxRetries << " attempts" << std::endl;
        return;
    }
#else
    (void)serial_port;
    std::cerr << "[HaplyInverse3] Compiled without Haply HardwareAPI (NO_HAPLY_HARDWARE_API)" << std::endl;
    _connected = false;
#endif
}

// --------------------------------------------------------------------------
// Destructor
// --------------------------------------------------------------------------
HaplyInverse3Device::~HaplyInverse3Device()
{
#ifndef NO_HAPLY_HARDWARE_API
    if (_device_handle)
    {
        // Send a final zero force before shutting down
        auto* device = static_cast<Haply::HardwareAPI::Devices::Inverse3*>(_device_handle);
        try
        {
            Haply::HardwareAPI::Devices::Inverse3::EndEffectorForceRequest req{};
            device->EndEffectorForce(req);
        }
        catch (...) {}
        delete device;
        _device_handle = nullptr;
    }
    if (_stream_handle)
    {
        auto* stream = static_cast<Haply::HardwareAPI::IO::SerialStream*>(_stream_handle);
        delete stream;
        _stream_handle = nullptr;
    }
#endif
}

// --------------------------------------------------------------------------
// Synchronous poll — call once per sim time step
// --------------------------------------------------------------------------
bool HaplyInverse3Device::poll()
{
    if (!_connected) return false;

#ifndef NO_HAPLY_HARDWARE_API
    auto* device = static_cast<Haply::HardwareAPI::Devices::Inverse3*>(_device_handle);

    Haply::HardwareAPI::Devices::Inverse3::EndEffectorForceRequest req{};
    req.force[0] = static_cast<float>(std::clamp(_commanded_force[0],
                        static_cast<Real>(-kMaxForcePerAxis),
                        static_cast<Real>(kMaxForcePerAxis)));
    req.force[1] = static_cast<float>(std::clamp(_commanded_force[1],
                        static_cast<Real>(-kMaxForcePerAxis),
                        static_cast<Real>(kMaxForcePerAxis)));
    req.force[2] = static_cast<float>(std::clamp(_commanded_force[2],
                        static_cast<Real>(-kMaxForcePerAxis),
                        static_cast<Real>(kMaxForcePerAxis)));

    try
    {
        auto resp = device->EndEffectorForce(req);

        bool valid = (resp.position[0] != 0.0f || resp.position[1] != 0.0f
                      || resp.position[2] != 0.0f);
        if (valid)
        {
            _position[0] = static_cast<Real>(resp.position[0]);
            _position[1] = static_cast<Real>(resp.position[1]);
            _position[2] = static_cast<Real>(resp.position[2]);
            _velocity[0] = static_cast<Real>(resp.velocity[0]);
            _velocity[1] = static_cast<Real>(resp.velocity[1]);
            _velocity[2] = static_cast<Real>(resp.velocity[2]);
        }

        // Log every ~1 second (assuming ~30 fps ≈ every 30 polls)
        if (++_poll_count % 30 == 0)
        {
            std::cout << "[HaplyInverse3] pos=(" << resp.position[0] << ", "
                      << resp.position[1] << ", " << resp.position[2] << ")"
                      << (valid ? "" : " [STALE]") << std::endl;
        }

        return valid;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[HaplyInverse3] Communication error: " << e.what() << std::endl;
        return false;
    }
#else
    return false;
#endif
}

// --------------------------------------------------------------------------
// Set force command
// --------------------------------------------------------------------------
void HaplyInverse3Device::setForce(const Vec3r& force)
{
    if (!_connected) return;
    _commanded_force = force;
}

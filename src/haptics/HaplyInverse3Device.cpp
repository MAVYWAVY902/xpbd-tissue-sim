#include "haptics/HaplyInverse3Device.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
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
    // Use the built-in detection API
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

    try
    {
        auto* stream = new Haply::HardwareAPI::IO::SerialStream(port.c_str());
        // Brief pause to let serial port settle
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        auto* device = new Haply::HardwareAPI::Devices::Inverse3(stream);
        Haply::HardwareAPI::Devices::Inverse3::DeviceInfoResponse info = device->DeviceWakeup();
        std::cout << "[HaplyInverse3] Device woken up on " << port
                  << "  (device ID: " << info.device_id << ")" << std::endl;

        // Read initial position to verify communication works
        auto state = device->GetEndEffectorPosition();
        std::cout << "[HaplyInverse3] Initial position: ("
                  << state.position[0] << ", " << state.position[1] << ", "
                  << state.position[2] << ")" << std::endl;
        _stream_handle = static_cast<void*>(stream);
        _device_handle = static_cast<void*>(device);
        _connected = true;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[HaplyInverse3] Failed to initialize device on " << port
                  << ": " << e.what() << std::endl;
        _connected = false;
        return;
    }

    // Spawn the 1 kHz communication thread
    _running = true;
    _thread = std::thread(&HaplyInverse3Device::_runLoop, this);
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
    _running = false;
    if (_thread.joinable())
        _thread.join();

#ifndef NO_HAPLY_HARDWARE_API
    if (_device_handle)
    {
        // Send a final zero force before shutting down
        auto* device = static_cast<Haply::HardwareAPI::Devices::Inverse3*>(_device_handle);
        try
        {
            Haply::HardwareAPI::Devices::Inverse3::EndEffectorForceRequest req;
            req.force[0] = 0.0f; req.force[1] = 0.0f; req.force[2] = 0.0f;
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
// 1 kHz communication loop
// --------------------------------------------------------------------------
void HaplyInverse3Device::_runLoop()
{
#ifndef NO_HAPLY_HARDWARE_API
    auto* device = static_cast<Haply::HardwareAPI::Devices::Inverse3*>(_device_handle);
    int loop_count = 0;

    while (_running)
    {
        // 1. Read the latest force command from the sim thread
        Haply::HardwareAPI::Devices::Inverse3::EndEffectorForceRequest req;
        {
            std::lock_guard<std::mutex> guard(_input_mtx);
            req.force[0] = static_cast<float>(_commanded_force[0]);
            req.force[1] = static_cast<float>(_commanded_force[1]);
            req.force[2] = static_cast<float>(_commanded_force[2]);
        }

        // Clamp each axis to the hardware limit
        for (int i = 0; i < 3; ++i)
            req.force[i] = std::clamp(req.force[i], -kMaxForcePerAxis, kMaxForcePerAxis);

        // 2. Exchange force for position+velocity (blocks ~1 ms)
        try
        {
            auto resp = device->EndEffectorForce(req);

            // 3. Write new device state (device→sim)
            {
                std::lock_guard<std::mutex> guard(_state_mtx);
                _device_state.position[0] = static_cast<Real>(resp.position[0]);
                _device_state.position[1] = static_cast<Real>(resp.position[1]);
                _device_state.position[2] = static_cast<Real>(resp.position[2]);
                _device_state.velocity[0] = static_cast<Real>(resp.velocity[0]);
                _device_state.velocity[1] = static_cast<Real>(resp.velocity[1]);
                _device_state.velocity[2] = static_cast<Real>(resp.velocity[2]);
                _copied_state.stale = true;
            }

            // Log position every ~1 second (every 1000 loops at 1kHz)
            if (++loop_count % 1000 == 0)
            {
                std::cout << "[HaplyInverse3] pos=(" << resp.position[0] << ", "
                          << resp.position[1] << ", " << resp.position[2] << ")" << std::endl;
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "[HaplyInverse3] Communication error: " << e.what() << std::endl;
            continue;
        }
    }
#endif
}

// --------------------------------------------------------------------------
// Thread-safe accessors (sim thread)
// --------------------------------------------------------------------------
Vec3r HaplyInverse3Device::position()
{
    if (!_connected) return Vec3r::Zero();

    if (_copied_state.stale)
    {
        std::lock_guard<std::mutex> guard(_state_mtx);
        _copied_state.position = _device_state.position;
        _copied_state.velocity = _device_state.velocity;
        _copied_state.stale = false;
    }
    return _copied_state.position;
}

Vec3r HaplyInverse3Device::velocity()
{
    if (!_connected) return Vec3r::Zero();

    if (_copied_state.stale)
    {
        std::lock_guard<std::mutex> guard(_state_mtx);
        _copied_state.position = _device_state.position;
        _copied_state.velocity = _device_state.velocity;
        _copied_state.stale = false;
    }
    return _copied_state.velocity;
}

void HaplyInverse3Device::setForce(const Vec3r& force)
{
    if (!_connected) return;

    std::lock_guard<std::mutex> guard(_input_mtx);
    _commanded_force = force;
}

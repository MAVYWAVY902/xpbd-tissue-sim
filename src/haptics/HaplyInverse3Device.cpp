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

#ifndef NO_HAPLY_HARDWARE_API
// --------------------------------------------------------------------------
// QuillHandle: subclass of Handle that captures orientation via callbacks.
// The wired VerseGrip Quill uses the traditional Handle protocol
// (SendDeviceWakeup → RequestStatus → Receive) rather than
// the wireless-dongle GetVersegripStatus() API.
// --------------------------------------------------------------------------
class QuillHandle : public Haply::HardwareAPI::Devices::Handle
{
public:
    using Handle::Handle;  // inherit constructor

    // Last received quaternion in WXYZ order (as per SDK docs)
    float last_quat[4] = {0, 0, 0, 1};  // identity
    uint16_t last_device_id = 0;
    bool has_data = false;

protected:
    void OnReceiveHandleStatusMessage(HandleStatusResponse& response) override
    {
        last_device_id = response.device_id;
        // HandleStatusResponse::quaternion is WXYZ order per SDK docs
        std::memcpy(last_quat, response.quaternion, sizeof(float) * 4);
        has_data = true;
    }
};
#endif

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
    // ---- 1. Connect to Inverse3 ----
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

    // ---- 2. Try to connect VerseGrip ----
    // Skip DetectHandles() — it probes ALL ports including the Inverse3,
    // which corrupts the Inverse3's serial protocol state.
    // Instead, directly scan for the VerseGrip on remaining ports.
    // The VerseGrip Quill uses CP210x (shows up as /dev/ttyUSB*)
    std::string handle_port;
    {
        const char* candidates[] = {
            "/dev/ttyUSB0", "/dev/ttyUSB1",
            "/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"
        };
        for (const char* p : candidates)
        {
            if (std::string(p) == port) continue;  // skip Inverse3 port
            if (access(p, R_OK | W_OK) == 0)
            {
                handle_port = p;
                std::cout << "[VerseGrip] Found port: " << handle_port << std::endl;
                break;
            }
        }
    }

    if (!handle_port.empty())
    {
        try
        {
            auto* h_stream = new Haply::HardwareAPI::IO::SerialStream(handle_port.c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            auto* handle = new QuillHandle(h_stream);

            // REQUIRED: wake up the Handle before any other commands
            std::cout << "[VerseGrip] Sending wakeup..." << std::endl;
            handle->SendDeviceWakeup();
            int wakeup_rc = handle->Receive();
            std::cout << "[VerseGrip] Wakeup Receive() returned: " << wakeup_rc
                      << "  device_id=" << handle->last_device_id << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            // Probe with the traditional RequestStatus → Receive flow
            bool grip_valid = false;
            for (int g = 0; g < 5; ++g)
            {
                handle->has_data = false;
                handle->RequestStatus();
                int rc = handle->Receive();

                float qw = handle->last_quat[0];  // WXYZ order
                float qx = handle->last_quat[1];
                float qy = handle->last_quat[2];
                float qz = handle->last_quat[3];
                float qnorm = qw*qw + qx*qx + qy*qy + qz*qz;

                std::cout << "[VerseGrip] Probe " << (g + 1)
                          << ": rc=" << rc
                          << "  has_data=" << handle->has_data
                          << "  quat_wxyz=(" << qw << ", " << qx
                          << ", " << qy << ", " << qz << ")"
                          << "  qnorm=" << qnorm << std::endl;

                if (handle->has_data && qnorm > 0.5f)
                {
                    _handle_stream_handle = static_cast<void*>(h_stream);
                    _handle_device_handle = static_cast<void*>(handle);
                    // Convert WXYZ → XYZW (codebase convention: scalar-last)
                    _orientation[0] = static_cast<Real>(qx);
                    _orientation[1] = static_cast<Real>(qy);
                    _orientation[2] = static_cast<Real>(qz);
                    _orientation[3] = static_cast<Real>(qw);
                    _versegrip_connected = true;
                    grip_valid = true;
                    std::cout << "[VerseGrip] Connected! Quill wired handle on "
                              << handle_port << std::endl;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            // If traditional API failed, also try GetVersegripStatus as fallback
            if (!grip_valid)
            {
                std::cout << "[VerseGrip] Traditional API didn't return data, "
                          << "trying GetVersegripStatus (wireless API)..." << std::endl;
                for (int g = 0; g < 3; ++g)
                {
                    auto status = handle->GetVersegripStatus();
                    float qnorm = status.q.x*status.q.x + status.q.y*status.q.y
                                + status.q.z*status.q.z + status.q.w*status.q.w;

                    std::cout << "[VerseGrip] Wireless probe " << (g + 1)
                              << ": err=" << (int)status.error_flag
                              << "  q=(" << status.q.x << ", " << status.q.y
                              << ", " << status.q.z << ", " << status.q.w << ")"
                              << "  qnorm=" << qnorm << std::endl;

                    if (qnorm > 0.5f && status.error_flag == 0)
                    {
                        _handle_stream_handle = static_cast<void*>(h_stream);
                        _handle_device_handle = static_cast<void*>(handle);
                        _orientation[0] = static_cast<Real>(status.q.x);
                        _orientation[1] = static_cast<Real>(status.q.y);
                        _orientation[2] = static_cast<Real>(status.q.z);
                        _orientation[3] = static_cast<Real>(status.q.w);
                        _versegrip_connected = true;
                        grip_valid = true;
                        _use_versegrip_api = true;
                        std::cout << "[VerseGrip] Connected via wireless API!" << std::endl;
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
            }

            if (!grip_valid)
            {
                std::cerr << "[VerseGrip] Device found on " << handle_port
                          << " but no valid orientation data received.\n"
                          << "            Is the VerseGrip powered on?" << std::endl;
                delete handle;
                delete h_stream;
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "[VerseGrip] Failed to connect on " << handle_port
                      << ": " << e.what() << std::endl;
        }
    }

    if (!_versegrip_connected)
    {
        std::cout << "[VerseGrip] Not detected. Using keyboard rotation (Q/E/R/F/Z/X)." << std::endl;
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
    if (_handle_device_handle)
    {
        auto* handle = static_cast<QuillHandle*>(_handle_device_handle);
        delete handle;
        _handle_device_handle = nullptr;
    }
    if (_handle_stream_handle)
    {
        auto* stream = static_cast<Haply::HardwareAPI::IO::SerialStream*>(_handle_stream_handle);
        delete stream;
        _handle_stream_handle = nullptr;
    }
#endif
}

// --------------------------------------------------------------------------
// Synchronous poll — call once per sim time step
// --------------------------------------------------------------------------
bool HaplyInverse3Device::poll()
{
    if (!_connected) return false;

    bool result = false;

#ifndef NO_HAPLY_HARDWARE_API
    // ---- Poll Inverse3 (position + velocity) ----
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

        // Log every ~1 second (assuming ~30 fps)
        if (++_poll_count % 30 == 0)
        {
            std::cout << "[HaplyInverse3] pos=(" << resp.position[0] << ", "
                      << resp.position[1] << ", " << resp.position[2] << ")"
                      << (valid ? "" : " [STALE]") << std::endl;
        }

        result = valid;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[HaplyInverse3] Communication error: " << e.what() << std::endl;
    }

    // ---- Poll VerseGrip (orientation) ----
    if (_versegrip_connected && _handle_device_handle)
    {
        auto* handle = static_cast<QuillHandle*>(_handle_device_handle);
        try
        {
            if (_use_versegrip_api)
            {
                // Wireless dongle API
                auto status = handle->GetVersegripStatus();
                if (status.error_flag == 0)
                {
                    float qnorm = status.q.x*status.q.x + status.q.y*status.q.y
                                + status.q.z*status.q.z + status.q.w*status.q.w;
                    if (qnorm > 0.5f)
                    {
                        _orientation[0] = static_cast<Real>(status.q.x);
                        _orientation[1] = static_cast<Real>(status.q.y);
                        _orientation[2] = static_cast<Real>(status.q.z);
                        _orientation[3] = static_cast<Real>(status.q.w);
                    }
                }
            }
            else
            {
                // Traditional wired Quill API
                handle->has_data = false;
                handle->RequestStatus();
                handle->Receive();

                if (handle->has_data)
                {
                    float qw = handle->last_quat[0];  // WXYZ order
                    float qx = handle->last_quat[1];
                    float qy = handle->last_quat[2];
                    float qz = handle->last_quat[3];
                    float qnorm = qw*qw + qx*qx + qy*qy + qz*qz;
                    if (qnorm > 0.5f)
                    {
                        // Convert WXYZ → XYZW (codebase: scalar-last)
                        _orientation[0] = static_cast<Real>(qx);
                        _orientation[1] = static_cast<Real>(qy);
                        _orientation[2] = static_cast<Real>(qz);
                        _orientation[3] = static_cast<Real>(qw);
                    }
                }
            }

            if (_poll_count % 30 == 0)
            {
                std::cout << "[VerseGrip] orient=(" << _orientation[0] << ", "
                          << _orientation[1] << ", " << _orientation[2] << ", "
                          << _orientation[3] << ")" << std::endl;
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "[VerseGrip] Communication error: " << e.what() << std::endl;
        }
    }
#endif

    return result;
}

// --------------------------------------------------------------------------
// Set force command
// --------------------------------------------------------------------------
void HaplyInverse3Device::setForce(const Vec3r& force)
{
    if (!_connected) return;
    _commanded_force = force;
}

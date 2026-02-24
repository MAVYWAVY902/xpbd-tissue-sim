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

    // Open the serial stream ONCE. Each new SerialStream triggers a DTR reset
    // that reboots the Inverse3 firmware, so we must NOT re-open on retry.
    Haply::HardwareAPI::IO::SerialStream* stream = nullptr;
    Haply::HardwareAPI::Devices::Inverse3* device = nullptr;
    try
    {
        stream = new Haply::HardwareAPI::IO::SerialStream(port.c_str());
        // Wait for device to recover from DTR-triggered reboot
        std::cout << "[HaplyInverse3] Waiting 3 seconds for device boot after port open..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(3));
        // Use default timeout (5s) — same as working HapticConnectionTest.
        // The background thread absorbs the blocking, so simulation won't lag.
        device = new Haply::HardwareAPI::Devices::Inverse3(stream);
    }
    catch (const std::exception& e)
    {
        std::cerr << "[HaplyInverse3] Failed to open " << port << ": " << e.what() << std::endl;
        delete stream;
        _connected = false;
        return;
    }

    // DeviceWakeup — retry up to 3 times (100ms timeout per attempt is fine;
    // in tests the wakeup responds quickly when the device is ready).
    bool wakeup_ok = false;
    for (int w = 0; w < 3; ++w)
    {
        try
        {
            auto info = device->DeviceWakeup();
            std::cout << "[HaplyInverse3] DeviceWakeup (attempt " << (w + 1)
                      << "): device ID = " << info.device_id << std::endl;
            wakeup_ok = true;
            break;
        }
        catch (const std::exception& e)
        {
            std::cerr << "[HaplyInverse3] DeviceWakeup attempt " << (w + 1)
                      << " failed: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    if (!wakeup_ok)
    {
        std::cerr << "[HaplyInverse3] DeviceWakeup failed after 3 attempts" << std::endl;
        delete device;
        delete stream;
        _connected = false;
        return;
    }

    // Wait after wakeup before sending force commands
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Pump EndEffectorForce repeatedly — device may need several cycles
    // before forward kinematics returns valid positions.
    constexpr int kMaxPumpCycles = 50;
    std::cout << "[HaplyInverse3] Pumping EndEffectorForce (up to " << kMaxPumpCycles
              << " cycles)..." << std::endl;
    for (int i = 0; i < kMaxPumpCycles; ++i)
    {
        try
        {
            Haply::HardwareAPI::Devices::Inverse3::EndEffectorForceRequest req{};
            auto resp = device->EndEffectorForce(req);
            bool valid = (resp.position[0] != 0.0f || resp.position[1] != 0.0f
                          || resp.position[2] != 0.0f);

            if (i % 10 == 0 || valid)
            {
                std::cout << "[HaplyInverse3] Cycle " << (i + 1)
                          << ": pos=(" << resp.position[0] << ", "
                          << resp.position[1] << ", " << resp.position[2] << ")"
                          << (valid ? " VALID!" : " (zero)") << std::endl;
            }

            if (valid)
            {
                std::cout << "[HaplyInverse3] Connected on " << port
                          << " after " << (i + 1) << " cycles" << std::endl;
                _stream_handle = static_cast<void*>(stream);
                _device_handle = static_cast<void*>(device);
                _initial_position[0] = static_cast<Real>(resp.position[0]);
                _initial_position[1] = static_cast<Real>(resp.position[1]);
                _initial_position[2] = static_cast<Real>(resp.position[2]);
                _position = _initial_position;
                _connected = true;
                break;
            }
        }
        catch (const std::exception& e)
        {
            if (i % 10 == 0)
            {
                std::cerr << "[HaplyInverse3] Cycle " << (i + 1)
                          << " error: " << e.what() << std::endl;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (!_connected)
    {
        std::cerr << "[HaplyInverse3] Failed after " << kMaxPumpCycles
                  << " cycles — no valid position received" << std::endl;
        delete device;
        delete stream;
    }

    if (!_connected)
    {
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
            auto* handle = new QuillHandle(h_stream, 2.0f);  // 2s timeout (bg thread)

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

    // ---- 3. Start background polling threads ----
    _poll_running = true;
    _inverse3_thread = std::thread(&HaplyInverse3Device::_inverse3ThreadFunc, this);
    std::cout << "[HaplyInverse3] Inverse3 polling thread started." << std::endl;

    if (_versegrip_connected)
    {
        _versegrip_thread = std::thread(&HaplyInverse3Device::_versegripThreadFunc, this);
        std::cout << "[HaplyInverse3] VerseGrip polling thread started." << std::endl;
    }

#else
    (void)serial_port;
    std::cerr << "[HaplyInverse3] Compiled without Haply HardwareAPI (NO_HAPLY_HARDWARE_API)" << std::endl;
    _connected = false;
#endif
}

// --------------------------------------------------------------------------
// Destructor — stop the polling thread, then clean up devices
// --------------------------------------------------------------------------
HaplyInverse3Device::~HaplyInverse3Device()
{
    // Stop all background polling threads
    _poll_running = false;
    if (_inverse3_thread.joinable())
        _inverse3_thread.join();
    if (_versegrip_thread.joinable())
        _versegrip_thread.join();

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
// Thread-safe getters
// --------------------------------------------------------------------------
Vec3r HaplyInverse3Device::position() const
{
    std::lock_guard<std::mutex> lock(_pos_mutex);
    return _position;
}

Vec3r HaplyInverse3Device::velocity() const
{
    std::lock_guard<std::mutex> lock(_pos_mutex);
    return _velocity;
}

Vec4r HaplyInverse3Device::orientation() const
{
    std::lock_guard<std::mutex> lock(_orient_mutex);
    return _orientation;
}

// --------------------------------------------------------------------------
// Inverse3 polling thread — tight loop, just like HapticConnectionTest
// --------------------------------------------------------------------------
void HaplyInverse3Device::_inverse3ThreadFunc()
{
#ifndef NO_HAPLY_HARDWARE_API
    auto* device = static_cast<Haply::HardwareAPI::Devices::Inverse3*>(_device_handle);
    int count = 0;
    int errors = 0;

    while (_poll_running)
    {
        try
        {
            Haply::HardwareAPI::Devices::Inverse3::EndEffectorForceRequest req{};
            auto resp = device->EndEffectorForce(req);

            bool valid = (resp.position[0] != 0.0f || resp.position[1] != 0.0f
                          || resp.position[2] != 0.0f);
            if (valid)
            {
                std::lock_guard<std::mutex> lock(_pos_mutex);
                _position[0] = static_cast<Real>(resp.position[0]);
                _position[1] = static_cast<Real>(resp.position[1]);
                _position[2] = static_cast<Real>(resp.position[2]);
                _velocity[0] = static_cast<Real>(resp.velocity[0]);
                _velocity[1] = static_cast<Real>(resp.velocity[1]);
                _velocity[2] = static_cast<Real>(resp.velocity[2]);
            }

            ++count;
            if (count % 5000 == 0)
            {
                std::cout << "[Inverse3 thread] " << count << " polls, "
                          << errors << " errors, pos=("
                          << resp.position[0] << ", " << resp.position[1]
                          << ", " << resp.position[2] << ")" << std::endl;
            }
        }
        catch (const std::exception& e)
        {
            ++errors;
            if (errors <= 3)
                std::cerr << "[Inverse3 thread] Error: " << e.what() << std::endl;
        }
        // No sleep — tight loop, same as working HapticConnectionTest
    }

    std::cout << "[Inverse3 thread] Stopped. " << count << " polls, "
              << errors << " errors." << std::endl;
#endif
}

// --------------------------------------------------------------------------
// VerseGrip polling thread — runs independently on its own serial port
// --------------------------------------------------------------------------
void HaplyInverse3Device::_versegripThreadFunc()
{
#ifndef NO_HAPLY_HARDWARE_API
    auto* handle = static_cast<QuillHandle*>(_handle_device_handle);
    int count = 0;
    int errors = 0;

    while (_poll_running)
    {
        try
        {
            if (_use_versegrip_api)
            {
                auto status = handle->GetVersegripStatus();
                if (status.error_flag == 0)
                {
                    float qnorm = status.q.x*status.q.x + status.q.y*status.q.y
                                + status.q.z*status.q.z + status.q.w*status.q.w;
                    if (qnorm > 0.5f)
                    {
                        std::lock_guard<std::mutex> lock(_orient_mutex);
                        _orientation[0] = static_cast<Real>(status.q.x);
                        _orientation[1] = static_cast<Real>(status.q.y);
                        _orientation[2] = static_cast<Real>(status.q.z);
                        _orientation[3] = static_cast<Real>(status.q.w);
                    }
                }
            }
            else
            {
                handle->has_data = false;
                handle->RequestStatus();
                handle->Receive();

                if (handle->has_data)
                {
                    float qw = handle->last_quat[0];
                    float qx = handle->last_quat[1];
                    float qy = handle->last_quat[2];
                    float qz = handle->last_quat[3];
                    float qnorm = qw*qw + qx*qx + qy*qy + qz*qz;
                    if (qnorm > 0.5f)
                    {
                        std::lock_guard<std::mutex> lock(_orient_mutex);
                        _orientation[0] = static_cast<Real>(qx);
                        _orientation[1] = static_cast<Real>(qy);
                        _orientation[2] = static_cast<Real>(qz);
                        _orientation[3] = static_cast<Real>(qw);
                    }
                }
            }
            ++count;
            if (count % 500 == 0)
            {
                std::lock_guard<std::mutex> lock(_orient_mutex);
                std::cout << "[VerseGrip thread] " << count << " polls, "
                          << errors << " errors, orient=("
                          << _orientation[0] << ", " << _orientation[1]
                          << ", " << _orientation[2] << ", " << _orientation[3]
                          << ")" << std::endl;
            }
        }
        catch (const std::exception& e)
        {
            ++errors;
            if (errors <= 3)
                std::cerr << "[VerseGrip thread] Error: " << e.what() << std::endl;
        }
        // Small sleep — orientation doesn't need kHz updates
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cout << "[VerseGrip thread] Stopped. " << count << " polls, "
              << errors << " errors." << std::endl;
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

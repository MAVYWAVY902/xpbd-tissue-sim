#pragma once

#include "common/types.hpp"
#include <mutex>
#include <atomic>
#include <thread>
#include <string>
#include <iostream>

/**
 * @brief Wrapper for the Haply Inverse3 haptic device.
 *
 * Spawns a 1 kHz thread that exchanges forces and positions with the device
 * via the Haply HardwareAPI. Thread-safe API following the same two-mutex +
 * stale-flag pattern used by HapticDeviceManager.
 *
 * When compiled with NO_HAPLY_HARDWARE_API or when no device is detected,
 * all methods become safe no-ops and isConnected() returns false.
 */
class HaplyInverse3Device
{
public:
    /// Construct and attempt to connect. Empty port = auto-detect.
    explicit HaplyInverse3Device(const std::string& serial_port = "");

    ~HaplyInverse3Device();

    // Non-copyable, non-movable
    HaplyInverse3Device(const HaplyInverse3Device&) = delete;
    HaplyInverse3Device& operator=(const HaplyInverse3Device&) = delete;

    /// True if the device was found and woken up successfully.
    bool isConnected() const { return _connected; }

    /// Get the latest end-effector position [m] in device frame.
    /// Uses stale-flag lazy copy (thread-safe).
    Vec3r position();

    /// Get the latest end-effector velocity [m/s] in device frame.
    Vec3r velocity();

    /// Set the force [N] to send to the device on the next loop iteration.
    /// Each axis is clamped to [-3.3, 3.3] N (hardware limit).
    void setForce(const Vec3r& force);

private:
    /// Background thread entry point (~1 kHz).
    void _runLoop();

    /// Try to auto-detect the Inverse3 serial port.
    static std::string _autoDetectPort();

    // ---- device state written by the 1 kHz thread, read by sim thread ----
    struct DeviceState
    {
        Vec3r position = Vec3r::Zero();
        Vec3r velocity = Vec3r::Zero();
        bool stale = true;
    };
    DeviceState _device_state;   // raw (written by thread under _state_mtx)
    DeviceState _copied_state;   // lazy copy (read by sim thread)
    std::mutex _state_mtx;

    // ---- force input written by sim thread, read by 1 kHz thread ----
    Vec3r _commanded_force = Vec3r::Zero();
    std::mutex _input_mtx;

    // ---- thread management ----
    std::atomic<bool> _running{false};
    std::thread _thread;
    bool _connected = false;

    // ---- opaque pointers to Haply device (avoids leaking HardwareAPI headers) ----
#ifndef NO_HAPLY_HARDWARE_API
    void* _stream_handle = nullptr;  // Haply::HardwareAPI::IO::SerialStream*
    void* _device_handle = nullptr;  // Haply::HardwareAPI::Devices::Inverse3*
#endif
};

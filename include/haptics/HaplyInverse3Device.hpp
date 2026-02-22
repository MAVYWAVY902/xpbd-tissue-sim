#pragma once

#include "common/types.hpp"
#include <string>
#include <iostream>

/**
 * @brief Wrapper for the Haply Inverse3 haptic device.
 *
 * Uses synchronous polling: the simulation calls poll() each time step
 * to exchange forces and read position/velocity. This matches the
 * single-threaded pattern that the HardwareAPI requires.
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

    /// Get the initial validated position recorded at construction time.
    Vec3r initialPosition() const { return _initial_position; }

    /// Poll the device: send the current force command and read back
    /// position/velocity. Call this once per simulation time step.
    /// Returns true if a valid (non-stale) response was received.
    bool poll();

    /// Get the latest end-effector position [m] in device frame.
    Vec3r position() const { return _position; }

    /// Get the latest end-effector velocity [m/s] in device frame.
    Vec3r velocity() const { return _velocity; }

    /// Set the force [N] to send to the device on the next poll() call.
    /// Each axis is clamped to [-3.3, 3.3] N (hardware limit).
    void setForce(const Vec3r& force);

private:
    /// Try to auto-detect the Inverse3 serial port.
    static std::string _autoDetectPort();

    // ---- device state (single-threaded, no mutex needed) ----
    Vec3r _position = Vec3r::Zero();
    Vec3r _velocity = Vec3r::Zero();
    Vec3r _commanded_force = Vec3r::Zero();

    bool _connected = false;
    Vec3r _initial_position = Vec3r::Zero();
    int _poll_count = 0;

    // ---- opaque pointers to Haply device (avoids leaking HardwareAPI headers) ----
#ifndef NO_HAPLY_HARDWARE_API
    void* _stream_handle = nullptr;  // Haply::HardwareAPI::IO::SerialStream*
    void* _device_handle = nullptr;  // Haply::HardwareAPI::Devices::Inverse3*
#endif
};

#pragma once

#include "common/types.hpp"
#include <string>
#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>

/**
 * @brief Wrapper for the Haply Inverse3 haptic device + optional VerseGrip.
 *
 * Runs serial I/O on dedicated background threads so the simulation
 * never blocks on serial communication:
 *   - Inverse3 thread: polls position/velocity in a tight loop (~7 kHz)
 *   - VerseGrip thread: polls orientation independently
 *
 * The simulation just reads position()/orientation() for latest values.
 */
class HaplyInverse3Device
{
public:
    explicit HaplyInverse3Device(const std::string& serial_port = "");
    ~HaplyInverse3Device();

    HaplyInverse3Device(const HaplyInverse3Device&) = delete;
    HaplyInverse3Device& operator=(const HaplyInverse3Device&) = delete;

    bool isConnected() const { return _connected; }
    bool hasVerseGrip() const { return _versegrip_connected; }
    Vec3r initialPosition() const { return _initial_position; }

    /// No-op — polling handled by background threads.
    bool poll() { return _connected; }

    /// Thread-safe getters — return latest values from background threads.
    Vec3r position() const;
    Vec3r velocity() const;
    Vec4r orientation() const;

    void setForce(const Vec3r& force);

    void toggleTestForce() { _test_force_enabled = !_test_force_enabled; _test_use_joint_torques = false; }
    void toggleTestTorque() { _test_force_enabled = !_test_force_enabled; _test_use_joint_torques = true; }

private:
    static std::string _autoDetectPort();

    /// Inverse3 polling thread — position/velocity in tight loop
    void _inverse3ThreadFunc();
    /// VerseGrip polling thread — orientation independently
    void _versegripThreadFunc();

    // ---- Thread-safe position data (Inverse3 thread writes, sim reads) ----
    mutable std::mutex _pos_mutex;
    Vec3r _position = Vec3r::Zero();
    Vec3r _velocity = Vec3r::Zero();

    // ---- Thread-safe orientation data (VerseGrip thread writes, sim reads) ----
    mutable std::mutex _orient_mutex;
    Vec4r _orientation = Vec4r(0, 0, 0, 1);

    // ---- Background threads ----
    std::thread _inverse3_thread;
    std::thread _versegrip_thread;
    std::atomic<bool> _poll_running{false};

    // ---- State ----
    Vec3r _commanded_force = Vec3r::Zero();
    bool _connected = false;
    Vec3r _initial_position = Vec3r::Zero();
    bool _test_force_enabled = false;
    bool _test_use_joint_torques = false;
    bool _versegrip_connected = false;
    bool _use_versegrip_api = false;

#ifndef NO_HAPLY_HARDWARE_API
    void* _stream_handle = nullptr;
    void* _device_handle = nullptr;
    void* _handle_stream_handle = nullptr;
    void* _handle_device_handle = nullptr;
#endif
};

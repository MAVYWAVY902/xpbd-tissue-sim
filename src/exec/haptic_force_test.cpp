/**
 * Minimal standalone force test for Haply Inverse3.
 * No simulation, no VerseGrip, no extra queries — just force commands.
 *
 * Usage: ./HapticForceTest [port]
 *   port defaults to /dev/ttyACM0
 *
 * Tests:
 *   1. Connect and wake up device
 *   2. Check power supply
 *   3. Check torque scaling (enable if disabled)
 *   4. Send zero force for 2 seconds (baseline)
 *   5. Send constant upward force (0, 2, 0) N for 5 seconds
 *   6. Send JointTorques (50, 50, 50) Nmm for 5 seconds
 *   7. Query motor currents during each phase
 */

#include <iostream>
#include <chrono>
#include <thread>
#include <cstring>
#include <unistd.h>

#ifndef NO_HAPLY_HARDWARE_API
#include "HardwareAPI.h"
#endif

int main(int argc, char** argv)
{
#ifdef NO_HAPLY_HARDWARE_API
    std::cerr << "Compiled without Haply HardwareAPI. Cannot test." << std::endl;
    return 1;
#else
    std::string port = "/dev/ttyACM0";
    if (argc > 1) port = argv[1];

    std::cout << "=== Haply Inverse3 Force Test ===" << std::endl;
    std::cout << "Port: " << port << std::endl;

    // ---- 1. Connect ----
    std::cout << "\n--- Step 1: Connect ---" << std::endl;
    Haply::HardwareAPI::IO::SerialStream* stream = nullptr;
    Haply::HardwareAPI::Devices::Inverse3* device = nullptr;
    try
    {
        stream = new Haply::HardwareAPI::IO::SerialStream(port.c_str());
        // Wait 3 seconds for device to recover from DTR-triggered reboot
        std::cout << "Waiting 3 seconds for device to boot after port open..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        device = new Haply::HardwareAPI::Devices::Inverse3(stream);
        auto info = device->DeviceWakeup();
        std::cout << "Device ID: " << info.device_id << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "FAILED to connect: " << e.what() << std::endl;
        return 1;
    }

    // Verify with initial read
    Haply::HardwareAPI::Devices::Inverse3::EndEffectorForceRequest zero_req{};
    auto init_resp = device->EndEffectorForce(zero_req);
    std::cout << "Initial position: (" << init_resp.position[0] << ", "
              << init_resp.position[1] << ", " << init_resp.position[2] << ")" << std::endl;

    // ---- 2. Power check ----
    std::cout << "\n--- Step 2: Power Supply ---" << std::endl;
    auto power = device->DevicePowerQuery();
    std::cout << "Power: " << (power.powered ? "CONNECTED" : "NOT CONNECTED") << std::endl;
    if (!power.powered)
    {
        std::cerr << "WARNING: No power — motors cannot generate force!" << std::endl;
        std::cerr << "Continue anyway? (forces will be zero) [y/N] ";
        // Don't wait for input, just continue for testing
    }

    // ---- 3. Torque scaling ----
    std::cout << "\n--- Step 3: Torque Scaling ---" << std::endl;
    auto ts = device->GetTorqueScaling();
    std::cout << "Torque scaling: " << (ts.enabled ? "ENABLED" : "DISABLED") << std::endl;
    if (!ts.enabled)
    {
        std::cout << "Enabling torque scaling..." << std::endl;
        Haply::HardwareAPI::Devices::Inverse3::TorqueScalingPayload ts_req;
        ts_req.enabled = true;
        auto ts_resp = device->SetTorqueScaling(ts_req);
        std::cout << "Torque scaling now: " << (ts_resp.enabled ? "ENABLED" : "DISABLED") << std::endl;
    }

    // Gravity compensation check
    auto gc = device->GetGravityCompensation();
    std::cout << "Gravity compensation: " << (gc.enabled ? "ENABLED" : "DISABLED")
              << "  scale=" << gc.gravity_scale_factor << std::endl;

    // Motor currents baseline
    auto mc = device->MotorCurrentsQuery();
    std::cout << "Motor currents (idle): (" << mc.currents[0] << ", "
              << mc.currents[1] << ", " << mc.currents[2] << ") A" << std::endl;

    // ---- 4. Zero force baseline (2s) ----
    std::cout << "\n--- Step 4: Zero Force Baseline (2 seconds) ---" << std::endl;
    std::cout << "Sending zero force..." << std::endl;
    auto t0 = std::chrono::steady_clock::now();
    int count = 0;
    while (std::chrono::steady_clock::now() - t0 < std::chrono::seconds(2))
    {
        Haply::HardwareAPI::Devices::Inverse3::EndEffectorForceRequest req{};
        auto resp = device->EndEffectorForce(req);
        count++;
        if (count % 500 == 0)
        {
            std::cout << "  pos=(" << resp.position[0] << ", " << resp.position[1]
                      << ", " << resp.position[2] << ")  count=" << count << std::endl;
        }
    }
    std::cout << "Zero force: " << count << " cycles in 2s = " << (count / 2) << " Hz" << std::endl;
    mc = device->MotorCurrentsQuery();
    std::cout << "Motor currents (zero force): (" << mc.currents[0] << ", "
              << mc.currents[1] << ", " << mc.currents[2] << ") A" << std::endl;

    // Re-enter force mode after the query
    device->EndEffectorForce(zero_req);

    // ---- 5. EndEffectorForce test (5s) ----
    std::cout << "\n--- Step 5: EndEffectorForce Test (5 seconds) ---" << std::endl;
    std::cout << "Sending force (0, 2.0, 0) N — you should feel UPWARD push!" << std::endl;
    std::cout << "HOLD THE DEVICE FIRMLY." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    t0 = std::chrono::steady_clock::now();
    count = 0;
    float last_pos[3] = {0};
    while (std::chrono::steady_clock::now() - t0 < std::chrono::seconds(5))
    {
        Haply::HardwareAPI::Devices::Inverse3::EndEffectorForceRequest req{};
        req.force[0] = 0.0f;
        req.force[1] = 2.0f;   // 2N upward
        req.force[2] = 0.0f;
        auto resp = device->EndEffectorForce(req);
        last_pos[0] = resp.position[0];
        last_pos[1] = resp.position[1];
        last_pos[2] = resp.position[2];
        count++;
        if (count % 500 == 0)
        {
            std::cout << "  pos=(" << resp.position[0] << ", " << resp.position[1]
                      << ", " << resp.position[2] << ")  force=(0, 2, 0) N  count=" << count << std::endl;
        }
    }
    std::cout << "EE Force: " << count << " cycles in 5s = " << (count / 5) << " Hz" << std::endl;
    std::cout << "Final pos=(" << last_pos[0] << ", " << last_pos[1] << ", " << last_pos[2] << ")" << std::endl;

    // Check currents immediately after
    mc = device->MotorCurrentsQuery();
    std::cout << "Motor currents (after 2N force): (" << mc.currents[0] << ", "
              << mc.currents[1] << ", " << mc.currents[2] << ") A" << std::endl;

    // Clear force
    device->EndEffectorForce(zero_req);
    std::cout << "Force cleared. Pause 2 seconds..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // ---- 6. JointTorques test (5s) ----
    std::cout << "\n--- Step 6: JointTorques Test (5 seconds) ---" << std::endl;
    std::cout << "Sending torques (50, 50, 50) Nmm — you should feel resistance!" << std::endl;
    std::cout << "(SDK says ~20 Nmm overcomes internal friction)" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    t0 = std::chrono::steady_clock::now();
    count = 0;
    while (std::chrono::steady_clock::now() - t0 < std::chrono::seconds(5))
    {
        Haply::HardwareAPI::Devices::Inverse3::JointTorquesRequest treq{};
        treq.torques[0] = 50.0f;
        treq.torques[1] = 50.0f;
        treq.torques[2] = 50.0f;
        auto tresp = device->JointTorques(treq);
        count++;
        if (count % 500 == 0)
        {
            std::cout << "  angles=(" << tresp.angles[0] << ", " << tresp.angles[1]
                      << ", " << tresp.angles[2] << ") deg  count=" << count << std::endl;
        }
    }
    std::cout << "JointTorques: " << count << " cycles in 5s = " << (count / 5) << " Hz" << std::endl;

    // Check currents
    mc = device->MotorCurrentsQuery();
    std::cout << "Motor currents (after 50 Nmm torque): (" << mc.currents[0] << ", "
              << mc.currents[1] << ", " << mc.currents[2] << ") A" << std::endl;

    // ---- 7. Ramp test: slowly increase force ----
    std::cout << "\n--- Step 7: Force Ramp Test (5 seconds) ---" << std::endl;
    std::cout << "Ramping Y-force from 0 to 3.3N over 5 seconds..." << std::endl;

    t0 = std::chrono::steady_clock::now();
    count = 0;
    while (std::chrono::steady_clock::now() - t0 < std::chrono::seconds(5))
    {
        float elapsed = std::chrono::duration<float>(
            std::chrono::steady_clock::now() - t0).count();
        float force_y = (elapsed / 5.0f) * 3.3f;  // ramp 0 → 3.3N

        Haply::HardwareAPI::Devices::Inverse3::EndEffectorForceRequest req{};
        req.force[0] = 0.0f;
        req.force[1] = force_y;
        req.force[2] = 0.0f;
        auto resp = device->EndEffectorForce(req);
        count++;
        if (count % 500 == 0)
        {
            std::cout << "  force_y=" << force_y << " N  pos=(" << resp.position[0]
                      << ", " << resp.position[1] << ", " << resp.position[2]
                      << ")" << std::endl;
        }
    }
    std::cout << "Ramp: " << count << " cycles in 5s = " << (count / 5) << " Hz" << std::endl;

    // ---- Cleanup ----
    std::cout << "\n--- Cleanup ---" << std::endl;
    device->EndEffectorForce(zero_req);
    std::cout << "Forces zeroed. Test complete." << std::endl;

    // Check power one more time
    power = device->DevicePowerQuery();
    std::cout << "Final power check: " << (power.powered ? "CONNECTED" : "NOT CONNECTED") << std::endl;

    delete device;
    delete stream;
    return 0;
#endif
}

/**
 * Minimal Inverse3 connection test — no OpenGL, no simulation.
 * Retries wakeup until valid, then prints position for 10 seconds.
 */
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>

#include "HardwareAPI.h"

int main()
{
    // 1. Find serial port
    std::string port;
    const char* candidates[] = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"};
    for (const char* p : candidates)
    {
        if (access(p, R_OK | W_OK) == 0)
        {
            port = p;
            break;
        }
    }
    if (port.empty())
    {
        std::cerr << "No /dev/ttyACM* device found." << std::endl;
        return 1;
    }
    std::cout << "Found port: " << port << std::endl;

    // 2. Retry initialization until we get valid position data
    Haply::HardwareAPI::IO::SerialStream* stream = nullptr;
    Haply::HardwareAPI::Devices::Inverse3* device = nullptr;

    constexpr int kMaxRetries = 5;
    for (int attempt = 1; attempt <= kMaxRetries; ++attempt)
    {
        std::cout << "\n--- Attempt " << attempt << " ---" << std::endl;

        stream = new Haply::HardwareAPI::IO::SerialStream(port.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        device = new Haply::HardwareAPI::Devices::Inverse3(stream);
        auto info = device->DeviceWakeup();
        std::cout << "Device ID: " << info.device_id << std::endl;

        // Test read
        Haply::HardwareAPI::Devices::Inverse3::EndEffectorForceRequest req{};
        auto resp = device->EndEffectorForce(req);
        std::cout << "Test pos: (" << resp.position[0] << ", "
                  << resp.position[1] << ", " << resp.position[2] << ")" << std::endl;

        bool valid = (resp.position[0] != 0.0f || resp.position[1] != 0.0f
                      || resp.position[2] != 0.0f);

        if (valid)
        {
            std::cout << "Connection OK!" << std::endl;
            break;
        }

        std::cerr << "Got zero position, retrying..." << std::endl;
        delete device; device = nullptr;
        delete stream; stream = nullptr;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    if (!device)
    {
        std::cerr << "Failed after " << kMaxRetries << " attempts." << std::endl;
        return 1;
    }

    // 3. Read position in a loop for 10 seconds
    std::cout << "\nReading end-effector position for 10 seconds...\n" << std::endl;

    auto start = std::chrono::steady_clock::now();
    int count = 0;
    int errors = 0;

    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(10))
    {
        try
        {
            Haply::HardwareAPI::Devices::Inverse3::EndEffectorForceRequest req{};
            auto resp = device->EndEffectorForce(req);
            ++count;

            if (count % 500 == 0)
            {
                std::cout << "  pos=(" << resp.position[0] << ", "
                          << resp.position[1] << ", " << resp.position[2]
                          << ")  vel=(" << resp.velocity[0] << ", "
                          << resp.velocity[1] << ", " << resp.velocity[2]
                          << ")  [" << count << " samples, " << errors << " errors]"
                          << std::endl;
            }
        }
        catch (const std::exception& e)
        {
            ++errors;
            if (errors <= 5)
                std::cerr << "  Error: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    std::cout << "\nDone. " << count << " successful reads, " << errors << " errors." << std::endl;

    delete device;
    delete stream;
    return 0;
}

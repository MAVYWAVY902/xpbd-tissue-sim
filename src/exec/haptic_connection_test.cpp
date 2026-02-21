/**
 * Minimal Inverse3 connection test — no OpenGL, no simulation.
 * Just opens the device, wakes it up, and prints position for 10 seconds.
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

    // 2. Open serial stream
    std::cout << "Opening serial stream..." << std::endl;
    Haply::HardwareAPI::IO::SerialStream stream(port.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 3. Create device and wake up
    std::cout << "Creating Inverse3 device..." << std::endl;
    Haply::HardwareAPI::Devices::Inverse3 device(&stream);

    std::cout << "Sending DeviceWakeup..." << std::endl;
    auto info = device.DeviceWakeup();
    std::cout << "Device ID: " << info.device_id << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 4. Read position in a loop for 10 seconds
    std::cout << "\nReading end-effector position for 10 seconds...\n" << std::endl;

    auto start = std::chrono::steady_clock::now();
    int count = 0;
    int errors = 0;

    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(10))
    {
        try
        {
            // Send zero force, get position back
            Haply::HardwareAPI::Devices::Inverse3::EndEffectorForceRequest req;
            req.force[0] = 0.0f;
            req.force[1] = 0.0f;
            req.force[2] = 0.0f;
            auto resp = device.EndEffectorForce(req);
            ++count;

            // Print every ~500ms (assuming ~1kHz loop)
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
            else if (errors == 6)
                std::cerr << "  (suppressing further errors...)" << std::endl;
            // Small delay on error to avoid tight spin
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    std::cout << "\nDone. " << count << " successful reads, " << errors << " errors." << std::endl;
    return 0;
}

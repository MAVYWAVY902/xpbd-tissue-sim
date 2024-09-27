#include "HapticDeviceManager.hpp"

HDCallbackCode HDCALLBACK positionCallbackSingle(void *data)
{
    HapticDeviceManager* device_manager = static_cast<HapticDeviceManager*>(data);
    if (!device_manager)
    {
        std::cerr << "Position callback failed!" << std::endl;
        assert(0);
    }

    hduVector3Dd position_hd;

    /* Begin haptics frame.  ( In general, all state-related haptics calls
    should be made within a frame. ) */
    hdBeginFrame(device_manager->hHD());

    /* Get the current position of the device. */
    hdGetDoublev(HD_CURRENT_POSITION, position_hd);

    /* End haptics frame. */
    hdEndFrame(device_manager->hHD());

    Eigen::Vector3d position;
    position(0) = position_hd[0];
    position(1) = position_hd[1];
    position(2) = position_hd[2];

    // std::cout << position << std::endl;

    device_manager->setPosition(position);

    return HD_CALLBACK_CONTINUE;
}

HapticDeviceManager::HapticDeviceManager()
{
    HDErrorInfo error;
    /* Initialize the device, must be done before attempting to call any hd 
    functions. Passing in HD_DEFAULT_DEVICE causes the default device to be 
    initialized. */
    _hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError())) 
    {
        std::cerr << KRED;
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        std::cerr << RST;
        assert(0);
    }

    std::cout << "Found device model: " << BOLD << hdGetString(HD_DEVICE_MODEL_TYPE) << RST << std::endl;

    HHD hHD = hdGetCurrentDevice();
    std::cout << hHD << ", " << _hHD << std::endl;
    HDSchedulerHandle position_callback_handle = hdScheduleAsynchronous(positionCallbackSingle, this, HD_MAX_SCHEDULER_PRIORITY);
    hdStartScheduler();

    // /* Check for errors and abort if so. */
    // if (HD_DEVICE_ERROR(error = hdGetError()))
    // {
    //     hduPrintError(stderr, &error, "Failed to start scheduler");
    // }
}

HapticDeviceManager::~HapticDeviceManager()
{
    /* For cleanup, unschedule callback and stop the scheduler. */
    hdStopScheduler();
    // hdUnschedule(position_callback_handle);

    /* Disable the device. */
    hdDisableDevice(_hHD);
}

Eigen::Vector3d HapticDeviceManager::getPosition()
{
    // hdScheduleSynchronous(positionCallbackSingle, 0, HD_MAX_SCHEDULER_PRIORITY);
    // hdStartScheduler();
    return _position;
}

void HapticDeviceManager::setPosition(const Eigen::Vector3d& new_position)
{
    _position = new_position;
}
#include "HapticDeviceManager.hpp"

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
    HDSchedulerHandle update_callback_handle = hdScheduleAsynchronous(_updateCallback, this, HD_MAX_SCHEDULER_PRIORITY);
    hdStartScheduler();

    // /* Check for errors and abort if so. */
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start scheduler");
    }
}

HapticDeviceManager::~HapticDeviceManager()
{
    /* For cleanup, unschedule callback and stop the scheduler. */
    hdStopScheduler();
    // hdUnschedule(position_callback_handle);

    /* Disable the device. */
    hdDisableDevice(_hHD);
}

HDCallbackCode HDCALLBACK HapticDeviceManager::_updateCallback(void *data)
{
    HapticDeviceManager* device_manager = static_cast<HapticDeviceManager*>(data);
    if (!device_manager)
    {
        std::cerr << "Update callback failed!" << std::endl;
        assert(0);
    }

    hduVector3Dd position_hd;
    int nButtons;
    HDboolean button1_pressed_hd;
    HDboolean button2_pressed_hd;

    /* Begin haptics frame.  ( In general, all state-related haptics calls
    should be made within a frame. ) */
    hdBeginFrame(device_manager->hHD());

    /* Get the current position of the device. */
    hdGetDoublev(HD_CURRENT_POSITION, position_hd);

    /* Retrieve the current button(s). */
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    
    /* In order to get the specific button 1 state, we use a bitmask to
       test for the HD_DEVICE_BUTTON_1 bit. */
    button1_pressed_hd = (nButtons & HD_DEVICE_BUTTON_1) ? HD_TRUE : HD_FALSE;
    button2_pressed_hd = (nButtons & HD_DEVICE_BUTTON_2) ? HD_TRUE : HD_FALSE;


    /* End haptics frame. */
    hdEndFrame(device_manager->hHD());

    device_manager->setDeviceData(button1_pressed_hd, button2_pressed_hd, position_hd);
    device_manager->setStale(true);

    return HD_CALLBACK_CONTINUE;
}

HDCallbackCode HDCALLBACK HapticDeviceManager::_copyCallback(void *data)
{
    HapticDeviceManager* device_manager = static_cast<HapticDeviceManager*>(data);
    if (!device_manager)
    {
        std::cerr << "Copy callback failed!" << std::endl;
        assert(0);
    }

    device_manager->copyState();

    return HD_CALLBACK_DONE;
}

void HapticDeviceManager::copyState()
{
    _position(0) = _device_data.device_position[0];
    _position(1) = _device_data.device_position[1];
    _position(2) = _device_data.device_position[2];

    _button1_pressed = _device_data.button1_state;
    _button2_pressed = _device_data.button2_state;

    _stale = false; 
}

void HapticDeviceManager::setDeviceData(const HDboolean& b1_state, const HDboolean& b2_state, const hduVector3Dd& position)
{
    _device_data.button1_state = b1_state;
    _device_data.button2_state = b2_state;
    _device_data.device_position = position;
}

Eigen::Vector3d HapticDeviceManager::position()
{
    if (_stale)
    {
        hdScheduleSynchronous(_copyCallback, this, HD_MIN_SCHEDULER_PRIORITY);
    }

    return _position;
}

bool HapticDeviceManager::button1Pressed()
{
    if (_stale)
    {
        hdScheduleSynchronous(_copyCallback, this, HD_MIN_SCHEDULER_PRIORITY);
    }

    return _button1_pressed;
}

bool HapticDeviceManager::button2Pressed()
{
    if (_stale)
    {
        hdScheduleSynchronous(_copyCallback, this, HD_MIN_SCHEDULER_PRIORITY);
    }

    return _button2_pressed;
}
#ifndef __HAPTIC_DEVICE_MANAGER_HPP
#define __HAPTIC_DEVICE_MANAGER_HPP

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

#include "colors.hpp"
#include <iostream>
#include <cassert>

#include <Eigen/Dense>

struct HapticDeviceData
{
    HDboolean button1_state;
    HDboolean button2_state;
    hduVector3Dd device_position;
    HDErrorInfo error;
};

class HapticDeviceManager
{
    public:
    explicit HapticDeviceManager();
    

    ~HapticDeviceManager();

    Eigen::Vector3d position();
    bool button1Pressed();
    bool button2Pressed();

    const HHD hHD() const { return _hHD; }

    protected:
    static HDCallbackCode HDCALLBACK _updateCallback(void *data);
    static HDCallbackCode HDCALLBACK _copyCallback(void *data);

    // void setPosition(const Eigen::Vector3d& new_position) { _position = new_position; }
    // void setButtonPressed(const bool pressed) { _button_pressed = pressed; }
    void copyState();
    void setStale(const bool stale) { _stale = stale; }
    inline void setDeviceData(const HDboolean& b1_state, const HDboolean& b2_state, const hduVector3Dd& position);

    HHD _hHD;
    HapticDeviceData _device_data;
    Eigen::Vector3d _position;
    bool _button1_pressed;
    bool _button2_pressed;
    bool _stale;
};

#endif // __HAPTIC_DEVICE_MANAGER_HPP
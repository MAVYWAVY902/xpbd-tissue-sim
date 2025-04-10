#ifndef __HAPTIC_DEVICE_MANAGER_HPP
#define __HAPTIC_DEVICE_MANAGER_HPP

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

#include "common/colors.hpp"
#include <iostream>
#include <cassert>
#include <mutex>

#include "common/types.hpp"

struct HapticDeviceData
{
    HDboolean button1_state;
    HDboolean button2_state;
    hduVector3Dd device_position;
    HDdouble device_transform[16];
    HDErrorInfo error;
};

class HapticDeviceManager
{
    public:
    explicit HapticDeviceManager();
    

    ~HapticDeviceManager();

    Vec3r position();
    Mat3r orientation();
    bool button1Pressed();
    bool button2Pressed();

    const HHD hHD() const { return _hHD; }

    void setForce(const Eigen::Vector3d& force);
    const Eigen::Vector3d& force() const { return _force; }

    protected:
    static HDCallbackCode HDCALLBACK _updateCallback(void *data);
    static HDCallbackCode HDCALLBACK _copyCallback(void *data);

    void copyState();
    void setStale(const bool stale) { _stale = stale; }
    inline void setDeviceData(const HDboolean& b1_state, const HDboolean& b2_state, const hduVector3Dd& position, const HDdouble* transform);

    private:
    HHD _hHD;
    HapticDeviceData _device_data;
    Vec3r _position;
    bool _button1_pressed;
    bool _button2_pressed;
    Mat3r _orientation;
    bool _stale;

    Eigen::Vector3d _force;

    std::mutex _state_mtx;
};

#endif // __HAPTIC_DEVICE_MANAGER_HPP
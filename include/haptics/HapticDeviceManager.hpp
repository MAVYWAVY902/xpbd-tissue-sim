#ifndef __HAPTIC_DEVICE_MANAGER_HPP
#define __HAPTIC_DEVICE_MANAGER_HPP

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

#include "colors.hpp"
#include <iostream>
#include <cassert>

#include <Eigen/Dense>

HDCallbackCode HDCALLBACK positionCallbackSingle(void *data);


class HapticDeviceManager
{
    public:
    explicit HapticDeviceManager();
    

    ~HapticDeviceManager();

    Eigen::Vector3d getPosition();

    void setPosition(const Eigen::Vector3d& new_position);

    const HHD hHD() const { return _hHD; }

    protected:
    HHD _hHD;
    Eigen::Vector3d _position;
};

#endif // __HAPTIC_DEVICE_MANAGER_HPP
#ifndef __VIRTUOSO_ARM_CONFIG_HPP
#define __VIRTUOSO_ARM_CONFIG_HPP

#include "config/ObjectConfig.hpp"

class VirtuosoArmConfig : public ObjectConfig
{
    public:
    explicit VirtuosoArmConfig(const YAML::Node& node)
        : ObjectConfig(node)
    {

    }

};

#endif // __VIRTUOSO_ARM_CONFIG_HPP
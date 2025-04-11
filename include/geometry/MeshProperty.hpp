#ifndef __MESH_PROPERTY_HPP
#define __MESH_PROPERTY_HPP

#include "common/types.hpp"
#include "common/TypeList.hpp"
#include "common/VariadicVectorContainer.hpp"

namespace Geometry
{

template<typename T>
class MeshProperty
{

    public:
    MeshProperty(const std::string& name, int size)
        : _name(name), _properties(size)
    {
    }

    MeshProperty(const std::string& name, int size, const T& default_value)
        : _name(name), _properties(size, default_value)
    {
    }

    const std::string& name() const { return _name; }

    const T& get(int index) const { return _properties[index]; }
    void set(int index, const T& new_val) { _properties[index] = new_val; }

    const std::vector<T>& properties() const { return _properties; }

    protected:
    std::string _name;
    std::vector<T> _properties; 

};

typedef TypeList<bool, int, Real> MeshPropertyTypeList;

template <typename PropertyTypeList> class PropertyContainer;

template <typename ...PropertyTypes>
class PropertyContainer<TypeList<PropertyTypes...>> : public VariadicVectorContainer<MeshProperty<PropertyTypes>...>
{
};



} // namespace Geometry


#endif // __MESH_PROPERTY_HPP
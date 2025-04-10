#ifndef __MONITORED_VECTOR_HPP
#define __MONITORED_VECTOR_HPP

#include <vector>

template<class T>
class MonitoredVector
{
    public:
    MonitoredVector(size_t initial_size)
        : _vec(initial_size), _state_changed(false)
    {
    }

    MonitoredVector()
        : _vec(), _state_changed(false)
    {
    }

    /** Incomplete Vector Element Access 
    * Assumes that any non-const access to the vector changes its state.
    */

    T& at(size_t pos)
    {
        _state_changed = true;
        return _vec.at(pos);
    }

    const T& at(size_t pos) const
    {
        return _vec.at(pos);
    }

    T& operator[](size_t pos)
    {
        _state_changed = true;
        return _vec[pos];
    }

    const T& operator[](size_t pos) const
    {
        return _vec[pos];
    }

    T* data()
    {
        _state_changed = true;
        return _vec.data();
    }

    const T* data() const
    {
        return _vec.data();
    }



    /** Incomplete Vector Capacity */

    bool empty() const
    {
        return _vec.empty();
    }

    size_t size() const
    {
        return _vec.size();
    }

    size_t capacity() const
    {
        return _vec.capacity();
    }

    void reserve(size_t new_cap)
    {
        return _vec.reserve(new_cap);
    }

    

    /** Incomplete Vector Modifiers */

    void push_back(const T& value)
    {
        _state_changed = true;
        _vec.push_back(value);
    }

    void push_back(T&& value)
    {
        _state_changed = true;
        _vec.push_back(std::forward<T>(value));
    }

    typename std::vector<T>::iterator insert(typename std::vector<T>::const_iterator pos, const T& value)
    {
        _state_changed = true;
        _vec.insert(pos, value);
    }

    typename std::vector<T>::iterator insert(typename std::vector<T>::const_iterator pos, T&& value)
    {
        _state_changed = true;
        _vec.insert(pos, std::forward<T>(value));
    }

    void clear()
    {
        _state_changed = true;
        _vec.clear();
    }

    typename std::vector<T>::iterator erase(typename std::vector<T>::const_iterator pos)
    {
        _state_changed = true;
        return _vec.erase(pos);
    }

    typename std::vector<T>::iterator erase(typename std::vector<T>::const_iterator first, typename std::vector<T>::const_iterator last)
    {
        _state_changed = true;
        return _vec.erase(first, last);
    }

    void resize(size_t count)
    {
        _state_changed = true;
        _vec.resize(count);
    }

    void resize(size_t count, const T& value)
    {
        _state_changed = true;
        _vec.resize(count, value);
    }



    /** Accessors for vector. If non-const accessor is used, we assume that the state is changing. */

    std::vector<T>& vec()
    {
        _state_changed = true;
        return _vec;
    }

    const std::vector<T>& vec() const
    {
        return _vec;
    }



    /** Accessors for state change */

    void resetStateChanged() { _state_changed = false; }
    bool stateChanged() const { return _state_changed; }

    private:
    std::vector<T> _vec;
    bool _state_changed;
};

#endif // __MONITORED_VECTOR_HPP
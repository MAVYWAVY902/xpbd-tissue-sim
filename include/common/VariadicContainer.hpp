#ifndef __VARIADIC_CONTAINER_HPP
#define __VARIADIC_CONTAINER_HPP

#include <vector>

// adapted from this StackOverflow answer: https://stackoverflow.com/a/53112843

/** A class that stores a heterogenous collection of types (similar to std::tuple without all the bells and whistles).
 * Uses CRTP inheritance to recursively add a private member variable for each type.
 */
template<class L, class... R> class VariadicContainer;

// Recursive base case
template<class L>
class VariadicContainer<L>
{
    protected:
    const L& _get() const
    {
        return _obj;
    }

    L& _get()
    {
        return _obj;
    }

    void _set(const L& obj)
    {
        _obj = obj;
    }

    void _set(L&& obj)
    {
        _obj = std::move(obj);
    }

    private:
    L _obj;
};

template<class L, class... R>
class VariadicContainer : public VariadicContainer<L>, public VariadicContainer<R...>
{
    public:
    template<class T>
    const T& get() const
    {
        return this->VariadicContainer<T>::_get();
    }

    template<class T>
    T& get()
    {
        return this->VariadicContainer<T>::_get();
    }

    template<class T>
    void set(const T& obj)
    {
        return this->VariadicContainer<T>::_set(obj);
    }

    template<class T>
    void set(T&& obj)
    {
        return this->VariadicContainer<T>::_set(std::move(obj));
    }
};

#endif // __VARIADIC_CONTAINER_HPP
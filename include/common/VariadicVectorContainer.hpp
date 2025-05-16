#ifndef __VARIADIC_VECTOR_CONTAINER_HPP
#define __VARIADIC_VECTOR_CONTAINER_HPP

#include <vector>
#include <iostream>

// adapted from this StackOverflow answer: https://stackoverflow.com/a/53112843

/** A class that stores a heterogeneous collection of vector types (i.e. vectors that each store different types).
 * The types stored are determined by the template parameters, meaning that it is determined at compile time.
 * 
 * Uses CRTP inheritance to recursively add a private member vector variable for each type.
 */
template<class L, class... R> class VariadicVectorContainer;

template<class L>
class VariadicVectorContainer<L>
{
    protected:
    const std::vector<L>& _get() const
    {
        return _vec;
    }

    std::vector<L>& _get()
    {
        return _vec;
    }

    void _resize(int size)
    {
        _vec.resize(size);
    }

    void _reserve(int size)
    {
        _vec.reserve(size);
    }

    void _push_back(const L& elem)
    {
        _vec.push_back(elem);
    }

    void _push_back(L&& elem)
    {
        _vec.push_back(std::move(elem));
    }

    template<class ...Args>
    L& _emplace_back(Args&&... args)
    {
        return _vec.emplace_back(std::forward<Args>(args)...);
    } 

    L& _set(int index, const L& elem)
    {
        _vec[index] = elem;
        return _vec[index];
    }

    L& _set(int index, L&& elem)
    {
        _vec[index] = std::move(elem);
        return _vec[index];
    }

    void _clear()
    {
        _vec.clear();
    }

    // internal method for visiting elements
    template<typename Visitor>
    void _for_each_element(Visitor&& visitor) const
    {
        for (const auto& elem : _vec)
        {
            visitor(elem);
        }
    }

    template<typename Visitor>
    void _for_each_element(Visitor&& visitor)
    {
        for (auto& elem : _vec)
        {
            visitor(elem);
        }
    }

    private:
    std::vector<L> _vec;
};

template<class L, class... R>
class VariadicVectorContainer : public VariadicVectorContainer<L>, public VariadicVectorContainer<R...>
{
    public:
    template<class T>
    const std::vector<T>& get() const
    {
        return this->VariadicVectorContainer<T>::_get();
    }

    template<class T>
    std::vector<T>& get()
    {
        return this->VariadicVectorContainer<T>::_get();
    }

    template<class T>
    void push_back(const T& elem)
    {
        return this->VariadicVectorContainer<T>::_push_back(elem);
    }

    template<class T>
    void push_back(T&& elem)
    {
        return this->VariadicVectorContainer<T>::_push_back(std::move(elem));
    }

    template<class T, class ...Args>
    T& emplace_back(Args&&... args)
    {
        return this->VariadicVectorContainer<T>::_emplace_back(std::forward<Args>(args)...);
    }

    template<class T>
    void resize(int size)
    {
        return this->VariadicVectorContainer<T>::_resize(size);
    }

    template<class T>
    void reserve(int size)
    {
        return this->VariadicVectorContainer<T>::_reserve(size);
    }

    template<class T>
    T& set(int index, const T& elem)
    {
        return this->VariadicVectorContainer<T>::_set(index, elem);
    }

    template<class T>
    T& set(int index, T&& elem)
    {
        return this->VariadicVectorContainer<T>::_set(index, std::move(elem));
    }

    template<class T>
    void clear()
    {
        return this->VariadicVectorContainer<T>::_clear();
    }

    // visit all elements in one vector
    template<typename T, typename Visitor>
    void for_each_element(Visitor&& visitor) const
    {
        _visit_elements<T>(std::forward<Visitor>(visitor));
    }

    template<typename T, typename Visitor>
    void for_each_element(Visitor&& visitor)
    {
        _visit_elements<T>(std::forward<Visitor>(visitor));
    }

    // visit all elements across all vectors
    template<typename Visitor>
    void for_each_element(Visitor&& visitor) const
    {
        _visit_elements<L, R...>(std::forward<Visitor>(visitor));
    }

    template<typename Visitor>
    void for_each_element(Visitor&& visitor)
    {
        _visit_elements<L, R...>(std::forward<Visitor>(visitor));
    }

    private:
    // recursive implementation to visit elements of all types
    template<typename T, typename... Ts, typename Visitor>
    void _visit_elements(Visitor&& visitor) const
    {
        this->VariadicVectorContainer<T>::_for_each_element(visitor);

        if constexpr (sizeof...(Ts) > 0)
        {
            _visit_elements<Ts...>(std::forward<Visitor>(visitor));
        }
    }

    template<typename T, typename... Ts, typename Visitor>
    void _visit_elements(Visitor&& visitor)
    {
        this->VariadicVectorContainer<T>::_for_each_element(visitor);

        if constexpr (sizeof...(Ts) > 0)
        {
            _visit_elements<Ts...>(std::forward<Visitor>(visitor));
        }
    }
};

#endif // __VARIADIC_VECTOR_CONTAINER_HPP
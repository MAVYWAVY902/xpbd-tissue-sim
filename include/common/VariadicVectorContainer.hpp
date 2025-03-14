#ifndef __VARIADIC_VECTOR_CONTAINER_HPP
#define __VARIADIC_VECTOR_CONTAINER_HPP

#include <vector>

// adapted from this StackOverflow answer: https://stackoverflow.com/a/53112843
template<class L, class... R> class VariadicVectorContainer;

template<class... Types> class VariadicIterator;

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

    void _push_back(const L& elem)
    {
        _vec.push_back(elem);
    }

    void _push_back(L&& elem)
    {
        _vec.push_back(std::move(elem));
    }

    void _set(int index, const L& elem)
    {
        _vec[index] = elem;
    }

    void _set(int index, L&& elem)
    {
        _vec[index] = std::move(elem);
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

    template<class T>
    void resize(int size)
    {
        return this->VariadicVectorContainer<T>::_resize(size);
    }

    template<class T>
    void set(int index, const T& elem)
    {
        return this->VariadicVectorContainer<T>::_set(index, elem);
    }

    template<class T>
    void set(int index, T&& elem)
    {
        return this->VariadicVectorContainer<T>::_set(index, std::move(elem));
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

    // Extension to VariadicVectorContainer for range-based for loop support
    public:
    // Iterator related methods
    VariadicIterator<L, R...> begin() {
        return VariadicIterator<L, R...>(*this);
    }
    
    VariadicIterator<L, R...> end() {
        return VariadicIterator<L, R...>(*this, true);
    }
    
    VariadicIterator<L, R...> begin() const {
        return VariadicIterator<L, R...>(*const_cast<VariadicVectorContainer*>(this));
    }
    
    VariadicIterator<L, R...> end() const {
        return VariadicIterator<L, R...>(*const_cast<VariadicVectorContainer*>(this), true);
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

// Iterator over different types in VariadicVectorContainer
template<class... Types>
class VariadicIterator
{
    public:
    using ContainerType = VariadicVectorContainer<Types...>;

    VariadicIterator(ContainerType& container, bool is_end = false)
        : _container(&container), _current_type(is_end ? sizeof...(Types) : 0), _current_index(0)
    {
        if (!is_end)
        {
            _skip_empty_vectors();
        }
    }

    template<typename Visitor>
    void visit(Visitor&& visitor) const
    {
        _visit_current<0, Visitor, Types...>(std::forward<Visitor>(visitor));
    }

    VariadicIterator& operator++()
    {
        _current_index++;
        _check_and_advance();
        return *this;
    }

    bool operator==(const VariadicIterator& other) const
    {
        return _container == other._container && 
               _current_type == other._current_type && 
               (_current_type == sizeof...(Types) || _current_index == other._current_index);
    }

    bool operator !=(const VariadicIterator& other) const
    {
        return !(*this == other);
    }

    private:
    ContainerType* _container;
    size_t _current_type;
    size_t _current_index;

    // cehck if we need to move to the next vector type
    void _check_and_advance()
    {
        if (_current_type < sizeof...(Types))
        {
            size_t size = _get_size_of_current_vector();
            if (_current_index >= size)
            {
                _current_type++;
                _current_index = 0;
                _skip_empty_vectors();
            }
        }
    }

    // skip empty vectors
    void _skip_empty_vectors()
    {
        while (_current_type < sizeof...(Types) && _get_size_of_current_vector() == 0)
        {
            _current_type++;
        }
    }

    // get size of current vector
    size_t _get_size_of_current_vector() const
    {
        size_t result = 0;
        _get_size<0, Types...>(result);
        return result;
    }

    template<size_t I, typename T, typename... Ts>
    void _get_size(size_t& result) const
    {
        if (I == _current_type)
        {
            result = _container->template get<T>().size();
        }
        else if constexpr (sizeof...(Ts) > 0)
        {
            _get_size<I+1, Ts...>(result);
        }
    }

    // visit the current element based on the current type
    template<size_t I, typename Visitor, typename T, typename... Ts>
    void _visit_current(Visitor&& visitor) const 
    {
        if (I == _current_type)
        {
            visitor(_container->template get<T>()[_current_index]);
        }
        else if constexpr (sizeof...(Ts) > 0)
        {
            _visit_current<I+1, Visitor, Ts...>(std::forward<Visitor>(visitor));
        }
    }
};

#endif // __VARIADIC_VECTOR_CONTAINER_HPP
#ifndef __TYPE_LIST_HPP
#define __TYPE_LIST_HPP

template<typename ...Types>
struct TypeList
{
};

// Helper type trait to concatenate TypeLists
template<typename... TypeLists>
struct ConcatenateTypeLists;

// Base case: when no TypeLists are provided
template<>
struct ConcatenateTypeLists<> 
{
    using type = TypeList<>;
};

// Base case: single TypeList
template<typename... Types>
struct ConcatenateTypeLists<TypeList<Types...>> 
{
    using type = TypeList<Types...>;
};

// Recursive case: concatenate multiple TypeLists
template<typename... Types1, typename... Types2, typename... RestLists>
struct ConcatenateTypeLists<TypeList<Types1...>, TypeList<Types2...>, RestLists...> 
{
    using type = typename ConcatenateTypeLists<TypeList<Types1..., Types2...>, RestLists...>::type;
};

// Constexpr function wrapper to make it more user-friendly
template<typename... TypeLists>
constexpr auto concatenate_type_lists_v = typename ConcatenateTypeLists<TypeLists...>::type{};

#endif // __TYPE_LIST_HPP
#ifndef __TYPE_LIST_HPP
#define __TYPE_LIST_HPP

/** Empty struct that stores a parameter pack. Very useful for template metaprogramming. */
template<typename ...Types>
struct TypeList
{
};

/////////////////////////////////////////////////////////////////////////////////////////
// Concatenating TypeLists
//////////////////////////////////////////////////////////////////////////////////////////
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



///////////////////////////////////////////////////////////////////////////
// Checking if TypeList contains a given type
///////////////////////////////////////////////////////////////////////////
template<typename T, typename List>
struct TypeListContains;

template<typename T, typename... Types>
struct TypeListContains<T, TypeList<Types...>>
{
    static constexpr bool value = (std::is_same_v<T, Types> || ...);
};

// helper metafunction
template <typename T, typename List>
constexpr bool type_list_contains_v = TypeListContains<T, List>::value;



#endif // __TYPE_LIST_HPP
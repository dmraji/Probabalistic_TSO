// Simple & concise multi-dim std::arrays

#ifndef multi_dim_array_hh
#define multi_dim_array_hh

#include <cstddef>
#include <array>

template <class T, std::size_t I, std::size_t... J>
struct MultiDimArray
{
  using Nested = typename MultiDimArray<T, J...>::type;
  // typedef typename MultiDimArray<T, J...>::type Nested;
  using type = std::array<Nested, I>;
  // typedef std::array<Nested, I> type;
};

template <class T, std::size_t I>
struct MultiDimArray<T, I>
{
  using type = std::array<T, I>;
  // typedef std::array<T, I> type;
};

#endif

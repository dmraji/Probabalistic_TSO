// Simple & concise multi-dim std::arrays

#ifndef multi_dim_array_hh
#define multi_dim_array_hh

#include <array>

template <class T, std::size_t I, std::size_t... J>
struct md_array
{
  using nested = typename md_array<T, J...>::type;
  // typedef typename MultiDimArray<T, J...>::type Nested;
  using type = std::array<nested, I>;
  // typedef std::array<Nested, I> type;
};

template <class T, std::size_t I>
struct md_array<T, I>
{
  using type = std::array<T, I>;
  // typedef std::array<T, I> type;
};

#endif

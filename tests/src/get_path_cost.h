#pragma once

#include <utility>
#include <numeric>

template <typename Iterator, typename NeighborWeightFn,
		typename ValueType = decltype(std::declval<NeighborWeightFn>()(*std::declval<Iterator>(), *std::declval<Iterator>()))>
	ValueType get_path_cost(Iterator begin, Iterator end, NeighborWeightFn weight_fn)
{
	return std::inner_product(begin, std::prev(end),
		std::next(begin),
		ValueType(0), std::plus<>(), weight_fn);
}
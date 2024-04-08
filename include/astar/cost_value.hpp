#pragma once

#include <utility>

namespace cds
{
	namespace astar
	{
		template <typename CostFn, typename NodeType>
		using cost_value_t = decltype(std::declval<CostFn>()(std::declval<NodeType>()));
	}
}
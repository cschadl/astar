// Copyright (C) 2018 by Christopher Schadl <cschadl@gmail.com>

// Permission to use, copy, modify, and/or distribute this software for any purpose
// with or without fee is hereby granted.

// THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD 
// TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS.
// IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
// DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
// WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
// ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

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
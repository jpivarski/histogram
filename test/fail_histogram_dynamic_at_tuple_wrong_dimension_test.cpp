// Copyright 2018 Hans Dembinski
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt
// or copy at http://www.boost.org/LICENSE_1_0.txt)

#include <boost/histogram.hpp>
#include <utility>

using namespace boost::histogram;
int main() {
  auto h = make_dynamic_histogram(axis::integer<>(0, 2));
  h.at(std::make_pair(0, 0));
}

// Copyright 2015-2017 Hans Dembinski
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt
// or copy at http://www.boost.org/LICENSE_1_0.txt)

#ifndef _BOOST_HISTOGRAM_HISTOGRAM_DYNAMIC_IMPL_HPP_
#define _BOOST_HISTOGRAM_HISTOGRAM_DYNAMIC_IMPL_HPP_

#include <algorithm>
#include <boost/assert.hpp>
#include <boost/config.hpp>
#include <boost/histogram/arithmetic_operators.hpp>
#include <boost/histogram/axis/any.hpp>
#include <boost/histogram/axis/types.hpp>
#include <boost/histogram/detail/axis_visitor.hpp>
#include <boost/histogram/detail/cat.hpp>
#include <boost/histogram/detail/meta.hpp>
#include <boost/histogram/detail/utility.hpp>
#include <boost/histogram/histogram_fwd.hpp>
#include <boost/histogram/iterator.hpp>
#include <boost/histogram/storage/array_storage.hpp>
#include <boost/histogram/storage/operators.hpp>
#include <boost/histogram/storage/weight_counter.hpp>
#include <boost/mp11.hpp>
#include <boost/type_index.hpp>
#include <cstddef>
#include <iterator>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

// forward declaration for serialization
namespace boost {
namespace serialization {
class access;
}
} // namespace boost

// forward declaration for python
namespace boost {
namespace python {
class access;
}
} // namespace boost

namespace boost {
namespace histogram {

template <typename Axes, typename Storage>
class histogram<dynamic_tag, Axes, Storage> {
  static_assert(mp11::mp_size<Axes>::value > 0, "at least one axis required");

public:
  using any_axis_type = mp11::mp_rename<Axes, axis::any>;
  using axes_type = std::vector<any_axis_type>;
  using element_type = typename Storage::element_type;
  using const_reference = typename Storage::const_reference;
  using const_iterator = iterator_over<histogram, Storage>;
  using iterator = const_iterator;

public:
  histogram() = default;
  histogram(const histogram &) = default;
  histogram(histogram &&) = default;
  histogram &operator=(const histogram &) = default;
  histogram &operator=(histogram &&) = default;

  template <typename Axis0, typename... Axis,
            typename = detail::requires_axis<Axis0>>
  explicit histogram(Axis0 &&axis0, Axis &&... axis)
      : axes_({std::forward<Axis0>(axis0), std::forward<Axis>(axis)...}) {
    storage_ = Storage(size_from_axes());
  }

  template <typename Iterator, typename = detail::requires_iterator<Iterator>>
  histogram(Iterator begin, Iterator end) : axes_(std::distance(begin, end)) {
    std::copy(begin, end, axes_.begin());
    storage_ = Storage(size_from_axes());
  }

  template <typename T, typename A, typename S>
  explicit histogram(const histogram<T, A, S> &rhs) : storage_(rhs.storage_) {
    detail::axes_assign(axes_, rhs.axes_);
  }

  template <typename T, typename A, typename S>
  histogram &operator=(const histogram<T, A, S> &rhs) {
    if (static_cast<const void *>(this) != static_cast<const void *>(&rhs)) {
      detail::axes_assign(axes_, rhs.axes_);
      storage_ = rhs.storage_;
    }
    return *this;
  }

  template <typename S>
  explicit histogram(dynamic_histogram<Axes, S> &&rhs)
      : axes_(std::move(rhs.axes_)), storage_(std::move(rhs.storage_)) {}

  template <typename S> histogram &operator=(dynamic_histogram<Axes, S> &&rhs) {
    if (static_cast<const void *>(this) != static_cast<const void *>(&rhs)) {
      axes_ = std::move(rhs.axes_);
      storage_ = std::move(rhs.storage_);
    }
    return *this;
  }

  template <typename T, typename A, typename S>
  bool operator==(const histogram<T, A, S> &rhs) const noexcept {
    return detail::axes_equal(axes_, rhs.axes_) && storage_ == rhs.storage_;
  }

  template <typename T, typename A, typename S>
  bool operator!=(const histogram<T, A, S> &rhs) const noexcept {
    return !operator==(rhs);
  }

  template <typename T, typename A, typename S>
  histogram &operator+=(const histogram<T, A, S> &rhs) {
    if (!detail::axes_equal(axes_, rhs.axes_))
      throw std::invalid_argument("axes of histograms differ");
    storage_ += rhs.storage_;
    return *this;
  }

  template <typename T> histogram &operator*=(const T &rhs) {
    storage_ *= rhs;
    return *this;
  }

  template <typename T> histogram &operator/=(const T &rhs) {
    storage_ *= 1.0 / rhs;
    return *this;
  }

  template <typename... Ts> void operator()(const Ts &... ts) {
    // case with one argument is ambiguous, is specialized below
    BOOST_ASSERT_MSG(dim() == sizeof...(Ts),
                     "fill arguments does not match histogram dimension "
                     "(did you use weight() in the wrong place?)");
    detail::xlin_recursive<axes_type> x(axes_, ts...);
    if (x.stride) {
      detail::fill_storage(storage_, x.idx);
    }
  }

  template <typename T> void operator()(const T &t) {
    // check whether T is unpackable
    if (dim() == 1) {
      fill_impl(detail::no_container_tag(), t);
    } else {
      fill_impl(detail::classify_container<T>(), t);
    }
  }

  template <typename W, typename... Ts>
  void operator()(const detail::weight<W> &w, const Ts &... ts) {
    // case with one argument is ambiguous, is specialized below
    BOOST_ASSERT_MSG(dim() == sizeof...(Ts),
                     "fill arguments does not match histogram dimension");
    detail::xlin_recursive<axes_type> x(axes_, ts...);
    if (x.stride) {
      detail::fill_storage(storage_, x.idx, w);
    }
  }

  template <typename W, typename T>
  void operator()(const detail::weight<W> &w, const T &t) {
    // check whether T is unpackable
    if (dim() == 1) {
      fill_impl(detail::no_container_tag(), t, w);
    } else {
      fill_impl(detail::classify_container<T>(), t, w);
    }
  }

  template <typename... Ts> const_reference at(const Ts &... ts) const {
    // case with one argument is ambiguous, is specialized below
    BOOST_ASSERT_MSG(dim() == sizeof...(Ts),
                     "bin arguments does not match histogram dimension");
    detail::lin_recursive<axes_type> x(axes_, static_cast<int>(ts)...);
    return detail::storage_get(storage_, x.idx, x.stride == 0);
  }

  template <typename T> const_reference at(const T &t) const {
    // check whether T is unpackable
    return at_impl(detail::classify_container<T>(), t);
  }

  template <typename T> const_reference operator[](const T &t) const {
    // check whether T is unpackable
    return at_impl(detail::classify_container<T>(), t);
  }

  /// Number of axes (dimensions) of histogram
  unsigned dim() const noexcept { return axes_.size(); }

  /// Total number of bins in the histogram (including underflow/overflow)
  std::size_t size() const noexcept { return storage_.size(); }

  /// Reset bin counters to zero
  void reset() { storage_ = Storage(size_from_axes()); }

  /// Return axis \a i
  any_axis_type &axis(unsigned i = 0) {
    BOOST_ASSERT_MSG(i < dim(), "axis index out of range");
    return axes_[i];
  }

  /// Return axis \a i (const version)
  const any_axis_type &axis(unsigned i = 0) const {
    BOOST_ASSERT_MSG(i < dim(), "axis index out of range");
    return axes_[i];
  }

  /// Apply unary functor/function to each axis
  template <typename Unary> void for_each_axis(Unary &&unary) const {
    for (const auto &a : axes_) {
      apply_visitor(detail::unary_visitor<Unary>(unary), a);
    }
  }

  /// Return a lower dimensional histogram
  template <int N, typename... Ts>
  histogram reduce_to(mp11::mp_int<N>, Ts...) const {
    const auto b = detail::bool_mask<mp11::mp_int<N>, Ts...>(dim(), true);
    return reduce_impl(b);
  }

  /// Return a lower dimensional histogram
  template <typename... Ts> histogram reduce_to(int n, Ts... ts) const {
    std::vector<bool> b(dim(), false);
    for (const auto &i : {n, int(ts)...})
      b[i] = true;
    return reduce_impl(b);
  }

  /// Return a lower dimensional histogram
  template <typename Iterator, typename = detail::requires_iterator<Iterator>>
  histogram reduce_to(Iterator begin, Iterator end) const {
    std::vector<bool> b(dim(), false);
    for (; begin != end; ++begin)
      b[*begin] = true;
    return reduce_impl(b);
  }

  const_iterator begin() const noexcept {
    return const_iterator(*this, storage_, 0);
  }

  const_iterator end() const noexcept {
    return const_iterator(*this, storage_, storage_.size());
  }

private:
  axes_type axes_;
  Storage storage_;

  std::size_t size_from_axes() const noexcept {
    detail::field_count_visitor v;
    for_each_axis(v);
    return v.value;
  }

  template <typename T, typename... Ts>
  void fill_impl(detail::dynamic_container_tag, const T &t, const Ts &... ts) {
    BOOST_ASSERT_MSG(dim() == std::distance(std::begin(t), std::end(t)),
                     "fill container does not match histogram dimension");
    detail::xlin_dynamic<axes_type, decltype(std::begin(t))> x(axes_, std::begin(t));
    if (x.stride) {
      detail::fill_storage(storage_, x.idx, ts...);
    }
  }

  template <typename T, typename... Ts>
  void fill_impl(detail::static_container_tag, const T &t, const Ts &... ts) {
    BOOST_ASSERT_MSG(dim() == mp11::mp_size<T>::value,
                     "fill container does not match histogram dimension");
    detail::xlin_static<axes_type, T> x(axes_, t);
    if (x.stride) {
      detail::fill_storage(storage_, x.idx, ts...);
    }
  }

  template <typename T, typename... Ts>
  void fill_impl(detail::no_container_tag, const T &t, const Ts &... ts) {
    BOOST_ASSERT_MSG(dim() == 1,
                     "fill argument does not match histogram dimension");
    detail::xlin_recursive<axes_type> x(axes_, t);
    if (x.stride) {
      detail::fill_storage(storage_, x.idx, ts...);
    }
  }

  template <typename T>
  const_reference at_impl(detail::dynamic_container_tag, const T &t) const {
    BOOST_ASSERT_MSG(dim() == std::distance(std::begin(t), std::end(t)),
                     "bin container does not match histogram dimension");
    detail::lin_dynamic<axes_type, decltype(std::begin(t))> x(axes_, std::begin(t));
    return detail::storage_get(storage_, x.idx, x.stride == 0);
  }

  template <typename T>
  const_reference at_impl(detail::static_container_tag, const T &t) const {
    BOOST_ASSERT_MSG(dim() == mp11::mp_size<T>::value,
                     "bin container does not match histogram dimension");
    detail::lin_static<axes_type, T> x(axes_, t);
    return detail::storage_get(storage_, x.idx, x.stride == 0);
  }

  template <typename T>
  const_reference at_impl(detail::no_container_tag, const T &t) const {
    BOOST_ASSERT_MSG(dim() == 1,
                     "bin argument does not match histogram dimension");
    detail::lin_recursive<axes_type> x(axes_, detail::indirect_int_cast(t));
    return detail::storage_get(storage_, x.idx, x.stride == 0);
  }

  histogram reduce_impl(const std::vector<bool> &b) const {
    axes_type axes;
    std::vector<unsigned> n(b.size());
    auto axes_iter = axes_.begin();
    auto n_iter = n.begin();
    for (const auto &bi : b) {
      if (bi)
        axes.emplace_back(*axes_iter);
      *n_iter = axes_iter->shape();
      ++axes_iter;
      ++n_iter;
    }
    histogram h(axes.begin(), axes.end());
    detail::index_mapper m(n, b);
    do {
      h.storage_.add(m.second, storage_[m.first]);
    } while (m.next());
    return h;
  }

  template <typename T, typename A, typename S> friend class histogram;
  friend class ::boost::python::access;
  friend class ::boost::serialization::access;
  template <typename Archive> void serialize(Archive &, unsigned);
};

template <typename... Axis>
dynamic_histogram<
    mp11::mp_set_push_back<axis::types, detail::rm_cv_ref<Axis>...>>
make_dynamic_histogram(Axis &&... axis) {
  using H = dynamic_histogram<
      mp11::mp_set_push_back<axis::types, detail::rm_cv_ref<Axis>...>>;
  return H(std::forward<Axis>(axis)...);
}

template <typename Storage, typename... Axis>
dynamic_histogram<
    mp11::mp_set_push_back<axis::types, detail::rm_cv_ref<Axis>...>, Storage>
make_dynamic_histogram_with(Axis &&... axis) {
  using H = dynamic_histogram<
      mp11::mp_set_push_back<axis::types, detail::rm_cv_ref<Axis>...>, Storage>;
  return H(std::forward<Axis>(axis)...);
}

template <typename Iterator, typename = detail::requires_iterator<Iterator>>
dynamic_histogram<
    detail::mp_union<axis::types, typename Iterator::value_type::types>>
make_dynamic_histogram(Iterator begin, Iterator end) {
  using H = dynamic_histogram<
      detail::mp_union<axis::types, typename Iterator::value_type::types>>;
  return H(begin, end);
}

template <typename Storage, typename Iterator, typename = detail::requires_iterator<Iterator>>
dynamic_histogram<
    detail::mp_union<axis::types, typename Iterator::value_type::types>,
    Storage>
make_dynamic_histogram_with(Iterator begin, Iterator end) {
  using H = dynamic_histogram<
      detail::mp_union<axis::types, typename Iterator::value_type::types>,
      Storage>;
  return H(begin, end);
}

} // namespace histogram
} // namespace boost

#endif

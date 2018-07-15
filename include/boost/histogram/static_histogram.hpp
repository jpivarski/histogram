// Copyright 2015-2017 Hans Dembinski
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt
// or copy at http://www.boost.org/LICENSE_1_0.txt)

#ifndef _BOOST_HISTOGRAM_HISTOGRAM_IMPL_STATIC_HPP_
#define _BOOST_HISTOGRAM_HISTOGRAM_IMPL_STATIC_HPP_

#include <boost/assert.hpp>
#include <boost/histogram/arithmetic_operators.hpp>
#include <boost/histogram/axis/types.hpp>
#include <boost/histogram/detail/axis_visitor.hpp>
#include <boost/histogram/detail/meta.hpp>
#include <boost/histogram/detail/utility.hpp>
#include <boost/histogram/histogram_fwd.hpp>
#include <boost/histogram/iterator.hpp>
#include <boost/histogram/storage/operators.hpp>
#include <boost/histogram/storage/weight_counter.hpp>
#include <boost/mp11.hpp>
#include <tuple>
#include <type_traits>
#include <utility>

// forward declaration for serialization
namespace boost {
namespace serialization {
class access;
}
} // namespace boost

namespace boost {
namespace histogram {

template <typename Axes, typename Storage>
class histogram<static_tag, Axes, Storage> {
  using axes_size = mp11::mp_size<Axes>;
  static_assert(axes_size::value > 0, "at least one axis required");

public:
  using axes_type = mp11::mp_rename<Axes, std::tuple>;
  using element_type = typename Storage::element_type;
  using const_reference = typename Storage::const_reference;
  using const_iterator = iterator_over<histogram, Storage>;
  using iterator = const_iterator;

  histogram() = default;
  histogram(const histogram &rhs) = default;
  histogram(histogram &&rhs) = default;
  histogram &operator=(const histogram &rhs) = default;
  histogram &operator=(histogram &&rhs) = default;

  template <typename Axis0, typename... Axis,
            typename = detail::requires_axis<Axis0>>
  explicit histogram(Axis0 &&axis0, Axis &&... axis)
      : axes_(std::forward<Axis0>(axis0), std::forward<Axis>(axis)...) {
    storage_ = Storage(size_from_axes());
  }

  explicit histogram(axes_type &&axes) : axes_(std::move(axes)) {
    storage_ = Storage(size_from_axes());
  }

  template <typename S>
  explicit histogram(const static_histogram<Axes, S> &rhs)
      : axes_(rhs.axes_), storage_(rhs.storage_) {}

  template <typename S>
  histogram &operator=(const static_histogram<Axes, S> &rhs) {
    if (static_cast<const void *>(this) != static_cast<const void *>(&rhs)) {
      axes_ = rhs.axes_;
      storage_ = rhs.storage_;
    }
    return *this;
  }

  template <typename A, typename S>
  explicit histogram(const dynamic_histogram<A, S> &rhs)
      : storage_(rhs.storage_) {
    detail::axes_assign(axes_, rhs.axes_);
  }

  template <typename A, typename S>
  histogram &operator=(const dynamic_histogram<A, S> &rhs) {
    if (static_cast<const void *>(this) != static_cast<const void *>(&rhs)) {
      detail::axes_assign(axes_, rhs.axes_);
      storage_ = rhs.storage_;
    }
    return *this;
  }

  template <typename A, typename S>
  bool operator==(const static_histogram<A, S> &) const noexcept {
    return false;
  }

  template <typename S>
  bool operator==(const static_histogram<Axes, S> &rhs) const noexcept {
    return detail::axes_equal(axes_, rhs.axes_) && storage_ == rhs.storage_;
  }

  template <typename A, typename S>
  bool operator==(const dynamic_histogram<A, S> &rhs) const noexcept {
    return detail::axes_equal(axes_, rhs.axes_) && storage_ == rhs.storage_;
  }

  template <typename T, typename A, typename S>
  bool operator!=(const histogram<T, A, S> &rhs) const noexcept {
    return !operator==(rhs);
  }

  template <typename S>
  histogram &operator+=(const static_histogram<Axes, S> &rhs) {
    if (!detail::axes_equal(axes_, rhs.axes_))
      throw std::invalid_argument("axes of histograms differ");
    storage_ += rhs.storage_;
    return *this;
  }

  template <typename A, typename S>
  histogram &operator+=(const dynamic_histogram<A, S> &rhs) {
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
    static_assert(sizeof...(Ts) == axes_size::value,
                  "fill arguments do not match histogram dimension");
    detail::xlin_recursive<axes_type> x(axes_, ts...);
    if (x.stride)
      detail::fill_storage(storage_, x.idx);
  }

  template <typename T> void operator()(const T &t) {
    // check whether we need to unpack argument
    fill_impl(mp11::mp_if_c<(axes_size::value == 1), detail::no_container_tag,
                            detail::classify_container<T>>(), t);
  }

  // TODO: merge this with unpacking
  template <typename W, typename... Ts>
  void operator()(const detail::weight<W> &w, const Ts &... ts) {
    // case with one argument is ambiguous, is specialized below
    detail::xlin_recursive<axes_type> x(axes_, ts...);
    if (x.stride)
      detail::fill_storage(storage_, x.idx, w);
  }

  // TODO: remove as obsolete
  template <typename W, typename T>
  void operator()(const detail::weight<W> &w, const T &t) {
    // check whether we need to unpack argument
    fill_impl(mp11::mp_if_c<(axes_size::value == 1), detail::no_container_tag,
                            detail::classify_container<T>>(),
              t, w);
  }

  template <typename... Ts> const_reference at(const Ts &... ts) const {
    // case with one argument is ambiguous, is specialized below
    static_assert(sizeof...(ts) == axes_size::value,
                  "bin arguments do not match histogram dimension");
    detail::lin_recursive<axes_type> x(axes_, static_cast<int>(ts)...);
    return detail::storage_get(storage_, x.idx, x.stride == 0);
  }

  template <typename T> const_reference operator[](const T &t) const {
    // check whether we need to unpack argument
    return at_impl(detail::classify_container<T>(), t);
  }

  template <typename T> const_reference at(const T &t) const {
    // check whether we need to unpack argument
    return at_impl(detail::classify_container<T>(), t);
  }

  /// Number of axes (dimensions) of histogram
  constexpr unsigned dim() const noexcept { return axes_size::value; }

  /// Total number of bins in the histogram (including underflow/overflow)
  std::size_t size() const noexcept { return storage_.size(); }

  /// Reset bin counters to zero
  void reset() { storage_ = Storage(size_from_axes()); }

  /// Get N-th axis (const version)
  template <int N>
  typename std::add_const<typename std::tuple_element<N, axes_type>::type>::type
      &axis(mp11::mp_int<N>) const {
    static_assert(N < axes_size::value, "axis index out of range");
    return std::get<N>(axes_);
  }

  /// Get N-th axis
  template <int N>
  typename std::tuple_element<N, axes_type>::type &axis(mp11::mp_int<N>) {
    static_assert(N < axes_size::value, "axis index out of range");
    return std::get<N>(axes_);
  }

  // Get first axis (convenience for 1-d histograms, const version)
  constexpr typename std::add_const<
      typename std::tuple_element<0, axes_type>::type>::type &
  axis() const {
    return std::get<0>(axes_);
  }

  // Get first axis (convenience for 1-d histograms)
  typename std::tuple_element<0, axes_type>::type &axis() {
    return std::get<0>(axes_);
  }

  /// Apply unary functor/function to each axis
  template <typename Unary> void for_each_axis(Unary &&unary) const {
    mp11::tuple_for_each(axes_, std::forward<Unary>(unary));
  }

  /// Returns a lower-dimensional histogram
  template <int N, typename... Ns>
  auto reduce_to(mp11::mp_int<N>, Ns...) const
      -> static_histogram<detail::selection<Axes, mp11::mp_int<N>, Ns...>,
                          Storage> {
    using HR = static_histogram<detail::selection<Axes, mp11::mp_int<N>, Ns...>,
                                Storage>;
    auto hr =
        HR(detail::make_sub_tuple<axes_type, mp11::mp_int<N>, Ns...>(axes_));
    const auto b = detail::bool_mask<mp11::mp_int<N>, Ns...>(dim(), true);
    reduce_impl(hr, b);
    return hr;
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
    detail::field_count_visitor fc;
    for_each_axis(fc);
    return fc.value;
  }

  template <typename T, typename... Ts>
  void fill_impl(detail::dynamic_container_tag, const T &t, const Ts &... ts) {
    BOOST_ASSERT_MSG(std::distance(std::begin(t), std::end(t)) ==
                         axes_size::value,
                     "fill container does not match histogram dimension");
    detail::xlin_dynamic<axes_type, decltype(std::begin(t))> x(axes_, std::begin(t));
    if (x.stride) {
      detail::fill_storage(storage_, x.idx, ts...);
    }
  }

  template <typename T, typename... Ts>
  void fill_impl(detail::static_container_tag, const T &t, const Ts &... ts) {
    static_assert(detail::mp_size<T>::value == axes_size::value,
                  "fill container does not match histogram dimension");
    detail::xlin_static<axes_type, T> x(axes_, t);
    if (x.stride) {
      detail::fill_storage(storage_, x.idx, ts...);
    }
  }

  template <typename T, typename... Ts>
  void fill_impl(detail::no_container_tag, const T &t, const Ts &... ts) {
    static_assert(axes_size::value == 1,
                  "fill argument does not match histogram dimension");
    detail::xlin_recursive<axes_type> x(axes_, t);
    if (x.stride) {
      detail::fill_storage(storage_, x.idx, ts...);
    }
  }

  template <typename T>
  const_reference at_impl(detail::dynamic_container_tag, const T &t) const {
    BOOST_ASSERT_MSG(std::distance(std::begin(t), std::end(t)) ==
                         axes_size::value,
                     "bin container does not match histogram dimension");
    detail::lin_dynamic<axes_type, decltype(std::begin(t))> x(axes_, std::begin(t));
    return detail::storage_get(storage_, x.idx, x.stride == 0);
  }

  template <typename T>
  const_reference at_impl(detail::static_container_tag, const T &t) const {
    static_assert(mp11::mp_size<T>::value == axes_size::value,
                  "bin container does not match histogram dimension");
    detail::lin_static<axes_type, T> x(axes_, t);
    return detail::storage_get(storage_, x.idx, x.stride == 0);
  }

  template <typename T>
  const_reference at_impl(detail::no_container_tag, const T &t) const {
    detail::lin_recursive<axes_type> x(axes_, detail::indirect_int_cast(t));
    return detail::storage_get(storage_, x.idx, x.stride == 0);
  }

  template <typename H>
  void reduce_impl(H &h, const std::vector<bool> &b) const {
    detail::shape_vector_visitor v(dim());
    for_each_axis(v);
    detail::index_mapper m(v.shape, b);
    do {
      h.storage_.add(m.second, storage_[m.first]);
    } while (m.next());
  }

  template <typename T, typename A, typename S> friend class histogram;
  friend class ::boost::serialization::access;
  template <typename Archive> void serialize(Archive &, unsigned);
};

/// default static type factory
template <typename... Axis>
static_histogram<mp11::mp_list<Axis...>>
make_static_histogram(Axis &&... axis) {
  using H = static_histogram<mp11::mp_list<Axis...>>;
  return H(std::forward<Axis>(axis)...);
}

/// static type factory with variable storage type
template <typename Storage, typename... Axis>
static_histogram<mp11::mp_list<Axis...>, Storage>
make_static_histogram_with(Axis &&... axis) {
  using H = static_histogram<mp11::mp_list<Axis...>, Storage>;
  return H(std::forward<Axis>(axis)...);
}

template <typename... Axis>
static_histogram<mp11::mp_list<Axis...>, array_storage<weight_counter<double>>>
make_static_weighted_histogram(Axis &&... axis) {
  using H = static_histogram<mp11::mp_list<Axis...>,
                             array_storage<weight_counter<double>>>;
  return H(std::forward<Axis>(axis)...);
}

} // namespace histogram
} // namespace boost

#endif

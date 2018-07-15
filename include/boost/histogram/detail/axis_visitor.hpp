// Copyright 2015-2017 Hans Demsizeki
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt
// or copy at http://www.boost.org/LICENSE_1_0.txt)

#ifndef _BOOST_HISTOGARM_AXIS_VISITOR_HPP_
#define _BOOST_HISTOGARM_AXIS_VISITOR_HPP_

#include <boost/histogram/axis/any.hpp>
#include <boost/histogram/detail/meta.hpp>
#include <boost/histogram/detail/utility.hpp>
#include <boost/histogram/detail/cat.hpp>
#include <boost/type_index.hpp>
#include <boost/mp11.hpp>
#include <tuple>
#include <type_traits>
#include <vector>
#include <stdexcept>

namespace boost {
namespace histogram {
namespace detail {

namespace {

template <typename Tuple, typename VecVar> struct axes_equal_tuple_vecvar {
  bool &equal;
  const Tuple &t;
  const VecVar &v;
  axes_equal_tuple_vecvar(bool &eq, const Tuple &tt, const VecVar &vv)
      : equal(eq), t(tt), v(vv) {}
  template <typename Int> void operator()(Int) const {
    using T = mp11::mp_at<Tuple, Int>;
    auto tp = ::boost::get<T>(&v[Int::value]);
    equal &= (tp && *tp == std::get<Int::value>(t));
  }
};

template <typename Tuple, typename VecVar> struct axes_assign_tuple_vecvar {
  Tuple &t;
  const VecVar &v;
  axes_assign_tuple_vecvar(Tuple &tt, const VecVar &vv) : t(tt), v(vv) {}
  template <typename Int> void operator()(Int) const {
    using T = mp11::mp_at<Tuple, Int>;
    std::get<Int::value>(t) = ::boost::get<T>(v[Int::value]);
  }
};

template <typename VecVar, typename Tuple> struct axes_assign_vecvar_tuple {
  VecVar &v;
  const Tuple &t;
  axes_assign_vecvar_tuple(VecVar &vv, const Tuple &tt) : v(vv), t(tt) {}
  template <typename Int> void operator()(Int) const {
    v[Int::value] = std::get<Int::value>(t);
  }
};

template <typename... Ts>
bool axes_equal_impl(mp11::mp_true, const std::tuple<Ts...> &t,
                     const std::tuple<Ts...> &u) {
  return t == u;
}

template <typename... Ts, typename... Us>
bool axes_equal_impl(mp11::mp_false, const std::tuple<Ts...> &,
                     const std::tuple<Us...> &) {
  return false;
}

} // namespace

template <typename... Ts, typename... Us>
bool axes_equal(const std::tuple<Ts...> &t, const std::tuple<Us...> &u) {
  return axes_equal_impl(
      mp11::mp_same<mp11::mp_list<Ts...>, mp11::mp_list<Us...>>(), t, u);
}

template <typename... Ts, typename... Us>
void axes_assign(std::tuple<Ts...> &t, const std::tuple<Us...> &u) {
  static_assert(std::is_same<mp11::mp_list<Ts...>, mp11::mp_list<Us...>>::value,
                "cannot assign incompatible axes");
  t = u;
}

template <typename... Ts, typename... Us>
bool axes_equal(const std::tuple<Ts...> &t,
                const std::vector<axis::any<Us...>> &u) {
  if (sizeof...(Ts) != u.size())
    return false;
  bool equal = true;
  auto fn =
      axes_equal_tuple_vecvar<std::tuple<Ts...>, std::vector<axis::any<Us...>>>(
          equal, t, u);
  mp11::mp_for_each<mp11::mp_iota_c<sizeof...(Ts)>>(fn);
  return equal;
}

template <typename... Ts, typename... Us>
void axes_assign(std::tuple<Ts...> &t, const std::vector<axis::any<Us...>> &u) {
  auto fn = axes_assign_tuple_vecvar<std::tuple<Ts...>,
                                     std::vector<axis::any<Us...>>>(t, u);
  mp11::mp_for_each<mp11::mp_iota_c<sizeof...(Ts)>>(fn);
}

template <typename... Ts, typename... Us>
bool axes_equal(const std::vector<axis::any<Ts...>> &t,
                const std::tuple<Us...> &u) {
  return axes_equal(u, t);
}

template <typename... Ts, typename... Us>
void axes_assign(std::vector<axis::any<Ts...>> &t, const std::tuple<Us...> &u) {
  t.resize(sizeof...(Us));
  auto fn = axes_assign_vecvar_tuple<std::vector<axis::any<Ts...>>,
                                     std::tuple<Us...>>(t, u);
  mp11::mp_for_each<mp11::mp_iota_c<sizeof...(Us)>>(fn);
}

template <typename... Ts, typename... Us>
bool axes_equal(const std::vector<axis::any<Ts...>> &t,
                const std::vector<axis::any<Us...>> &u) {
  if (t.size() != u.size())
    return false;
  for (std::size_t i = 0; i < t.size(); ++i) {
    if (t[i] != u[i])
      return false;
  }
  return true;
}

template <typename... Ts, typename... Us>
void axes_assign(std::vector<axis::any<Ts...>> &t,
                 const std::vector<axis::any<Us...>> &u) {
  for (std::size_t i = 0; i < t.size(); ++i) {
    t[i] = u[i];
  }
}

struct field_count_visitor : public static_visitor<void> {
  mutable std::size_t value = 1;
  template <typename T> void operator()(const T &t) const {
    value *= t.shape();
  }
};

template <typename Unary> struct unary_visitor : public static_visitor<void> {
  Unary &unary;
  unary_visitor(Unary &u) : unary(u) {}
  template <typename Axis> void operator()(const Axis &a) const { unary(a); }
};

struct lin_visitor : public static_visitor<void> {
  std::size_t &idx;
  std::size_t &stride;
  const int j;

  lin_visitor(std::size_t &i, std::size_t &s, const int x) noexcept
      : idx(i),
        stride(s),
        j(x) {}
  template <typename A> void operator()(const A &a) const noexcept {
    const auto a_size = a.size();
    const auto a_shape = a.shape();
    stride *= (-1 <= j && j <= a_size); // set stride to zero, if j is invalid
    lin(idx, stride, a_size, a_shape, j);
  }
};

template <typename V> struct xlin_visitor : public static_visitor<void> {
  std::size_t &idx;
  std::size_t &stride;
  const V &val;

  xlin_visitor(std::size_t &i, std::size_t &s, const V& x) noexcept
      : idx(i),
        stride(s),
        val(x) {}

  template <typename Axis> void operator()(const Axis &a) const {
    impl(std::is_convertible<V, typename Axis::value_type>(), a);
  }

  template <typename Axis> void impl(std::true_type, const Axis &a) const {
    const auto a_size = a.size();
    const auto a_shape = a.shape();
    const auto j = a.index(val);
    lin(idx, stride, a_size, a_shape, j);
  }

  template <typename Axis> void impl(std::false_type, const Axis &) const {
    throw std::invalid_argument(
        cat("fill argument not convertible to axis value type: ",
            boost::typeindex::type_id<Axis>().pretty_name(), ", ",
            boost::typeindex::type_id<V>().pretty_name()));
  }
};

struct shape_vector_visitor : public static_visitor<void> {
  mutable std::vector<unsigned> shape;
  mutable std::vector<unsigned>::iterator iter;
  shape_vector_visitor(unsigned dim) : shape(dim) {
    iter = shape.begin();
  }

  template <typename Axis> void operator()(const Axis &a) const {
    *iter++ = a.shape();
  }
};

template <int N, typename V, typename... Ts>
void apply(const std::tuple<Ts...>& t, V& v) {
  v(std::get<N>(t));
}

template <int N, typename V, typename... Ts>
void apply(const std::vector<axis::any<Ts...>>& t, V& v) {
  boost::apply_visitor(v, t[N]);
}

template <typename Axes>
struct xlin_recursive {
  const Axes& axes;
  std::size_t idx = 0;
  std::size_t stride = 1;

  template <typename... Ts>
  xlin_recursive(const Axes& a, const Ts&... ts) : axes(a) {
    iterate(mp11::mp_int<0>(), ts...);
  }

  template <int N>
  void iterate(mp11::mp_int<N>) {}

  template <int N, typename T, typename... Ts>
  void iterate(mp11::mp_int<N>, const T& t, const Ts&... ts) {
    xlin_visitor<T> v{idx, stride, t};
    apply<N>(axes, v);
    iterate(mp11::mp_int<N + 1>(), ts...);
  }
};

template <typename Axes>
struct lin_recursive {
  const Axes& axes;
  std::size_t idx = 0;
  std::size_t stride = 1;

  // pre-condition: there are all ints
  template <typename... Ts>
  lin_recursive(const Axes& a, Ts... ts) : axes(a) {
    iterate(mp11::mp_int<0>(), ts...);
  }

  template <int N>
  void iterate(mp11::mp_int<N>) {}

  template <int N, typename T, typename... Ts>
  void iterate(mp11::mp_int<N>, T t, Ts... ts) {
    lin_visitor v{idx, stride, t};
    apply<N>(axes, v);
    iterate(mp11::mp_int<N + 1>(), ts...);
  }
};

template <typename Axes, typename T>
struct xlin_static {
  const Axes& axes;
  const T& arg;
  std::size_t idx = 0;
  std::size_t stride = 1;

  xlin_static(const Axes& a, const T& t) : axes(a), arg(t) {
    mp11::mp_for_each<mp11::mp_iota<mp11::mp_size<T>>>(*this);
  }

  template <typename Int>
  void operator()(Int) {
    const auto& u = std::get<Int::value>(arg);
    xlin_visitor<decltype(u)> v{idx, stride, u};
    apply<Int::value>(axes, v);
  }
};

template <typename Axes, typename T>
struct lin_static {
  const Axes& axes;
  const T& arg;
  std::size_t idx = 0;
  std::size_t stride = 1;

  lin_static(const Axes& a, const T& t) : axes(a), arg(t) {
    mp11::mp_for_each<mp11::mp_iota<mp11::mp_size<T>>>(*this);
  }

  template <typename Int>
  void operator()(Int) {
    const auto& u = std::get<Int::value>(arg);
    lin_visitor v{idx, stride, static_cast<int>(u)};
    apply<Int::value>(axes, v);
  }
};

template <typename Axes, typename Iterator>
struct xlin_dynamic {
  const Axes& axes;
  Iterator iter;
  std::size_t idx = 0;
  std::size_t stride = 1;

  template <typename... Ts>
  xlin_dynamic(const std::tuple<Ts...>& a, Iterator i) : axes(a), iter(i) {
    mp11::mp_for_each<mp11::mp_iota<mp11::mp_size<Axes>>>(*this);
  }

  template <typename... Ts>
  xlin_dynamic(const std::vector<axis::any<Ts...>>& a, Iterator iter) : axes(a) {
    for (const auto a : axes) {
      const auto& u = *iter++;
      xlin_visitor<decltype(u)> v{idx, stride, u};
      boost::apply_visitor(v, a);
    }
  }

  template <typename Int>
  void operator()(Int) {
    const auto& u = *iter++;
    xlin_visitor<decltype(u)> v{idx, stride, u};
    apply<Int::value>(axes, v);
  }
};

template <typename Axes, typename Iterator>
struct lin_dynamic {
  const Axes& axes;
  Iterator iter;
  std::size_t idx = 0;
  std::size_t stride = 1;

  template <typename... Ts>
  lin_dynamic(const std::tuple<Ts...>& a, Iterator i) : axes(a), iter(i) {
    mp11::mp_for_each<mp11::mp_iota<mp11::mp_size<Axes>>>(*this);
  }

  template <typename... Ts>
  lin_dynamic(const std::vector<axis::any<Ts...>>& a, Iterator iter) : axes(a) {
    for (const auto& a : axes) {
      const auto& u = *iter++;
      lin_visitor v{idx, stride, static_cast<int>(u)};
      boost::apply_visitor(v, a);
    }
  }

  template <typename Int>
  void operator()(Int) {
    const auto& u = *iter++;
    lin_visitor v{idx, stride, static_cast<int>(u)};
    apply<Int::value>(axes, v);
  }
};

} // namespace detail
} // namespace histogram
} // namespace boost

#endif

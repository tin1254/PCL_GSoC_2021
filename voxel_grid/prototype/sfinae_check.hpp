// #include <iostream>
#include <boost/optional.hpp>
#include <type_traits>
#include <vector>

template <typename, typename>
class TransformFilter;

struct PointXYZ {};

namespace detail {
using std::begin;

template <typename C, typename PointT>
struct IsGridStruct {
 private:
  template <typename T>
  static constexpr auto hasSetUp(T*) ->
      typename std::is_same<decltype(std::declval<T>().setUp(
                                std::declval<TransformFilter<T, PointT>*>())),
                            bool>::type;
  template <typename>
  static constexpr std::false_type hasSetUp(...);

  template <typename T>
  static constexpr auto hasAddPoint(T*) -> typename std::is_same<
      decltype(std::declval<T>().addPointToGrid(std::declval<PointT&>())),
      void>::type;
  template <typename>
  static constexpr std::false_type hasAddPoint(...);

  template <typename T>
  static constexpr auto hasFilterBegin(T*) -> typename std::is_same<
      decltype(std::declval<T>().filterGrid(begin(std::declval<T&>()))),
      boost::optional<PointT>>::type;
  template <typename T>
  static constexpr auto hasFilterBegin(T*) -> typename std::is_same<
      decltype(std::declval<T>().filterGrid(std::declval<T>().begin())),
      boost::optional<PointT>>::type;
  template <typename>
  static constexpr std::false_type hasFilterBegin(...);

  template <typename T>
  static constexpr auto hasFilterEnd(T*) -> typename std::is_same<
      decltype(std::declval<T>().filterGrid(end(std::declval<T>()))),
      boost::optional<PointT>>::type;
  template <typename T>
  static constexpr auto hasFilterEnd(T*) -> typename std::is_same<
      decltype(std::declval<T>().filterGrid(std::declval<T>().end())),
      boost::optional<PointT>>::type;
  template <typename>
  static constexpr std::false_type hasFilterEnd(...);

  template <typename T>
  static constexpr auto hasSize(T*) ->
      typename std::is_same<decltype(std::declval<T>().size()),
                            std::size_t>::type;
  template <typename>
  static constexpr std::false_type hasSize(...);

  using has_set_up = decltype(hasSetUp<C>(0));
  using has_add_point = decltype(hasAddPoint<C>(0));
  using has_filter_begin = decltype(hasFilterBegin<C>(0));
  using has_filter_end = decltype(hasFilterEnd<C>(0));
  using has_size = decltype(hasSize<C>(0));

  static constexpr bool is_set_up_valid = has_set_up::value;
  static constexpr bool is_add_point_valid = has_add_point::value;
  static constexpr bool is_filter_valid =
      has_filter_begin::value && has_filter_end::value;
  static constexpr bool is_size_valid = has_size::value;

 public:
  static constexpr bool is_valid =
      is_set_up_valid && is_add_point_valid && is_filter_valid && is_size_valid;
};
}  // namespace detail

template <typename GridStruct>
auto begin(GridStruct& grid_struct) -> decltype(grid_struct.grid_.begin()) {
  return grid_struct.grid_.begin();
}

template <typename PointT>
struct MyGridStruct;

template <typename PointT>
typename MyGridStruct<PointT>::Grid::iterator begin(
    MyGridStruct<PointT>& grid_struct) {
  return grid_struct.grid_.begin();
}

template <typename PointT>
struct MyGridStruct {
  using Grid = std::vector<size_t>;

  bool setUp(TransformFilter<MyGridStruct, PointT>* f) { return true; }

  size_t size() { return grid_.size(); }

  //   Grid::iterator begin() { return grid_.begin(); }

  Grid::iterator end() { return grid_.end(); }

  void addPointToGrid(const PointT& pt) {}

  boost::optional<PointT> filterGrid(const Grid::iterator grid_it) {
    return boost::optional<PointT>();
  }

  Grid grid_;
};

template <typename GridStruct, typename PointT>
class TransformFilter {
  static_assert(detail::IsGridStruct<GridStruct, PointT>::is_valid,
                "GridStruct requirement is not satisfied");

 public:
  void applyFilter() {
    const bool res = grid_struct_.setUp(this);

    grid_struct_.addPointToGrid(PointT());

    const size_t s = grid_struct_.size();

    using std::begin;  // invoke ADL, just like ranged-for loop
    using std::end;
    auto it = begin(grid_struct_);
    auto end_it = end(grid_struct_);

    for (; it != end_it; ++it) {
      const boost::optional<PointT> opt = grid_struct_.filterGrid(it);
    }
  }

  GridStruct grid_struct_;
};

int main() {
  TransformFilter<MyGridStruct<PointXYZ>, PointXYZ> tf;
  tf.applyFilter();
  return 0;
}
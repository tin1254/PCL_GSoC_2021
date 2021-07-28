// #include <iostream>
#include <optional>
// #include <string>
#include <type_traits>
#include <vector>

template <typename, typename>
class TransformFilter;

namespace detail {
template <typename, typename>
struct IsGridStruct;

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
      decltype(std::declval<T>().filterGrid(
          std::declval<decltype(std::declval<T>().begin())>())),
      std::optional<PointT>>::type;
  template <typename>
  static constexpr std::false_type hasFilterBegin(...);

  template <typename T>
  static constexpr auto hasFilterEnd(T*) -> typename std::is_same<
      decltype(std::declval<T>().filterGrid(
          std::declval<decltype(std::declval<T>().end())>())),
      std::optional<PointT>>::type;
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

 public:
  static constexpr bool is_set_up_valid = has_set_up::value;
  static constexpr bool is_add_point_valid = has_add_point::value;
  static constexpr bool is_filter_valid =
      has_filter_begin::value && has_filter_end::value;
  static constexpr bool is_size_valid = has_size::value;

  // static constexpr bool is_valid = has_add_point::value && has_set_up::value
  // && has_filter::value; static constexpr std::string print(){
  //     return "testing " + is_set_up_valid;
  // }
  // static constexpr std::string status = print();
};
}  // namespace detail

struct PointXYZ {};

template <typename GridStruct, typename PointT>
class TransformFilter {
  using is_grid_struct = detail::IsGridStruct<GridStruct, PointXYZ>;
  static_assert(is_grid_struct::is_set_up_valid,
                "Required member is not satisfied: bool "
                "setUp(TransformFilter<MyGridStruct,PointT>*)");
  static_assert(
      is_grid_struct::is_add_point_valid,
      "Required member is not satisfied: void addPointToGrid(PointT& pt)");
  static_assert(is_grid_struct::is_filter_valid,
                "Required member is not satisfied: std::optional<PointXYZ> "
                "filterGrid(begin()) or std::optional<PointXYZ> "
                "filterGrid(end())");
  static_assert(is_grid_struct::is_size_valid,
                "Required member is not satisfied: std::size_t size()");

  void applyFilter() {
    const bool res = grid_struct_.setUp(this);

    grid_struct_.addPointToGrid(PointT());

    const size_t s = grid_struct_.size();

    for (auto it = grid_struct_.begin(); it != grid_struct_.end(); ++it) {
      const std::optional<PointT> opt =
          grid_struct_.filterGrid(grid_struct_.begin());
    }
  }

  GridStruct grid_struct_;
};

template <typename PointT>
struct MyGridStruct {
  using Grid = std::vector<size_t>;

  bool setUp(TransformFilter<MyGridStruct, PointT>* f) { return true; }

  size_t size() { return grid_.size(); }

  Grid::iterator begin() { return grid_.begin(); }

  Grid::iterator end() { return grid_.end(); }

  void addPointToGrid(const PointT& pt) {}

  std::optional<PointXYZ> filterGrid(const Grid::iterator grid_it) {
    return std::nullopt;
  }

  Grid grid_;
};

int main() {
  TransformFilter<MyGridStruct<PointXYZ>, PointXYZ> tf;
  return 0;
}
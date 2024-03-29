

#include <Eigen/Dense>
#include <boost/optional.hpp>
#include <iostream>
#include <unordered_map>
#include <vector>

// ================== Necessary stuff for running ==================

using Vec3f = Eigen::Vector3f;

class PointXYZ {
 public:
  PointXYZ(const float x = 0, const float y = 0, const float z = 0) {
    p << x, y, z;
  }
  PointXYZ(const Vec3f &p) : p(p) {}
  Vec3f p;
};

bool isXYZFinite(const PointXYZ &pt) { return true; }

template <typename PointT>
class PointCloudT {
 public:
  PointCloudT() {}

  void push_back(const PointT &p) { points.push_back(p); }
  void reserve(const size_t s) { (void)s; }
  auto begin() { return points.begin(); }
  auto end() { return points.end(); }

  // Defined in pcl::PointCloud
  using PointType = PointT;

  bool is_dense;
  std::vector<PointT> points;
};

template <typename PointT>
class Filter {
  using PointCloud = PointCloudT<PointXYZ>;

 public:
  Filter() {}

  void filter(PointCloud &output) { applyFilter(output); }
  const PointCloud &getInputCloud() const { return input_; }
  void setInputCloud(const PointCloud &input) { input_ = input; }

 protected:
  virtual void applyFilter(PointCloud &pc) = 0;
  PointCloud input_;
};

// ================== transform_filter.hpp ==================

#define GET_POINT_TYPE(GridStructT) typename GridStructT::PointCloud::PointType

template <typename GridStruct, typename PointT = GET_POINT_TYPE(GridStruct)>
class TransformFilter : public Filter<PointT> {
  using PointCloud = PointCloudT<PointXYZ>;

  TransformFilter(GridStruct grid_struct) {
    grid_struct_ = std::move(grid_struct);
  }

 protected:
  void applyFilter(PointCloud &output) override {
    if (!grid_struct_.setUp(this)) {
      output = this->input_;
      return;
    }

    for (const auto &pt : this->input_) {
      if (this->input_.is_dense || isXYZFinite(pt)) {
        grid_struct_.addPointToGrid(pt);
      }
    }

    // allocate enough space for points
    output.reserve(grid_struct_.size());
    for (auto it = grid_struct_.begin(); it != grid_struct_.end(); ++it) {
      const auto &res = grid_struct_.filterGrid(it);
      if (res) {
        output.push_back(res.value());
      }
    }
  }

  const GridStruct &getGridStruct() { return grid_struct_; }

  GridStruct grid_struct_;
};

// ================== grid_filter_base.hpp ==================

template <typename GridStruct, typename PointT = GET_POINT_TYPE(GridStruct)>
class GridFilterBase : public TransformFilter<GridStruct> {
  using PointCloud = PointCloudT<PointT>;

 public:
  GridFilterBase() : TransformFilter<GridStruct>(GridStruct()) {}

  // ====== common grid filter functions ======

  Eigen::Vector3f getLeafSize() const { return leaf_size_; }

  void getMinMax3D(const PointCloud &cloud, Vec3f &min_pt,
                   Vec3f &max_pt) const {
    // set initialize values
    for (const auto &pt : cloud.points) {
      min_pt = min_pt.cwiseMin(pt.p);
      max_pt = max_pt.cwiseMax(pt.p);
    }
  }

  void setMinimumPointsNumberPerVoxel(const size_t num) {
    min_points_per_voxel_ = num;
  }

  size_t getMinimumPointsNumberPerVoxel() const {
    return min_points_per_voxel_;
  }

  //  ============================================

 protected:
  Vec3f leaf_size_ = Eigen::Vector3f::Constant(0.05);
  size_t min_points_per_voxel_;
};

// ================== voxel_grid.hpp ==================

// structure of a single voxel
struct Voxel {
  Vec3f centroid = Vec3f::Zero();
  size_t num_pt = 0;
};

// structure and operations of the grid
template <typename PointT>
class VoxelStructT {
 public:
  // Require user to define them
  using PointCloud = PointCloudT<PointT>;
  using Grid = std::unordered_map<size_t, Voxel>;

  auto begin() { return grid_.begin(); }
  auto end() { return grid_.end(); }
  size_t size() { return grid_.size(); }

  bool setUp(const TransformFilter<VoxelStructT> *transform_filter) {
    const auto grid_filter =
        static_cast<const GridFilterBase<VoxelStructT> *>(transform_filter);

    Vec3f min_p, max_p;
    const auto &input = grid_filter->getInputCloud();
    grid_filter->getMinMax3D(input, min_p, max_p);

    inverse_leaf_size_ = 1 / grid_filter->getLeafSize().array();
    min_b_ = (min_p.array() * inverse_leaf_size_.array()).cast<int>();
    max_b_ = (max_p.array() * inverse_leaf_size_.array()).cast<int>();

    div_b_ = (max_b_ - min_b_).array() + 1;
    divb_mul_ << div_b_[0], div_b_[0], div_b_[1];

    // NOTE: we need to store the members variables of GridFilterBase which are
    // needed by the member functions in this class as member variables
    // e.g.
    min_points_per_voxel_ = grid_filter->getMinimumPointsNumberPerVoxel();
    // ...

    // we have 10 points in main
    leaf_layout_.resize(10, -1);

    return true;
  }

  size_t hashPoint(const PointT &pt) {
    const Vec3f tmp = pt.p.array() * inverse_leaf_size_.array();
    return divb_mul_.dot(tmp.cast<int>() - min_b_);
  }

  void addPointToGrid(const PointT &pt) {
    // can be batch/running mode
    const size_t h = hashPoint(pt);
    grid_[h].centroid += pt.p;
    grid_[h].num_pt++;
  }

  boost::optional<PointT> filterGrid(Grid::iterator grid_it) {
    const auto &voxel = grid_it->second;
    if (voxel.num_pt >= min_points_per_voxel_) {
      leaf_layout_[grid_it->first] = num_voxels_++;
      return PointT(voxel.centroid / voxel.num_pt);
    } else {
      leaf_layout_[grid_it->first] = -1;
      return boost::none;
    }
  }

  Eigen::Vector3i min_b_, max_b_, div_b_, divb_mul_;
  Vec3f inverse_leaf_size_;
  size_t min_points_per_voxel_;
  Grid grid_;
  size_t num_voxels_ = 0;
  std::vector<int> leaf_layout_;
};

// COMMENT: or directly declare this class as VoxelGrid
template <typename GridStruct, typename PointT>
class VoxelFilter : public GridFilterBase<GridStruct<PointT>> {
 public:
  VoxelFilter() {}

  //  ====== functions specific to VoxelGrid ======

  inline int getCentroidIndex(float x, float y, float z) const {
    return this->getGridStruct().hashPoint(PointT(x, y, z));
  }

  std::vector<int> getLeafLayout() const {
    return this->getGridStruct().leaf_layout_;
  }

  inline Eigen::Vector3i getGridCoordinates(float x, float y, float z) const {
    const auto &inverse_leaf_size = this->getGridStruct().inverse_leaf_size_;
    return Eigen::Vector3i(
        static_cast<int>(std::floor(x * inverse_leaf_size[0])),
        static_cast<int>(std::floor(y * inverse_leaf_size[1])),
        static_cast<int>(std::floor(z * inverse_leaf_size[2])));
  }

  //  =============================================
};

template <typename PointT>
using VoxelGrid = VoxelFilter<VoxelStructT<PointT>>;

int main() {
  PointCloudT<PointXYZ> input;
  // 0.00, 0.01, ...
  for (size_t i = 0; i < 10; ++i) {
    Vec3f p = Vec3f::Constant(i) / 100;
    input.push_back(p);
  }

  VoxelGrid<PointXYZ> f;
  f.setInputCloud(input);
  f.setMinimumPointsNumberPerVoxel(1);
  PointCloudT<PointXYZ> output;
  f.filter(output);

  for (const auto &pt : output.points)
    std::cout << pt.p.transpose() << std::endl;

  return 0;
}
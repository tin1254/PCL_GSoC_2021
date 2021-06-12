// C++ 17

#include <iostream>
// #include <type_traits>
#include <vector>
#include <unordered_map>
#include <Eigen/Dense>
#include <optional>

using Vec3f = Eigen::Vector3f;

class PointXYZ
{
public:
    PointXYZ(const float x = 0, const float y = 0, const float z = 0) { p << x, y, z; }
    PointXYZ(const Vec3f &p) : p(p) {}
    Vec3f p;
};

template <typename PointT>
class PointCloudT
{
public:
    PointCloudT() {}

    void push_back(const PointT &p)
    {
        points.push_back(p);
    }

    // Defined in pcl::PointCloud
    using PointType = PointT;

    std::vector<PointT> points;
};

template <typename PointT>
class Filter
{
    using PointCloud = PointCloudT<PointXYZ>;

public:
    Filter() {}
    virtual void applyFilter(PointCloud &pc) = 0;
    void filter(PointCloud &output)
    {
        applyFilter(output);
    }
};

// ================== Base of VoxelGrid ==================

// Deduce type

// I've tried deducing by: https://stackoverflow.com/questions/22632236/how-is-possible-to-deduce-function-argument-type-in-c
// but it doesn't work in our case
template <typename T>
using get_point_type = typename T::PointCloud::PointType;

template <typename T>
using get_grid_type = typename T::Grid;

template <typename GridStruct, typename Grid = get_grid_type<GridStruct>, typename PointT = get_point_type<GridStruct>>
class GridFilter : public Filter<PointT>, public GridStruct
{
    // TODO:
    // check if the required functions exist in Grid
    // static_assert(...);

    using PointCloud = PointCloudT<PointT>;

public:
    GridFilter() : GridStruct()
    {
    }

    const PointCloud &getInputCloud() const
    {
        return input_;
    }

    void setInputCloud(const PointCloud &input)
    {
        input_ = input;
    }

    Eigen::Vector3f getLeafSize() const
    {
        return leaf_size_;
    }

    void getMinMax3D(const PointCloud &cloud, Vec3f &min_pt, Vec3f &max_pt) const
    {
        // TODO: set initialize values
        for (const auto &pt : cloud.points)
        {
            min_pt = min_pt.cwiseMin(pt.p);
            max_pt = max_pt.cwiseMax(pt.p);
        }
    }

protected:
    void applyFilter(PointCloud &output)
    {
        GridStruct::setUp(this);
        GridStruct::addPointsToGrid(input_, grid_);

        for (auto it = grid_.begin(); it != grid_.end(); ++it)
            if (std::optional<PointT> res = GridStruct::filterGrid(it, grid_); res)
                output.push_back(res.value());
    }

    PointCloud input_;
    Grid grid_;
    Eigen::Vector3f leaf_size_ = Eigen::Vector3f::Constant(0.05);
    // ...
};

// ================== VoxelGrid.hpp ==================

// structure of a single voxel
struct Voxel
{
    Vec3f centroid = Vec3f::Zero();
    size_t num_pt = 0;
};

// structure and operations of the grid
template <typename PointT>
class VoxelStructT
{
public:
    // Require user to define them
    // not sure if there are a nicer way to do it
    using PointCloud = PointCloudT<PointT>;
    using Grid = std::unordered_map<size_t, Voxel>;

    void setMinimumPointsNumberPerVoxel(const size_t num)
    {
        min_points_per_voxel_ = num;
    }

protected:
    void setUp(const GridFilter<VoxelStructT> *grid_filter)
    {
        Vec3f min_p, max_p;
        const auto &input = grid_filter->getInputCloud();
        grid_filter->getMinMax3D(input, min_p, max_p);

        inverse_leaf_size_ = 1 / grid_filter->getLeafSize().array();
        min_b_ = (min_p.array() * inverse_leaf_size_.array()).cast<int>();
        max_b_ = (max_p.array() * inverse_leaf_size_.array()).cast<int>();

        div_b_ = (max_b_ - min_b_).array() + 1;
        divb_mul_ << div_b_[0], div_b_[0], div_b_[1];
    }

    // only for accessing public methods
    const auto getDerived()
    {
        return static_cast<GridFilter<VoxelStructT> *>(this);
    }

    size_t hashPoint(const PointT &pt)
    {
        const Vec3f tmp = pt.p.array() * inverse_leaf_size_.array();
        return divb_mul_.dot(tmp.cast<int>() - min_b_);
    }

    void addPointToGrid(Grid &grid, const PointT &pt)
    {
        // can be batch/running mode
        const size_t h = hashPoint(pt);
        grid[h].centroid += pt.p;
        grid[h].num_pt++;
    }

    void addPointsToGrid(const PointCloud &input, Grid &grid)
    {
        for (const auto &pt : input.points)
            addPointToGrid(grid, pt);
    }

    std::optional<PointT> filterGrid(Grid::iterator grid_it, Grid &grid)
    {
        const auto &voxel = grid_it->second;
        if (voxel.num_pt >= min_points_per_voxel_)
            return PointT(voxel.centroid / voxel.num_pt);
        else
            return std::nullopt;
    }

    Eigen::Vector3i min_b_, max_b_, div_b_, divb_mul_;
    Vec3f inverse_leaf_size_;
    size_t min_points_per_voxel_;
};

template <typename PointT>
using VoxelGrid = GridFilter<VoxelStructT<PointT>>;

int main()
{
    PointCloudT<PointXYZ> input;
    // 0.00, 0.01, ...
    for (size_t i = 0; i < 10; ++i)
    {
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
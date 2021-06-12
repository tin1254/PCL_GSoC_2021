#include <iostream>
#include <type_traits>
#include <vector>
#include <unordered_map>
#include <Eigen/Dense>
#include <optional>

class PointXYZ
{
public:
    PointXYZ() {}
};

template <typename PointT>
class PointCloudT
{
public:
    PointCloudT() {}

    void push_back(PointT p) {}

    // Defined in pcl::PointCloud
    using PointType = PointT;
};

template <typename PointT>
class Filter
{
    using PointCloud = PointCloudT<PointXYZ>;

public:
    Filter() {}
    virtual void applyFilter(PointCloud &pc) = 0;
    void filter(PointCloud &output) { applyFilter(output); }
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
class GridFilter : public GridStruct, public Filter<PointT>
{
    // TODO:
    // check if the required functions exist in Grid
    // static_assert(...);

    using PointCloud = PointCloudT<PointT>;

public:
    GridFilter() : GridStruct(input_, grid_)
    {
    }

protected:
    void applyFilter(PointCloud &output)
    {
        GridStruct::addPointsToGrid(input_, grid_);

        for (auto it = grid_.begin(); it != grid_.end(); ++it)
        {
            // or filterGrid(it, grid_, input_) ?
            if (std::optional<PointT> res = GridStruct::filterGrid(it, grid_); res)
            {
                output.push_back(res.value());
            }
        }
    }

    PointCloud input_;
    Grid grid_;
    // ...
};

// ================== VoxelGrid.hpp ==================

// structure of a single grid
struct Voxel
{
    Eigen::Vector4f centroid;
    float metric;
    std::vector<size_t> point_indices;
};

// structure and operations of the grids

template <typename PointT>
class VoxelStructT
{
public:
    // Require user to define them
    // not sure if there are a nicer way to do it
    using PointCloud = PointCloudT<PointT>;
    using Grid = std::unordered_map<size_t, Voxel>;

    VoxelStructT(const PointCloud &input, Grid &voxels) {}

    void addPointToGrid() {}

    void addPointsToGrid(const PointCloud &input, Grid &voxels)
    {
        addPointToGrid();
    }

    // bool filterGrid(const std::pair<const size_t, Voxel> &v)
    // {
    //     return false;
    // }

    // PointT reduceGrid(const std::pair<const size_t, Voxel> &v) { return PointT(); }

    // filterGrid + reduceGrid
    std::optional<PointT> filterGrid(Grid::iterator grid_it, Grid &grid)
    {
        if (true)
            return PointT();
        else
            return std::nullopt;
    }

    // attributes of the grid structure
};

template <typename PointT>
using VoxelGrid = GridFilter<VoxelStructT<PointT>>;

int main()
{
    VoxelGrid<PointXYZ> g1;
    PointCloudT<PointXYZ> output;
    g1.filter(output);

    return 0;
}
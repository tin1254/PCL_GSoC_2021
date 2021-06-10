#include <iostream>
#include <type_traits>
#include <vector>
#include <unordered_map>
#include <Eigen/Dense>
#include <iterator>

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

// Check the point type:

// Option 1
// Read the point type based on the returning type of reduceGrid(...)
// With this implementation, I think we can't check beforehand if
// the user provided the mandatory functions
// (use invoke_result_t from C++17)
#define GET_POINT_TYPE_1(GridStruct, Grid) \
    typename std::result_of_t<decltype (&GridStruct::reduceGrid)(GridStruct, const Grid &)>

// Option 2
// Check typedef/using in the GridStruct
// Require user to define it
#define GET_POINT_TYPE_2(GridStruct) \
    typename GridStruct::PointType

template <typename GridStruct, typename Grid, typename PointT = GET_POINT_TYPE_2(GridStruct)>
class GridFilter : public GridStruct, public Filter<PointT>
{
    // TODO:
    // check if the required functions exist in Grid
    // by e.g. https://jguegant.github.io/blogs/tech/sfinae-introduction.html

    // I want to at least enforce user use unordered_map
    // otherwise the class structure will be over-complicated (I think)
    using Grids = std::unordered_map<size_t, Grid>;
    using PointCloud = PointCloudT<PointT>;

public:
    GridFilter() : GridStruct()
    {
    }

protected:
    void applyFilter(PointCloud &output)
    {
        // ...
        GridStruct::setUp(grids_);
        GridStruct::addPointsToGrid(input_, grids_);

        for (const auto &grid : grids_)
        {
            GridStruct::filterGrid(grid.second);
            GridStruct::reduceGrid(grid.second);
        }
    }

    PointCloud input_;
    Grids grids_;
};

// ================== VoxelGrid.hpp ==================

struct Voxel
{
    Eigen::Vector4f centroid;
    float metric;
    std::vector<size_t> point_indices;
};

template <typename PointT>
class VoxelGridT
{
    using Voxels = std::unordered_map<size_t, Voxel>;
    using PointCloud = PointCloudT<PointT>;

public:
    // user have to define this
    using PointType = PointT;

    VoxelGridT() {}

    void setUp(Voxels &voxels) {}

    void addPointToGrid() {}

    void addPointsToGrid(const PointCloud &input, Voxels &voxels)
    {
        for (auto &voxel : voxels)
            addPointToGrid();
    }

    bool filterGrid(const Voxel &v)
    {
        // filter based on metric ...
        return false;
    }

    PointT reduceGrid(const Voxel &v) { return PointT(); }

    // attributes of the grid structure
};

template <typename PointT>
using VoxelGrid = GridFilter<VoxelGridT<PointT>, Voxel>;

int main()
{
    VoxelGrid<PointXYZ> g1;
    PointCloudT<PointXYZ> output;
    g1.filter(output);

    return 0;
}
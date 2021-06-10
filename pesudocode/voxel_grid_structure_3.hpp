#include <iostream>
#include <type_traits>
#include <vector>
#include <unordered_map>
#include <Eigen/Dense>

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

// Check point type by looking at typedef/using in the GridStruct,
// require user to define it
#define GET_POINT_TYPE(GridStruct) \
    typename GridStruct::PointType

template <typename GridStruct, typename Grids, typename PointT = GET_POINT_TYPE(GridStruct)>
class GridFilter : public GridStruct, public Filter<PointT>
{
    // TODO:
    // check if the required functions exist in Grid
    // static_assert(...);

    using PointCloud = PointCloudT<PointT>;

public:
    GridFilter() : GridStruct(input_, grids_)
    {
    }

protected:
    void applyFilter(PointCloud &output)
    {
        // move setup to the contructor of GridStruct?
        // GridStruct::setUp(input_, grids_);
        GridStruct::addPointsToGrid(input_, grids_);

        for (const auto &grid : grids_)
        {
            if (GridStruct::filterGrid(grid))
                continue;

            output.push_back(GridStruct::reduceGrid(grid));
        }
    }

    PointCloud input_;
    Grids grids_;
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

using Voxels = std::unordered_map<size_t, Voxel>;

template <typename PointT>
class VoxelStructT
{
    using PointCloud = PointCloudT<PointT>;

public:
    // Require user to define PointType or explicitly pass to GridFilter
    using PointType = PointT;

    VoxelStructT(const PointCloud &input, Voxels &voxels) {}

    // void setUp() {}

    void addPointToGrid() {}

    void addPointsToGrid(const PointCloud &input, Voxels &voxels)
    {
        addPointToGrid();
    }

    bool filterGrid(const std::pair<const size_t, Voxel> &v)
    {
        // filter based on metric ...
        return false;
    }

    PointT reduceGrid(const std::pair<const size_t, Voxel> &v) { return PointT(); }

    // attributes of the grid structure
};

template <typename PointT>
using VoxelGrid = GridFilter<VoxelStructT<PointT>, Voxels>;

int main()
{
    VoxelGrid<PointXYZ> g1;
    PointCloudT<PointXYZ> output;
    g1.filter(output);

    return 0;
}
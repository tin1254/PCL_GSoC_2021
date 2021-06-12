# VoxelGrid refactored

## Intro

I write a concept of the refactored `applyFilter` of `VoxelGrid` with the consideration of the other voxel filters: `GridMinimum` and `ApproximateVoxelGrid`. The below `applyFilter` can be used by `VoxelGrid` and `GridMinimum`  as they have similar logics. 

The new functors/methods are in the last section, they can be passed to / implemented in the class depending on the logic. We may want to create a new base class for voxel filters to inherit it.

On the other hand, I guess the logic of `ApproximateVoxelGrid` focusing on filtering with just two pass: one for points and one for grids. It  is way different from these two classes, so I think we have to treat it separately, but still 1~2 functors from `VoxelGrid` and `GridMinimum`  can probably be reused in `ApproximateVoxelGrid` . 

## `applyFilter` interface

```c++
// Here presents the drafts of the refactored method
// This draft of applyFilter consists of new highly abstracted methods that has the same functionality as the original class. These new methods can be passed to the class as a functor by having more input arguments for the class attributes.
// On top of each methods/functors, the corresponding code piece with the same functionlity in the original class will be indicated with the line number
// e.g.: For VoxelGrid (filters/include/pcl/filters/impl/voxel_grid.hpp), a new method/functor is indicated with L216-258

template <typename PointT> void
pcl::VoxelGrid<PointT>::applyFilter (PointCloud &output)
{
    // ......

    // same
    Eigen::Vector4f min_p, max_p;
    if (!filter_field_name_.empty ())
        // .......
    else
        getMinMax3D(......,min_p,max_p);

    // L236-L258
    // set up min_b_, max_b_, div_b_, divb_mul_
    setUpGrid(min_p,max_p);

    if (!filter_field_name_.empty ())
    {
        // TODO 
        // ......
    }
    else
    {
        for (const auto& index : (*indices_))
        {
            size_t hash = computePointGridHash(index);
            index_vector.emplace_back(hash, index);
        }    
    }

    // L338-362
    // The idea of the original code I guess is to put all the points into bins, such that the points in the same grid is inside the same bin
    // Since the order of points in a grid isn't necessary, eliminate the sorting here and replace it with a hash map would be a nice touch
    // From two passes O(NlogN) + O(N) to a single pass O(N). And the original code is also difficult to parallelize
    // It should be better by updating it with a concurrent hash map later (I only used TBB for concurrence before and it has a concurrent hash map for parallelizism)
    // Is this better than before? Is there a better approach? @mentors
    std::unordered_map<size_t,std::vector<size_t>> grids;
    size_t n = div_b_[0]*div_b_[1]*div_b_[2];
    // size_t density = indices_.size()/n+1;
    grids.reserve(n);

    for (const auto pt_idx: index_vector)
    {
        const auto& grid = grids[pt_idx.idx];
        // if (grid.empty())
        //     grid.reserve(density);
        grid.push_back(pt_idx.cloud_point_index);
    }

    if (save_leaf_layout_)
    {
        // ......    
    }

    // L392-L428
    output.reserve (grids.size());
    for (const auto& grid:grids)
    {
        if (ignoreGrid(grid))
            continue;
        
		output.push_back(reduceGrid(grid.second));
    }
    
    // ......
}

```



## functors to be passed to the class / methods to be implemented in the class

```c++
// set up min_b_, max_b_, div_b_, divb_mul_
// can be reused in GridMinimum
template <typename PointT> void
pcl::VoxelGrid<PointT>::setUpGrid(const Eigen::Vector4f& min_p, const Eigen::Vector4f& max_p)
{
	// ......
}

// compute the grid index of a single point
// can be reused in GridMinimum and ApproximateVoxelGrid
template <typename PointT> size_t
pcl::VoxelGrid<PointT>::computePointGridHash(const size_t p_idx)
{
    // ......
    return ...;
}

// whether a grid should be ignored
// we can also use FunctorFilter to do the job
template <typename PointT> bool
pcl::VoxelGrid<PointT>::ignoreGrid(const std::vector<size_t>& grid)
{
    // for instance
    return grid.size()<min_points_per_voxel_;
}

// reduce the points inside a grid to a single point
// can be reused in GridMinimum
template <typename PointT> PointT
pcl::VoxelGrid<PointT>::reduceGrid(const std::vector<size_t> indices)
{
    // ......
    return ...;
}
```


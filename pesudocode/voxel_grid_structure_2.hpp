#include <iostream>
#include <vector>
#include <unordered_map>

//  base class inherits class specialized

class PointXYZ{
    public:
    PointXYZ(){
    }
};

template <typename PointT>
class Filter{
    public:
    Filter(){}
};

// new base class

// in order to initialize like this:
// GridFilter<VoxelGridT<PointT>>
template <typename PointT>
class GridFilter;

template <template <typename> class Grid, typename PointT>
class GridFilter<Grid<PointT>>: public Filter<PointT>, protected Grid<PointT>{
    using GridT = Grid<PointT>;
    public:
    GridFilter(): Filter<PointT>(),GridT(){
        // do some checks with GridT
        // like this: https://jguegant.github.io/blogs/tech/sfinae-introduction.html
    }

    void filter(){
        applyFilter();
    }

    protected:

    void applyFilter(){
        GridT::setUpGrid();
        GridT::addPointsToGrid();

        // for ...
        // {
            GridT::filterGrid();
            GridT::reduceGrid();
        // }
    }
};

// VoxelGrid.hpp
template <typename PointT> 
class VoxelGridT{
    public:
    VoxelGridT(){
        // ...
    }

    void setUpGrid(){
        // ...
    }
    
    void addPointToGrid(){
        // ...
    }

    void addPointsToGrid(){
        // for ...
            addPointToGrid();
    }

    bool filterGrid(){
        // filter based on metric ...
        return false;
    }

    PointT reduceGrid(){
        return PointT();
    }

    // attributes:
    // grid structure, metric, ...
};

template <typename PointT>
using VoxelGrid = GridFilter<VoxelGridT<PointT>>;

int main()
{
    VoxelGrid<PointXYZ> test;
    test.filter();
    
    return 0;
}

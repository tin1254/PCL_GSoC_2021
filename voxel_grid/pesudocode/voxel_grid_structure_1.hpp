#include <iostream>
#include <vector>
#include <unordered_map>

// specialized class inherits base class

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
template <typename PointT>
class GridFilter: public Filter<PointT> {
    public:
    GridFilter(){}

    void filter(){
        applyFilter();
    }

    protected:

    virtual void setUpGrid() = 0;
    virtual void addPointsToGrid(){
        // some default implementation
        addPointToGrid();
    };
    virtual void addPointToGrid(){
        // some default implementation
    };
    virtual bool filterGrid() = 0;
    virtual PointT reduceGrid() = 0;

    void applyFilter(){
        setUpGrid();
        addPointsToGrid();

        // for ...
            filterGrid();
            reduceGrid();
    }
};

// VoxelGrid.hpp

// option 1
template <typename PointT>
class VoxelGrid1: public GridFilter<PointT>{
    public:
    VoxelGrid1():GridFilter<PointT>(){
        // ......
    }
    
    protected:

    void setUpGrid() override {
    }

    bool filterGrid() override {
        return false;
    }

    PointT reduceGrid() override {
        return PointT();
    }

    // some attributes for the grid
    // ...
};


// option 2

// in order to initialize like this:
// VoxelGridT<GridFilter<PointT>>
template <typename PointT>
class VoxelGridT;

template <template <typename> class GridFilter, typename PointT>
class VoxelGridT<GridFilter<PointT>>: public GridFilter<PointT>{
    public:
    VoxelGridT():GridFilter<PointT>(){
        // ......
    }
    
    protected:

    void setUpGrid() override {
    }

    bool filterGrid() override {
        return false;
    }

    PointT reduceGrid() override {
        return PointT();
    }

    // some attributes for the grid
    // ...
};

template <typename PointT>
using VoxelGrid2 = VoxelGridT<GridFilter<PointT>>;


int main()
{
    VoxelGrid1<PointXYZ> test1;
    test1.filter();

    VoxelGrid2<PointXYZ> test2;
    test2.filter();

    
    return 0;
}

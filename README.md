**Student:** [Tin Chon Chan][tin1254]

**Mentors:** [Kunal Tyagi][kunaltyagi], [Markus Vieth][mvieth], [Haritha Jayasinghe][haritha-j]

## Status of existing PCL filters

There are various filters in PCL, in which many of them have similar filtering logic in at least part of their code. But since they were implemented by different developers in the early years, a unified code design isn’t the top priority, therefore there are many boilerplate codes across the filter module. The boilerplate code makes further improvements to the filters like parallelizing loops difficult, since one has to change all the similar code in different classes without breaking them.

We found the boilerplate codes mostly come from the voxel filters: `ApproximateVoxelGrid`, `VoxelGrid`, `GridMinimum`, `VoxelGridLabel`, `UniformSampling`. Similarly there are also some boilerplate codes in the binary removal filters. 

The goal of this project is to explore a unified code design to remove the redundancy by refactoring similar code pieces in different filters without breaking the existing API, and try to improve their runtime performance at the same time.

## Developments

We firstly worked on the frequently used voxel filters which have the most redundancy. After a close inspection, we identified the common pattern and the boilerplate codes in the existing voxel filters. Next, I proposed different code design which contains highly abstracted classes, while allowing some level of customization and maximizing the reusability of our implements, to make PCL users able to implement their own grid filter with ease. After some close discussions with my mentors, we selected several plausible code designs and implemented some prototypes for further discussions. In the end, I divided the filter logic of voxel filters and implemented them into new building blocks with template metaprogramming. I applied the new design and refactored two voxel filters, and it proved the new design not only remove the redundant codes, but also improve the performance by up to 40%. The design allows PCL users to create their custom grid filters by reusing our existing implementations and only work on the part with a different filtering logic.

Apart from voxel filters, we also explored new code designs to remove redundancy in other filters. There is a new proposed filter design that takes a lambda function or functor as the filtering logic. I tried to apply the newly introduced design to the existing filters with different approaches. In the end, it proved to have a huge potential that can remove many boilerplate codes and have noticeable runtime performance improvement to the existing filters.

## Challenges

During the project, I realized I don’t have adequate experience in code design, because code quality is usually not the priority in university projects as long as they work. This issue got more critical when I have to design highly abstracted classes. There are many trade-offs between code design complexity, the level of customization for the user, and upgradability. But thanks to my mentors [@kunaltyagi][kunaltyagi] [@mvieth][mvieth] [@haritha-j][haritha-j], they gave me many valuable and prompt feedbacks on the code designs and code quality.

Another challenge for me is template metaprogramming. This is a skill that I will never apply to my university projects so no surprise that I was totally new to it. I invested plenty of time to learn the techniques and finally comfortable applying them to the project.

Finally, it is not rare to filter point clouds with over millions of points, performance can be easily an issue hidden somewhere in the code especially since everything can go wrong with CPP. I spent lots of time ensuring every line of code is efficient enough by exploring different implements for the same filtering logic. Fortunately, all the refactored filters have noticeable performance improvement over the old design.

## Summary

`VoxelGrid` and `VoxelGridLabel` were refactored and implemented in the experimental namespace as a proof of the new code design. `VoxelGrid` is now comprised of several building blocks: `Voxel` (define the voxel grid cell), `VoxelStruct` (implement `VoxelGrid` filtering logic), `VoxelFilter` (implement API for voxel based grid filters), `CartesianFilter` (implement common API for cartesian based grid filter), `TransformFilter` (implement the abstracted logic of grid filters, accept `VoxelStruct` as the filtering logic definition). `VoxelGridLabel` reused all building blocks other than a custom voxel grid cell. The code design concept was documented as a tutorial.

`PassThrough` and `Cropbox` were refactored using `FunctorFilter`. A `FunctorFilter` object will be created for each filtering operation inside applyFilter. Only the core logic for filtering a point is needed to implement now, the boilerplate iteration is handled by `FunctorFilter`.

Visible boilerplate codes were removed in the refactored class. In all the refactored classes, there are measurable runtime improvements. The new grid filter code design allows some level of customization from the users and maximizing the reusability. Different approaches for integrating `FunctorFilter` were explored. It is proven to be able to remove many boilerplate codes in the filter module and bring noticeable runtime improvements.

### PRs

- [Base of grid filters](https://github.com/PointCloudLibrary/pcl/pull/4828)
- [VoxelGrid](https://github.com/PointCloudLibrary/pcl/pull/4829)
- [VoxelGridLabel](https://github.com/PointCloudLibrary/pcl/pull/4870)
- [CropBox & PassThrough](https://github.com/PointCloudLibrary/pcl/pull/4892)
- [Add deprecations in current VoxelGrid](https://github.com/PointCloudLibrary/pcl/pull/4861)

### TODO

- Fix `VoxelGrid` error with point types with label
- Finalize API changes for `VoxelGrid` leaf layout ([discussion](https://github.com/PointCloudLibrary/pcl/issues/4897#issuecomment-903039969))

[tin1254]: https://github.com/tin1254
[kunaltyagi]: https://github.com/kunaltyagi
[mvieth]: https://github.com/mvieth
[haritha-j]: https://github.com/haritha-j

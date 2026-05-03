1. 查看 vertical_ray_caster 采样代码逻辑是否还能优化？
    需要实现查看 nvblox 核心源码提供的各种工具（nvblox_core），包括：
        tsdf_zero_crossings_extractor，
        ray_caster，
        ransac_plane_fitter，
        ground_plane_estimator，
        interpolation_2d，interpolation_3d 等等，
    全部查看分析下功能

    然后考虑两件事情：
        i.      当前的采样功能代码是否有更高效率的实现形式？
        ii.     是否能借助已有工具提升或重构现有代码？
        iii.    在不增加处理算力消耗的情况下，是否还能提升采样精度？
2. 分析下 nvblox 在 dynamics 模式下运行有那些改变？
3. 分析下能否改代码，让 nvblox 在水平坐标系上建图？结合现有的 odom_horizontal_transform 功能包
4. 考虑写个简单实用的插值逻辑，对采样出的高程点阵进行无效值插值处理，并且要高效
5. 优化下点阵可视化效果


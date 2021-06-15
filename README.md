# GateDetector3D

## How to run
___
1. Download and compile [Point Cloud Library](https://pointclouds.org/) (this project used the compile-from-source method, but a prebuilt release should work).
2. Navigate to the workspace directory.
3. Do:
> catkin_make
3. Clone [the simulation repo](https://github.com/vanttec/vanttec_uv_sim/tree/feature/testmissions) (that specific branch) and try to fire it up following the instructions referring to the UUV.
4. Run:

> source devel/setup.sh

> rosrun cloudproc cloudproc_node
___
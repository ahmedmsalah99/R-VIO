# R-VIO





This is a ROS2 implementation with ZUPT for R-VIO.

Everything is very similar:

Build using
``` bash
colcon build
```
run using

```bash
ros2 launch rvio_euroc.launch.py 
```

There is a configuration file for rviz2 almost identical to the rviz config also for visualization.

- Zheng Huai and Guoquan Huang, **Robocentric visual-inertial odometry**, *The International Journal of Robotics Research (IJRR)*, 2022: [download](https://journals.sagepub.com/doi/10.1177/0278364919853361).
```
@article{huai2022robocentric,
  title={Robocentric visual-inertial odometry},
  author={Huai, Zheng and Huang, Guoquan},
  journal={The International Journal of Robotics Research},
  volume={41},
  number={7},
  pages={667--689},
  year={2022},
  publisher={SAGE Publications Sage UK: London, England}
}
```

- Zheng Huai and Guoquan Huang, **Robocentric visual-inertial odometry**, *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, Madrid, Spain, Oct 1-5, 2018: [download](https://ieeexplore.ieee.org/document/8593643).
```
@inproceedings{huai2018robocentric,
  title     = {Robocentric visual-inertial odometry},
  author    = {Huai, Zheng and Huang, Guoquan},
  booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages     = {6319--6326},
  year      = {2018},
  address   = {Madrid, Spain}
}
```

This work has been further extended in our *IEEE RA-L* paper below, and the proposed [R-VIO2](https://github.com/rpng/R-VIO2) is also open sourced.
- Zheng Huai and Guoquan Huang, **Square-Root Robocentric Visual-Inertial Odometry with Online Spatiotemporal Calibration**, *IEEE Robotics and Automation Letters (RA-L)*, 2022: [download](https://ieeexplore.ieee.org/document/9830847).
```
@article{huai2022square,
  title={Square-root robocentric visual-inertial odometry with online spatiotemporal calibration},
  author={Huai, Zheng and Huang, Guoquan},
  journal={IEEE Robotics and Automation Letters},
  volume={7},
  number={4},
  pages={9961--9968},
  year={2022},
  publisher={IEEE}
}
```

![](https://media.giphy.com/media/RMecOYlfxEcy4T8JdS/giphy.gif)

IROS video (**EuRoC MAV** dataset): [YouTube](https://www.youtube.com/watch?v=UtiZ0EKa55M).

![](rvio.gif)

IJRR video (9.8km **Urban Driving** test): [YouTube](https://www.youtube.com/watch?v=l9IC2ddBEYQ).

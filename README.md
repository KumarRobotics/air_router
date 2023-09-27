air_router
---------
![air_router gif](air_router.gif)


air_router replaces the waypoint mission following in PX4-based flight controllers for a high-altitude robot. We use it to maximize communication in a multi-robot setting, where the aerial robot can fly over ground robots and act as a data mule.

## Instructions

To use this node, you should create a QGC mission with the following
characteristics:

1. We take the take off waypoint as the origin (command 22). Please be sure to
   align this takeoff point with your image origin.

2. Be sure to create **only one** fence (inclusion fence), and be sure that all the waypoints are
   within the fence.

3. You can create as many *no fly* zones as you want (exclusion fences).

## Citation

If you find air_router useful, please cite:

```
@misc{cladera2023mocha,
      title={Enabling Large-scale Heterogeneous Collaboration with Opportunistic Communications}, 
      author={Fernando Cladera, Zachary Ravichandran, Ian D. Miller, M. Ani Hsieh, C. J. Taylor, and Vijay Kumar}
}

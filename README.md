# Object-Tracking-Simulation-Evaluation-Script

For watonomous\
Purpose is to compare tracker outputs to ground truth CARLA data, and gives MOTA, MOTP metrics.\
We will also evaluate tracker outputs based on nuScenes/KITTI datasets and provide said metrics.
This would allow us to better analyze the performance of the object tracker over time and given various different configurations or improvements.


-publishes dataset to ros stack for the object tracker to use.
-subscribes to the output of the object tracker and store its result
-calculate MOTA and MOTP metrics

https://visailabs.com/evaluating-multiple-object-tracking-accuracy-and-performance-metrics-in-a-real-time-setting/

Ground truths : nuScene and KITTI datasets, and manually collected CARLA data.
Predicted detection is whatever the object tracket spits out.

nuScenes download:
https://www.nuscenes.org/nuscenes

KITTI download:
http://www.cvlibs.net/datasets/kitti/eval_tracking.php

To evaluate nuScenes, we will convert them to kitti.
include an argument config


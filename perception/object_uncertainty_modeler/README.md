# object_uncertainty_modeler

This package contains a radar object filter module for `radar_msgs/msg/RadarTrack`.
This package can filter noise objects in RadarTracks.

## Algorithm

The core algorithm of this package is `RadarTrackCrossingNoiseFilterNode::isNoise()` function.
See the function and the parameters for details.

## Input

| Name             | Type                           | Description         |
| ---------------- | ------------------------------ | ------------------- |
| `~/input/tracks` | radar_msgs/msg/RadarTracks.msg | 3D detected tracks. |

## Output

| Name              | Type                           | Description                                 |
| ----------------- | ------------------------------ | ------------------------------------------- |
| `~/output/tracks` | radar_msgs/msg/RadarTracks.msg | Objects with covariance matrix is simulated |

## Parameters

| Name                   | Type   | Description                                                                                                                        | Default value |
| :--------------------- | :----- | :--------------------------------------------------------------------------------------------------------------------------------- | :------------ |
| `velocity_y_threshold` | double | Y-axis velocity threshold [m/s]. If y-axis velocity of RadarTrack is more than `velocity_y_threshold`, it treats as noise objects. | 7.0           |

# Vision ROI object detector

## Purpose

vision_roi_object_detector is a package to detect 3D bounding box object from 2D region of interest (ROI).
This 
 
## Inner-working / Algorithms

Object position on the image is projected to the ground and the object position is estimated.

## Input / Output

### Input

| Name                | Type                                                     | Description         |
| ------------------- | -------------------------------------------------------- | ------------------- |
| `input/rois`        | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | ROI Detections      |
| `input/camera_info` | `sensor_msgs::msg::CameraInfo`                           | Camera Infos        |

### Output

| Name                 | Type                                                  | Description     |
| -------------------- | ----------------------------------------------------- | --------------- |
| `output/roi_objects` | `autoware_auto_perception_msgs::msg::DetectedObjects` | 3D Objects      |

## Parameters

| Name              | Type   | Description                          | Default value |
| :---------------- | :----- | :----------------------------------- | :------------ |
| `output_frame_id` | string | The header frame_id of output topic. | base_link     |

## Assumptions / Known limits

<!-- Write assumptions and limitations of your implementation.

Example:
  This algorithm assumes obstacles are not moving, so if they rapidly move after the vehicle started to avoid them, it might collide with them.
  Also, this algorithm doesn't care about blind spots. In general, since too close obstacles aren't visible due to the sensing performance limit, please take enough margin to obstacles.
-->

## (Optional) Error detection and handling

<!-- Write how to detect errors and how to recover from them.

Example:
  This package can handle up to 20 obstacles. If more obstacles found, this node will give up and raise diagnostic errors.
-->

## (Optional) Performance characterization

<!-- Write performance information like complexity. If it wouldn't be the bottleneck, not necessary.

Example:
  ### Complexity

  This algorithm is O(N).

  ### Processing time

  ...
-->

## (Optional) References/External links

<!-- Write links you referred to when you implemented.

Example:
  [1] {link_to_a_thesis}
  [2] {link_to_an_issue}
-->

## (Optional) Future extensions / Unimplemented parts

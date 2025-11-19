from enum import Enum
import logging
import os
from pathlib import Path
import pprint

# Import multi-terminal launcher functionality
# ROS2 launch files don't support relative imports, so we add the launch directory to path
import sys

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

launch_dir = Path(__file__).parent
if str(launch_dir) not in sys.path:
    sys.path.insert(0, str(launch_dir))

import multi_terminal.multi_terminal_launcher as multi_terminal_launcher

detect_terminal_method = multi_terminal_launcher.detect_terminal_method
launch_in_tmux = multi_terminal_launcher.launch_in_tmux
launch_in_terminator = multi_terminal_launcher.launch_in_terminator

# Configuration: Enable/disable launching each launcher in separate terminals
USE_SEPARATE_TERMINALS = True

logger = logging.getLogger(__name__)
# Configure logger to output to terminal
if not logger.handlers:
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter("%(levelname)s: %(message)s"))
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)


class PerceptionModality(str, Enum):
    NONE = "none"
    CAMERA_LIDAR_RADAR_FUSION = "camera_lidar_radar_fusion"
    CAMERA_LIDAR_FUSION = "camera_lidar_fusion"
    LIDAR_RADAR_FUSION = "lidar_radar_fusion"
    LIDAR = "lidar"
    CAMERA = "camera"
    RADAR = "radar"


class ObstacleSegmentationFilterType(Enum):
    NONE = 0
    NoFilter = 1
    SingleFrame = 2
    TimeSeries = 3
    Combined = 4


class ObjectRecognitionFusionType(Enum):
    NONE = 0
    Serial = 1
    Parallel = 2


class ObjectRecognitionLidarModelType(Enum):
    NONE = 0
    Centerpoint = 1
    Pointpainting = 2
    Apollo = 3
    Transfusion = 4


class ObjectRecognitionPredictionType(Enum):
    NONE = 0
    MapBased = 1
    SimplModel = 2


class OccupancyGridMapType(Enum):
    NONE = 0
    PointcloudBased = 1
    LaserscanBased = 2
    MultiLidarPointcloudBased = 3


class TrafficLightRecognitionType(Enum):
    NONE = 0
    FusionOnly = 1
    FusionOnlyFineDetection = 2
    FineDetection = 3
    WholeImageDetection = 4
    StandardDetection = 5


def config_to_bool(config_name: str, context) -> bool:
    """Convert LaunchConfiguration to boolean."""
    value = LaunchConfiguration(config_name).perform(context)
    return value == "True" or value == "true"


def config_to_str(config_name: str, context) -> str:
    """Convert LaunchConfiguration to string."""
    value = LaunchConfiguration(config_name).perform(context)
    return str(value)


def determine_modes(context) -> tuple[bool, dict]:
    """Determine mode based on launch configurations."""
    modes = {
        "perception_modality": PerceptionModality.NONE,
        "obstacle_segmentation_filter": ObstacleSegmentationFilterType.NONE,
        "object_recognition_fusion_type": ObjectRecognitionFusionType.NONE,
        "object_recognition_lidar_model_type": ObjectRecognitionLidarModelType.NONE,
        "object_recognition_prediction_type": ObjectRecognitionPredictionType.NONE,
        "occupancy_grid_map_type": OccupancyGridMapType.NONE,
        "traffic_light_recognition_type": TrafficLightRecognitionType.NONE,
    }
    is_supported = True

    ## common
    if config_to_bool("downsample_perception_common_pointcloud", context) == True:
        is_supported = False
        return is_supported, modes

    if config_to_bool("use_perception_online_evaluator", context):
        modes["perception_online_evaluator"] = True
    else:
        modes["perception_online_evaluator"] = False

    if config_to_bool("use_perception_analytics_publisher", context):
        modes["perception_analytics_publisher"] = True
    else:
        modes["perception_analytics_publisher"] = False

    ## obstacle segmentation
    if config_to_bool("use_obstacle_segmentation_time_series_filter", context):
        if config_to_bool("use_obstacle_segmentation_single_frame_filter", context):
            modes["obstacle_segmentation_filter"] = ObstacleSegmentationFilterType.Combined
        else:
            modes["obstacle_segmentation_filter"] = ObstacleSegmentationFilterType.SingleFrame
    else:
        if config_to_bool("use_obstacle_segmentation_single_frame_filter", context):
            modes["obstacle_segmentation_filter"] = ObstacleSegmentationFilterType.TimeSeries
        else:
            modes["obstacle_segmentation_filter"] = ObstacleSegmentationFilterType.NoFilter

    ## object recognition
    # modality with lidar detection model
    lidar_mode, lidar_model = config_to_str("lidar_detection_model", context).split("/")
    modality = config_to_str("mode", context)
    if modality == PerceptionModality.CAMERA_LIDAR_RADAR_FUSION:
        modes["perception_modality"] = PerceptionModality.CAMERA_LIDAR_RADAR_FUSION
        if lidar_mode == "centerpoint":
            modes["object_recognition_lidar_model_type"] = (
                ObjectRecognitionLidarModelType.Centerpoint
            )
        elif lidar_mode == "pointpainting":
            modes["object_recognition_lidar_model_type"] = (
                ObjectRecognitionLidarModelType.Pointpainting
            )
        elif lidar_mode == "transfusion":
            modes["object_recognition_lidar_model_type"] = (
                ObjectRecognitionLidarModelType.Transfusion
            )
        else:
            is_supported = False
            return is_supported, modes
    elif modality == PerceptionModality.CAMERA_LIDAR_FUSION:
        modes["perception_modality"] = PerceptionModality.CAMERA_LIDAR_FUSION
        if lidar_mode == "centerpoint":
            modes["object_recognition_lidar_model_type"] = (
                ObjectRecognitionLidarModelType.Centerpoint
            )
        elif lidar_mode == "pointpainting":
            modes["object_recognition_lidar_model_type"] = (
                ObjectRecognitionLidarModelType.Pointpainting
            )
        elif lidar_mode == "transfusion":
            modes["object_recognition_lidar_model_type"] = (
                ObjectRecognitionLidarModelType.Transfusion
            )
        elif lidar_mode == "apollo":
            modes["object_recognition_lidar_model_type"] = ObjectRecognitionLidarModelType.Apollo
        else:
            is_supported = False
            return is_supported, modes
    elif modality == PerceptionModality.LIDAR_RADAR_FUSION:
        modes["perception_modality"] = PerceptionModality.LIDAR_RADAR_FUSION
        if lidar_model == "pointpainting":
            logger.error("Pointpainting is not supported without camera")
            return False, modes
        is_supported = False
        return is_supported, modes
    elif modality == PerceptionModality.LIDAR:
        modes["perception_modality"] = PerceptionModality.LIDAR
        if lidar_model == "pointpainting":
            logger.error("Pointpainting is not supported without camera")
            return False, modes
        if lidar_mode == "centerpoint":
            modes["object_recognition_lidar_model_type"] = (
                ObjectRecognitionLidarModelType.Centerpoint
            )
        elif lidar_mode == "transfusion":
            modes["object_recognition_lidar_model_type"] = (
                ObjectRecognitionLidarModelType.Transfusion
            )
        else:
            is_supported = False
            return is_supported, modes
    elif modality == PerceptionModality.CAMERA:
        modes["perception_modality"] = PerceptionModality.CAMERA
        is_supported = False
        return is_supported, modes
    elif modality == PerceptionModality.RADAR:
        modes["perception_modality"] = PerceptionModality.RADAR
        is_supported = False
        return is_supported, modes
    else:
        is_supported = False
        return is_supported, modes

    # use_multi_channel_tracker_merger only supported for camera_lidar_radar_fusion
    if config_to_bool("use_multi_channel_tracker_merger", context) == True:
        if modes["perception_modality"] == PerceptionModality.CAMERA_LIDAR_RADAR_FUSION:
            modes["object_recognition_fusion_type"] = ObjectRecognitionFusionType.Parallel
        else:
            modes["object_recognition_fusion_type"] = ObjectRecognitionFusionType.Serial
    elif config_to_bool("use_multi_channel_tracker_merger", context) == True:
        # the other modalities do not support multi-channel tracker merger
        is_supported = False
        return is_supported, modes
    else:
        # the other modalities with serial fusion
        modes["object_recognition_fusion_type"] = ObjectRecognitionFusionType.Serial

    # object prediction
    if config_to_str("prediction_model_type", context) == "map_based":
        modes["object_recognition_prediction_type"] = ObjectRecognitionPredictionType.MapBased
    else:
        is_supported = False
        return is_supported, modes

    ## occupancy grid map
    if config_to_str("occupancy_grid_map_method", context) == "pointcloud_based_occupancy_grid_map":
        modes["occupancy_grid_map"] = OccupancyGridMapType.PointcloudBased
    elif (
        config_to_str("occupancy_grid_map_method", context) == "laserscan_based_occupancy_grid_map"
    ):
        # not supported
        is_supported = False
        return is_supported, modes
    elif (
        config_to_str("occupancy_grid_map_method", context)
        == "multi_lidar_pointcloud_based_occupancy_grid_map"
    ):
        modes["occupancy_grid_map"] = OccupancyGridMapType.MultiLidarPointcloudBased
    else:
        modes["occupancy_grid_map"] = OccupancyGridMapType.NONE

    ## traffic light recognition
    if config_to_bool("use_traffic_light_recognition", context) == False:
        modes["traffic_light_recognition"] = TrafficLightRecognitionType.NONE
    elif config_to_bool("traffic_light_recognition/fusion_only", context) == True:
        if (
            config_to_str("traffic_light_recognition/high_accuracy_detection_type", context)
            == "fine_detection"
        ):
            modes["traffic_light_recognition"] = TrafficLightRecognitionType.FusionOnlyFineDetection
        else:
            modes["traffic_light_recognition"] = TrafficLightRecognitionType.FusionOnly
    elif config_to_bool("traffic_light_recognition/use_high_accuracy_detection", context) == True:
        if (
            config_to_str("traffic_light_recognition/high_accuracy_detection_type", context)
            == "fine_detection"
        ):
            modes["traffic_light_recognition"] = TrafficLightRecognitionType.FineDetection
        elif (
            config_to_str("traffic_light_recognition/high_accuracy_detection_type", context)
            == "whole_image_detection"
        ):
            modes["traffic_light_recognition"] = TrafficLightRecognitionType.WholeImageDetection
        else:
            is_supported = False
            return is_supported, modes
    else:
        modes["traffic_light_recognition"] = TrafficLightRecognitionType.NONE

    return is_supported, modes


def determine_launcher_paths(modes: dict) -> list[str]:
    """
    Determine launcher paths based on modes.

    Returns a list of launcher file paths (relative to package share directory).
    These are placeholders that should be filled based on the actual mode combinations.
    """
    launcher_paths = []

    # common

    # obstacle segmentation
    if modes.get("obstacle_segmentation_filter") == ObstacleSegmentationFilterType.Combined:
        launcher_paths.append(
            "exports/LidarCenterpoint.system/launcher/Runtime/main_ecu/obstacle_segmentation/obstacle_segmentation.launch.xml"
        )
    elif modes.get("obstacle_segmentation_filter") == ObstacleSegmentationFilterType.SingleFrame:
        launcher_paths.append(
            "exports/LidarCenterpoint.system/launcher/Runtime/main_ecu/obstacle_segmentation/obstacle_segmentation.launch.xml"
        )
    elif modes.get("obstacle_segmentation_filter") == ObstacleSegmentationFilterType.TimeSeries:
        launcher_paths.append(
            "exports/LidarCenterpoint.system/launcher/Runtime/main_ecu/obstacle_segmentation/obstacle_segmentation.launch.xml"
        )
    elif modes.get("obstacle_segmentation_filter") == ObstacleSegmentationFilterType.NoFilter:
        launcher_paths.append(
            "exports/LidarCenterpoint.system/launcher/Runtime/main_ecu/obstacle_segmentation/obstacle_segmentation.launch.xml"
        )
    else:
        logger.warning(
            "Without obstacle segmentation: %s", modes.get("obstacle_segmentation_filter")
        )

    # object recognition
    modality = modes.get("perception_modality")
    fusion_type = modes.get("object_recognition_fusion_type")
    lidar_model = modes.get("object_recognition_lidar_model_type")

    if modality == PerceptionModality.CAMERA_LIDAR_RADAR_FUSION:
        # centerpoint only
        if lidar_model == ObjectRecognitionLidarModelType.Centerpoint:
            if fusion_type == ObjectRecognitionFusionType.Parallel:
                launcher_paths.append(
                    "exports/CameraLidarRadarCenterpointParallel.system/launcher/Runtime/main_ecu/object_recognition/object_recognition.launch.xml"
                )
            elif fusion_type == ObjectRecognitionFusionType.Serial:
                launcher_paths.append(
                    "exports/CameraLidarRadarCenterpointSerial.system/launcher/Runtime/main_ecu/object_recognition/object_recognition.launch.xml"
                )
            launcher_paths.append(
                "exports/CameraLidarCenterpointA.system/launcher/Runtime/main_ecu/centerpoint/centerpoint.launch.xml"
            )
        else:
            logger.error("Invalid lidar model type for camera_lidar_radar_fusion: %s", lidar_model)
    elif modality == PerceptionModality.CAMERA_LIDAR_FUSION:
        if lidar_model == ObjectRecognitionLidarModelType.Centerpoint:
            launcher_paths.append(
                "exports/CameraLidarCenterpointA.system/launcher/Runtime/main_ecu/centerpoint/centerpoint.launch.xml"
            )
            launcher_paths.append(
                "exports/CameraLidarCenterpointA.system/launcher/Runtime/main_ecu/object_recognition/object_recognition.launch.xml"
            )
        elif lidar_model == ObjectRecognitionLidarModelType.Pointpainting:
            launcher_paths.append(
                "exports/CameraLidarPointpainting.system/launcher/Runtime/main_ecu/pointpainting/pointpainting.launch.xml"
            )
            launcher_paths.append(
                "exports/CameraLidarPointpainting.system/launcher/Runtime/main_ecu/object_recognition/object_recognition.launch.xml"
            )
        elif lidar_model == ObjectRecognitionLidarModelType.Apollo:
            launcher_paths.append(
                "exports/CameraLidarApollo.system/launcher/Runtime/main_ecu/apollo/apollo.launch.xml"
            )
            launcher_paths.append(
                "exports/CameraLidarApollo.system/launcher/Runtime/main_ecu/object_recognition/object_recognition.launch.xml"
            )
        else:
            logger.error("Invalid lidar model for camera_lidar_fusion: %s", lidar_model)

    elif modality == PerceptionModality.LIDAR_RADAR_FUSION:
        logger.error("Lidar radar fusion is not supported")
    elif modality == PerceptionModality.LIDAR:
        if lidar_model == ObjectRecognitionLidarModelType.Centerpoint:
            launcher_paths.append(
                "exports/LidarCenterpoint.system/launcher/Runtime/main_ecu/centerpoint/centerpoint.launch.xml"
            )
            launcher_paths.append(
                "exports/LidarCenterpoint.system/launcher/Runtime/main_ecu/object_recognition/object_recognition.launch.xml"
            )
        elif lidar_model == ObjectRecognitionLidarModelType.Transfusion:
            launcher_paths.append(
                "exports/LidarTransfusion.system/launcher/Runtime/main_ecu/transfusion/transfusion.launch.xml"
            )
            launcher_paths.append(
                "exports/LidarTransfusion.system/launcher/Runtime/main_ecu/object_recognition/object_recognition.launch.xml"
            )
        else:
            logger.error("Invalid lidar model for lidar: %s", lidar_model)
    elif modality == PerceptionModality.CAMERA:
        logger.error("Camera is not supported")
    elif modality == PerceptionModality.RADAR:
        logger.error("Radar is not supported")
    else:
        logger.warning("Without object recognition: %s", modality)

    # occupancy grid map
    occupancy_grid_map_mode = modes.get("occupancy_grid_map")
    if occupancy_grid_map_mode == OccupancyGridMapType.PointcloudBased:
        launcher_paths.append(
            "exports/CameraLidarCenterpointA.system/launcher/Runtime/main_ecu/occupancy_grid_map/occupancy_grid_map.launch.xml"
        )
    elif occupancy_grid_map_mode == OccupancyGridMapType.LaserscanBased:
        launcher_paths.append(
            "exports/CameraLidarCenterpointB.system/launcher/Runtime/main_ecu/occupancy_grid_map/occupancy_grid_map.launch.xml"
        )
    elif occupancy_grid_map_mode == OccupancyGridMapType.MultiLidarPointcloudBased:
        launcher_paths.append(
            "exports/CameraLidarCenterpointC.system/launcher/Runtime/main_ecu/occupancy_grid_map/occupancy_grid_map.launch.xml"
        )
    else:
        logger.error("Invalid occupancy grid map mode: %s", occupancy_grid_map_mode)

    # traffic light recognition
    traffic_light_mode = modes.get("traffic_light_recognition")
    if traffic_light_mode == TrafficLightRecognitionType.NONE:
        pass
    elif traffic_light_mode == TrafficLightRecognitionType.FusionOnly:
        launcher_paths.append(
            "exports/CameraLidarCenterpointA.system/launcher/Runtime/main_ecu/traffic_light_recognition/traffic_light_recognition.launch.xml"
        )
    elif traffic_light_mode == TrafficLightRecognitionType.FineDetection:
        launcher_paths.append(
            "exports/CameraLidarCenterpointB.system/launcher/Runtime/main_ecu/traffic_light_recognition/traffic_light_recognition.launch.xml"
        )
    elif traffic_light_mode == TrafficLightRecognitionType.WholeImageDetection:
        launcher_paths.append(
            "exports/CameraLidarCenterpointC.system/launcher/Runtime/main_ecu/traffic_light_recognition/traffic_light_recognition.launch.xml"
        )
    else:
        logger.error("Invalid traffic light recognition mode: %s", traffic_light_mode)

    return launcher_paths


def determine_parameters(context, modes: dict) -> dict:
    """
    Determine parameters dictionary based on launch arguments and modes.

    This is a placeholder function. Parameters should be extracted from launch arguments
    and organized into a dictionary structure for passing to launchers.
    """
    parameters = {}

    # Placeholder: Extract and organize parameters from context
    # This should be implemented to extract all relevant launch arguments
    # and organize them into the appropriate structure

    # Example (to be replaced with actual logic):
    # parameters["data_path"] = config_to_str("data_path", context)
    # parameters["config_path"] = config_to_str("config_path", context)
    # parameters["vehicle_param_file"] = config_to_str("vehicle_param_file", context)
    # etc.

    return parameters


def create_pointcloud_container():
    """Create the pointcloud_container composable node container."""
    return ComposableNodeContainer(
        name="pointcloud_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
    )


def launch_xml_fallback(launch_arguments_names, xml_launcher_file: str):
    """Launch XML launcher file with all given arguments (fallback for unsupported modes)."""
    # Build launch arguments dictionary using LaunchConfiguration for each argument
    logger.info("Launching XML fallback: %s", xml_launcher_file)
    launch_args_dict = {name: LaunchConfiguration(name) for name in launch_arguments_names}

    return IncludeLaunchDescription(
        XMLLaunchDescriptionSource(xml_launcher_file), launch_arguments=launch_args_dict.items()
    )


def launch_prebuilt_launchers(
    context,
    launch_arguments_names: list[str],
    launcher_paths: list[str],
    use_separate_terminals: bool,
    terminal_method: str,
    launch_pointcloud_container: bool = False,
):
    """
    Launch pre-built launcher files with all given arguments.

    Args:
        context: Launch context
        launch_arguments_names: List of launch argument names
        launcher_paths: List of launcher file paths (relative to package share directory)
        use_separate_terminals: Whether to launch in separate terminals
        terminal_method: Terminal method to use ('terminator', 'tmux', or 'none')
        launch_pointcloud_container: If True, launch pointcloud container in separate terminal
    """
    launcher_pkg_install_dir = get_package_share_directory("tier4_perception_launch")

    launch_actions = []

    if use_separate_terminals and terminal_method != "none":
        # Launch each launcher in a separate terminal
        if terminal_method == "tmux":
            # Launch with tmux: create new panes in a single window
            launch_actions.extend(
                launch_in_tmux(
                    context,
                    launch_arguments_names,
                    launcher_paths,
                    launcher_pkg_install_dir,
                    launch_pointcloud_container=launch_pointcloud_container,
                )
            )
        elif terminal_method == "terminator":
            # Launch with terminator: create split panes using layout file
            titles = [os.path.basename(path) for path in launcher_paths]
            if launch_pointcloud_container:
                titles.insert(0, "pointcloud_container")
            launch_actions.extend(
                launch_in_terminator(
                    context,
                    launch_arguments_names,
                    launcher_paths,
                    launcher_pkg_install_dir,
                    launch_pointcloud_container=launch_pointcloud_container,
                    titles=titles,
                )
            )
    else:
        # Use regular launch (current approach)
        launch_args_dict = {name: LaunchConfiguration(name) for name in launch_arguments_names}

        for launcher_path in launcher_paths:
            launcher_file = os.path.join(launcher_pkg_install_dir, launcher_path)
            launch_actions.append(
                IncludeLaunchDescription(
                    XMLLaunchDescriptionSource(launcher_file),
                    launch_arguments=launch_args_dict.items(),
                )
            )
    return launch_actions


def opaque_launch_perception_system(
    context, launch_argument_names: list[str], xml_fallback_file: str
):
    """
    OpaqueFunction wrapper to launch perception system at launch time.

    Flow:
    1. Parse arguments and determine modes and if the combination is supported
    2. If not supported, launch with XML, handing over all the given arguments
    3. If supported, determine set of launchers (pre-built) and launch them
    """
    # Parse arguments and determine modes
    is_supported, modes = determine_modes(context)

    # Determine parameters (placeholder - to be implemented)
    parameters = determine_parameters(context, modes)

    # Detect terminal method if separate terminals are enabled
    terminal_method = "none"
    if USE_SEPARATE_TERMINALS:
        terminal_method = detect_terminal_method()
        if terminal_method != "none":
            logger.info("Using separate terminals with method: %s", terminal_method)
        else:
            logger.info(
                "Separate terminals requested but no suitable terminal found. Using regular launch."
            )

    launch_actions = []

    # Determine if pointcloud container should be launched in separate terminal
    launch_pointcloud_container_in_terminal = USE_SEPARATE_TERMINALS and terminal_method != "none"

    # If using separate terminals but terminal method is not available, add pointcloud container directly
    if USE_SEPARATE_TERMINALS and terminal_method == "none":
        logger.info("Adding pointcloud container directly (terminal method not available)")
        launch_actions.append(create_pointcloud_container())

    if not is_supported:
        # If not supported, launch with XML fallback, handing over all given arguments
        logger.warning(
            "Mode combination not supported. Using XML fallback. Modes: %s",
            pprint.pformat(modes, indent=2),
        )
        launch_actions.append(launch_xml_fallback(launch_argument_names, xml_fallback_file))
    else:
        # If supported, determine set of launchers (pre-built) and launch them
        logger.info(
            "Mode combination supported. Using pre-built launchers. Modes: %s",
            pprint.pformat(modes, indent=2),
        )
        launcher_paths = determine_launcher_paths(modes)

        if not launcher_paths:
            logger.warning(
                "No launcher paths determined for modes. Using XML fallback. Modes: %s",
                pprint.pformat(modes, indent=2),
            )
            launch_actions.append(launch_xml_fallback(launch_argument_names, xml_fallback_file))
        else:
            # print launcher paths with indent
            logger.info("Launching pre-built launchers:")
            for launcher_path in launcher_paths:
                logger.info("  - %s", launcher_path)
            # launch the pre-built launchers
            launch_actions.extend(
                launch_prebuilt_launchers(
                    context,
                    launch_argument_names,
                    launcher_paths,
                    USE_SEPARATE_TERMINALS,
                    terminal_method,
                    launch_pointcloud_container=launch_pointcloud_container_in_terminal,
                )
            )

    return launch_actions


def generate_launch_description():
    """Generate autoware system and launch the generated launch file."""

    # set launch arguments
    launch_arguments = []
    launch_argument_names = []

    def add_launch_arg(name: str, default_value=None, **kwargs):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value, **kwargs))
        launch_argument_names.append(name)

    # deployment configuration (must be declared first so it can be referenced)
    add_launch_arg(
        "data_path",
        default_value=[PathJoinSubstitution([EnvironmentVariable("HOME"), "autoware_data"])],
    )  # config

    # for perception ecu (now using LaunchConfiguration to reference data_path)
    add_launch_arg(
        "config_path",
        default_value=[PathJoinSubstitution([FindPackageShare("autoware_launch"), "config"])],
    )  # config
    add_launch_arg("vehicle_model", default_value="j6_gen2")  # vehicle model
    add_launch_arg("pointcloud_container_name", default_value="pointcloud_container")

    # 1. pipeline junctions: switches to change SW system

    # Simulation / Evaluation
    add_launch_arg("use_empty_dynamic_object_publisher", default_value="false")
    add_launch_arg("use_perception_online_evaluator", default_value="false")
    add_launch_arg("use_perception_analytics_publisher", default_value="true")

    # Common
    add_launch_arg("downsample_perception_common_pointcloud", default_value="false")

    # Object recognition
    add_launch_arg("mode", default_value="camera_lidar_radar_fusion")
    add_launch_arg("use_multi_channel_tracker_merger", default_value="false")  # merger and tracker
    add_launch_arg(
        "use_irregular_object_detector", default_value="true"
    )  # detector on/off, also for merger
    add_launch_arg(
        "use_detection_by_tracker", default_value="true"
    )  # detector on/off, also for merger
    add_launch_arg("use_object_validator", default_value="true")  # always true
    add_launch_arg("lidar_detection_model", default_value="centerpoint/centerpoint_tiny")

    # Object recognition / detection / detector
    add_launch_arg(
        "use_low_height_cropbox", default_value="false"
    )  # filter on/off of euclidean clustering

    # Object recognition / detection / detector / camera_lidar
    add_launch_arg(
        "use_low_intensity_cluster_filter", default_value="true"
    )  # additional detector on/off, also for merger
    add_launch_arg("use_image_segmentation_based_filter", default_value="false")  # filter on/off
    add_launch_arg(
        "segmentation_pointcloud_fusion_camera_ids", default_value="[0,2,4]"
    )  # sensor set, only if use_image_segmentation_based_filter

    # Object recognition / detection / filter
    add_launch_arg("use_object_filter", default_value="true")  # filter and merger, almost true
    add_launch_arg(
        "objects_filter_method", default_value="lanelet_filter"
    )  # objects filter mode, lanelet_filter or position_filter
    add_launch_arg(
        "objects_validation_method", default_value="obstacle_pointcloud"
    )  # mode, obstacle_pointcloud or occupancy_grid
    add_launch_arg("use_pointcloud_map", default_value="true")  # pointcloud map filter mode

    # Object recognition / detection / merger
    add_launch_arg(
        "ml_camera_lidar_merger_priority_mode", default_value="0"
    )  # dynamic configuration, only if !use_multi_channel_tracker_merger

    # Object recognition / tracking
    add_launch_arg(
        "use_radar_tracking_fusion", default_value="true"
    )  # merge topology change, only if !use_multi_channel_tracker_merger and radar is used
    add_launch_arg(
        "tracker_publish_merged_objects", default_value="false"
    )  # dynamic configuration, only if use_multi_channel_tracker_merger

    # Object recognition / prediction
    add_launch_arg("use_vector_map", default_value="true")  # always true
    add_launch_arg(
        "prediction_model_type", default_value="map_based"
    )  # prediction mode, map_based or SIMPL model

    # Obstacle segmentation
    add_launch_arg(
        "use_obstacle_segmentation_single_frame_filter", default_value="false"
    )  # ground segmentation filter mode
    add_launch_arg(
        "use_obstacle_segmentation_time_series_filter", default_value="true"
    )  # ground segmentation filter mode

    # Occupancy grid map
    add_launch_arg(
        "occupancy_grid_map_method", default_value="pointcloud_based_occupancy_grid_map"
    )  # occupancy grid map mode
    # add_launch_arg("occupancy_grid_map_updater", default_value="binary_bayes_filter") # always binary_bayes_filter

    # traffic light
    add_launch_arg(
        "use_traffic_light_recognition", default_value="true"
    )  # module switch. need for simulation
    add_launch_arg(
        "traffic_light_recognition/fusion_only", default_value="false"
    )  # linked with ecu
    add_launch_arg(
        "traffic_light_recognition/camera_namespaces", default_value="[camera6, camera7]"
    )  # sensor set
    add_launch_arg(
        "traffic_light_recognition/use_high_accuracy_detection", default_value="true"
    )  # filter on/off
    add_launch_arg(
        "traffic_light_recognition/high_accuracy_detection_type", default_value="fine_detection"
    )  # filter mode, whole_image_detection or fine_detection

    # Setup XML fallback launcher file path (for unsupported modes)
    launcher_pkg_install_dir = get_package_share_directory("tier4_perception_launch")
    xml_fallback_path = "launch/perception.launch.xml"
    xml_fallback_file = os.path.join(launcher_pkg_install_dir, xml_fallback_path)

    # Launch the perception system (will determine if supported and launch accordingly)
    # The pointcloud container will be launched in a separate terminal if multi-terminal is enabled
    # Otherwise, it will be added directly to the launch description
    launch_actions_list = [
        OpaqueFunction(
            function=lambda context: opaque_launch_perception_system(
                context, launch_argument_names, xml_fallback_file
            )
        )
    ]

    # Only add pointcloud container directly if NOT using separate terminals
    # (If using separate terminals, it will be handled in the opaque function)
    if not USE_SEPARATE_TERMINALS:
        pointcloud_container = create_pointcloud_container()
        launch_actions_list.insert(0, pointcloud_container)

    return LaunchDescription(launch_arguments + launch_actions_list)

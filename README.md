# rcprg_gazebo_utils {#rcprg_gazebo_utils_readme}

This package provides utilities for Gazebo simulator.

Launch files:
* gazebo_client.launch - runs Gazebo client; please note that you should provide the same paths for resources as for the server using ROS launch arguments:
    - *GAZEBO_MODEL_PATH* - - please refer to Gazebo documentation
    - *GAZEBO_RESOURCE_PATH* - - please refer to Gazebo documentation
* gazebo_world_editor.launch - runs Gazebo server and client; can be used as world editor; use the following arguments to set resource paths:
    - *GAZEBO_MODEL_PATH* - - please refer to Gazebo documentation
    - *GAZEBO_RESOURCE_PATH* - - please refer to Gazebo documentation
    - *paused*
    - *gui*
    - *verbose*
    - *debug*
    - *world_name* - world file name with respect to GAZEBO_RESOURCE_PATH
* spawn_object.launch - runs script that spawns object in Gazebo; arguments:
    - *model* - name of model
    - *x* - position x coordinate
    - *y* - position y coordinate
    - *z* - position z coordinate
    - *GAZEBO_MODEL_PATH* - please refer to Gazebo documentation
* gazebo_publish_ros_tf_object.launch - runs ROS node that publisher pose of a specified object to tf ROS topic; arguments:
    - *link_name* - name of link in Gazebo simulator, usually `model_name::link_name`
    - *frame_id* - frame name to be published

# rcprg_gazebo_utils {#rcprg_gazebo_utils_readme}

This package provides utilities for Gazebo simulator.

Launch files:
* gazebo_client.launch - runs Gazebo client; please note that you should provide the same paths for resources as for the server using ROS launch arguments:
    - *GAZEBO_MODEL_PATH*
    - *GAZEBO_RESOURCE_PATH*
* gazebo_world_editor.launch - runs Gazebo server and client; can be used as world editor; use the following arguments to set resource paths:
    - *GAZEBO_MODEL_PATH*
    - *GAZEBO_RESOURCE_PATH*
* spawn_object.launch - runs script that spawns object in Gazebo; arguments:
    - *model* - name of model
    - *x* - position x coordinate
    - *y* - position y coordinate
    - *z* - position z coordinate
    - *GAZEBO_MODEL_PATH*


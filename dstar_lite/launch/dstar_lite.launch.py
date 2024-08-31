import os
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, LogInfo, EmitEvent
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg
import launch.events
import launch_ros.event_handlers
from ament_index_python.packages import get_package_share_directory
import sys

def generate_launch_description():
    ld = LaunchDescription()

    # Define LifecycleNode for dstar_lite
    dstar_lite_node = LifecycleNode(
        package="dstar_lite",
        executable="dstar_node",
        name="dstar_lite",
        namespace='',
        output="both",
        emulate_tty=True,
    )

    # When the dstar_lite reaches the 'inactive' state, make it take the 'activate' transition.
    register_event_handler_for_dstar_lite_reaches_inactive_state = RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=dstar_lite_node, goal_state='inactive',
            entities=[
                LogInfo(
                    msg="node 'dstar_lite' reached the 'inactive' state, 'activating'."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(dstar_lite_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # Make the dstar_lite node take the 'configure' transition.
    emit_event_to_request_that_dstar_lite_does_configure_transition = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(dstar_lite_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Add the actions to the launch description.
    ld.add_action(register_event_handler_for_dstar_lite_reaches_inactive_state)
    ld.add_action(dstar_lite_node)
    ld.add_action(emit_event_to_request_that_dstar_lite_does_configure_transition)

    return ld

if __name__ == '__main__':
    generate_launch_description()

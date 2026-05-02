import bosdyn.client

if __name__ == "__main__":
    sdk = bosdyn.client.create_standard_sdk('understanding-spot')
    robot = sdk.create_robot('10.0.0.3')
    # robot.time_sync.wait_for_sync()
    robot.authenticate('user','scgau6g5w987')
    state_client = robot.ensure_client('robot-state')
    robot_state = state_client.get_robot_state()
    print(robot_state)

    snapshot = robot_state.kinematic_state.transforms_snapshot
    frame_names =bosdyn.client.frame_helpers.get_frame_names(snapshot)
    print(frame_names)

    for transform in robot_state.kinematic_state.transforms_snapshot.child_to_parent_edge_map: 
        print(f"Child: {transform} -> Parent: {robot_state.kinematic_state.transforms_snapshot.child_to_parent_edge_map[transform].parent_frame_name}")
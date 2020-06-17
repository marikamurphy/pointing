import rospy
import actionlib
import numpy as np
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from tmc_manipulation_msgs.srv import (
    FilterJointTrajectory,
    FilterJointTrajectoryRequest,
    FilterJointTrajectoryWithConstraints,
    FilterJointTrajectoryWithConstraintsRequest,
    SelectConfig,
    SelectConfigRequest,
)

CONFIG = {
    "joint_states_topic": "/hsrb/joint_states",
    "controller_template": "%s_trajectory_controller",
    "topic_template": "/hsrb/%s_trajectory_controller/follow_joint_trajectory",
    "valid_joints": {
        "head": ["head_tilt_joint", "head_pan_joint"],
        "arm": ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    },
    "joint_limits": {
        "head_pan_joint": [-3.87, 1.75]
    }
}

"""
Exceptions
"""
class TimeoutException(Exception):
    pass
class JointLimitException(Exception):
    pass


"""
A groovy whole_body controller that allows parallel motion (e.g. base can move alongside head) and returns futures
"""
class GroovyMotion(object):

    def __init__(self):
        # We want the clients for all valid_joints
        self.clients = {joint: actionlib.SimpleActionClient(CONFIG['topic_template'] % joint, FollowJointTrajectoryAction) for joint in CONFIG['valid_joints'].keys()}

        # Wait on all the action clients
        for c in self.clients.values():
            c.wait_for_server()

        # Wait for the controllers to run
        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        self.list_controllers = rospy.ServiceProxy('/hsrb/controller_manager/list_controllers', ListControllers)
        controllers_whitelist = [CONFIG['controller_template'] % j for j in CONFIG['valid_joints'].keys()]
        all_running = lambda: all([c.state == 'running' for c in filter(lambda c: c.name in controllers_whitelist, self.list_controllers().controller)])
        while not all_running() and not rospy.is_shutdown():
            rospy.sleep(0.1)

        # Receive joint states
        self.joint_states = None
        def _cb(data):
            self.joint_states = data
        rospy.Subscriber(CONFIG['joint_states_topic'], JointState, _cb)
        # Wait for a joint state to come through
        while self.joint_states is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
    def configure_compliance(self, setting): 
            setter_service = rospy.ServiceProxy("/hsrb/impedance_control/select_config", SelectConfig)
            req = SelectConfigRequest()
            req.name = setting
            try:
                res = setter_service.call(req)
                if not res.is_success:
                    msg = "Failed to set impedance config"
                    raise
            except rospy.ServiceException:
                import traceback
                traceback.print_exc()
                raise


    """Move joints to neutral(initial) pose of a robot."""
    def unsafe_move_to_neutral(self):
        goals = {
            'arm_lift_joint': 0.0,
            'arm_flex_joint': 0.0,
            'arm_roll_joint': 0.0,
            'wrist_flex_joint': -1.57,
            'wrist_roll_joint': 0.0,
            'head_pan_joint': 0.0,
            'head_tilt_joint': 0.0,
        }
        return self.move_to_joint_positions(goals)


    """Move arm joints to neutral(initial) pose of a robot."""
    def unsafe_move_arm_to_neutral(self):
        goals = {
            'arm_lift_joint': 0.0,
            'arm_flex_joint': 0.0,
            'arm_roll_joint': 0.0,
            'wrist_flex_joint': -1.57,
            'wrist_roll_joint': 0.0,
        }
        return self.move_to_joint_positions(goals)


    def move_to_joint_positions(self, goals):
        trajectory_point = goals.copy()
        trajectory_point['time'] = 0.1
        trajectory = [trajectory_point]
        return self.execute_trajectory(trajectory)


    """
    Moves through a sequence of joint positions.
    Format must be as follows:
    [{arm_lift_joint: blah, etc..., time: seconds},
     ...
    ]
    where time is the minimum amount of time a point in the
    trajectory should take
    """
    def execute_trajectory(self, trajectory):
        futures = []

        # Create separate trajectories for arm, head, etc.
        for joint in CONFIG['valid_joints'].keys():
            # First set target_state to the current joint state
            target_state = {j: self.joint_states.position[self.joint_states.name.index(j)] for j in CONFIG['valid_joints'][joint]}

            traj = JointTrajectory()
            traj.joint_names = CONFIG['valid_joints'][joint]

            for goals in trajectory:

                # Filter out goals for this joint
                valid_goals = {name: val for name, val in goals.iteritems() if name in CONFIG['valid_joints'][joint]}

                if valid_goals:
                    # Modify the joint positions we want to change
                    for name, pos in valid_goals.iteritems():
                        target_state[name] = pos

                    # Extract out joint goal positions in the 'valid_joints' order
                    target_positions = [target_state[j] for j in CONFIG['valid_joints'][joint]]

                    # Calculate time from start based on previous point in trajectory
                    try:
                        time = traj.points[-1].time_from_start + rospy.Duration(goals['time'])
                    except IndexError:
                        time = rospy.Time(goals['time'])

                    p = JointTrajectoryPoint()
                    p.positions = target_positions
                    p.velocities = [0]*len(target_positions)
                    p.time_from_start = time
                    traj.points.append(p)

            if len(traj.points) > 0:
                goal = FollowJointTrajectoryGoal()
                goal.trajectory = traj
                self.clients[joint].send_goal(goal)
                futures.append(self.__construct_future(self.clients[joint], CONFIG['valid_joints'][joint], traj.points[-1].positions))

        return lambda timeout, soft_wait=False: [f(timeout, soft_wait) for f in futures]


    def __construct_future(self, client, joint_names, target_positions):
        def wait(timeout, soft_wait=False):
            try:
                start_time = rospy.Time.now()
                end_time = start_time + rospy.Duration(timeout)

                # wait_for_result says success WAY too early
                # so this is just to make sure trajectory did not fail
                while not client.wait_for_result(rospy.Duration(0.01)):
                    if rospy.Time.now() >= end_time:
                        break

                state = client.get_state()
                if state != actionlib.GoalStatus.SUCCEEDED:

                    # Bizarre case where aborted status but goal actually succeeded...
                    if state == actionlib.GoalStatus.ABORTED:
                        rospy.logerr('Groovy goal ABORTED, but pretending like it succeeded...')
                    elif not soft_wait:
                        error = 'Failed to reach goal (%d: %s)' % (state, client.get_goal_status_text())
                        raise RuntimeError(error)

                # Clip target positions by joint limits
                clipped_target = [target_positions[i] if CONFIG['joint_limits'].get(j) is None else
                        min(max(target_positions[i], CONFIG['joint_limits'].get(j)[0]), CONFIG['joint_limits'].get(j)[1])
                        for i, j in enumerate(joint_names)]
                over_joint_limit = clipped_target != target_positions

                # If "SUCCEEDED" the real wait is here
                norm = float('inf')
                while norm > 0.1:
                    current_positions = [self.joint_states.position[self.joint_states.name.index(j)] for j in joint_names]
                    norm = np.linalg.norm([a - b for a,b in zip(current_positions, clipped_target)])
                    rospy.sleep(0.05)

                    if rospy.Time.now() >= end_time:
                        if soft_wait:
                            return False
                        else:
                            client.cancel_goal()
                            raise TimeoutException("GroovyMove timed out!")

                if over_joint_limit:
                    raise JointLimitException("Had to clip!")

                return True

            # Cancel on Ctrl-C
            except KeyboardInterrupt:
                client.cancel_goal()

        return wait
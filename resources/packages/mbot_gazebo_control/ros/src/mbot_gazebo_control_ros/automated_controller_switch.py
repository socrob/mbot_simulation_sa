#!/usr/bin/env python

'''
This node automatically switches between position, velocity and trajectory ctrl (mbot sim)
to match behavior with real robot, where all controllers are running at the same time

NOTE: the more complex part is with the traj ctrler, let us explain steps:
1. this node offers an actionlib server with what we call a real robot joint action traj crtler
2. upon receiving a request we check if traj ctrler is running and if not then we switch to it
3. we relay/re-send the received goal to the sim controller (based on ros control)

This makes the simulator equal in behavior to the real robot, where no switching between ctrlers is needed

Author: Oscar Lima (olima@isr.tecnico.ulisboa.pt)

'''

import rospy
import actionlib

from std_msgs.msg import Float64
from controller_manager_msgs.srv import ListControllers, SwitchController, SwitchControllerRequest
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult

class AutomatedCtrlSwitch(object):
    '''
    This node is to match mbot sim behavior with real robot
    In real robot both arm and position ctrl are simultaneously running
    In simulation only one at a time can be running

    This node subscribes to the position ctrl command, queries if position ctrl is running
    and if it is then pipes the command to the sim driver (ros control), if not then
    first switches controller and then pipes the command
    '''
    def __init__(self):
        # subscriptions
        self.pub_joint_array = []
        # callback array
        position_cb_array = [self.posCtrlCB0, self.posCtrlCB1, self.posCtrlCB2, self.posCtrlCB3, self.posCtrlCB4, self.posCtrlCB5, self.posCtrlCB6]
        velocity_cb_array = [self.velCtrlCB0, self.velCtrlCB1, self.velCtrlCB2, self.velCtrlCB3, self.velCtrlCB4, self.velCtrlCB5, self.velCtrlCB6]
        for i in range(0, 7):
            rospy.Subscriber('/left_arm_joint' + str(i) + '_position_controller/command', Float64, position_cb_array[i], queue_size=1)
            rospy.Subscriber('/left_arm_joint' + str(i) + '_velocity_controller/command', Float64, velocity_cb_array[i], queue_size=1)
        # prepare srv to list controllers (will also inform their status : running or stopped)
        self.list_ctrler_srv = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)
        # to control the frequency of the node
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))
        rospy.loginfo("Automated controller switch node started")
        # set flag to low at start
        self.position_ctrl_cmd_received = False
        self.velocity_ctrl_cmd_received = False
        # by default trajectory ctler is running
        self.running_ctler = 'trajectory'
        # prepare service to switch between controllers
        self.switch_ctrler_srv = rospy.ServiceProxy ('/controller_manager/switch_controller', SwitchController)
        # controller list
        position_ctrl_list = ['left_arm_joint0_position_controller', 'left_arm_joint1_position_controller', \
            'left_arm_joint2_position_controller', 'left_arm_joint3_position_controller', 'left_arm_joint4_position_controller',\
            'left_arm_joint5_position_controller', 'left_arm_joint6_position_controller']
        velocity_ctrl_list = ['left_arm_joint0_velocity_controller', 'left_arm_joint1_velocity_controller', \
            'left_arm_joint2_velocity_controller', 'left_arm_joint3_velocity_controller', 'left_arm_joint4_velocity_controller',\
            'left_arm_joint5_velocity_controller', 'left_arm_joint6_velocity_controller']
        # prepare list of controllers to start, stop when x ctrler is needed
        self.position_ctrler_start_stop_list = [position_ctrl_list, velocity_ctrl_list + ['sim_left_arm_traj_controller']]
        self.velocity_ctrler_start_stop_list = [velocity_ctrl_list, position_ctrl_list + ['sim_left_arm_traj_controller']]
        self.trajectory_ctrler_start_stop_list = [['sim_left_arm_traj_controller'], position_ctrl_list + velocity_ctrl_list]
        # count to every once in a while read the controller status
        self.count = 0
        # query at node startup which controller is running
        rospy.wait_for_service('/controller_manager/list_controllers', 10.0)
        rospy.logdebug('Service /controller_manager/list_controllers found, proceed')
        self.update_running_ctler()
        # inform user about the active ctrler
        rospy.loginfo('Detected controller running : %s', self.running_ctler)

        # traj ctrler automated switching

        # spawn an intermediate ctrler equal to real robot, catch the goal, switch to trajectory ctrler and re-send goal
        self.action_server = actionlib.SimpleActionServer('left_arm_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction, execute_cb=self.process_follow_trajectory, auto_start=False)
        self.action_server.start()
        # create action lib client
        self.client = actionlib.SimpleActionClient('sim_left_arm_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.client.wait_for_server()
        rospy.loginfo('Action server left_arm_traj_controller is up !')
        # maximum expected arm trajectory execution time
        self.arm_timeout = rospy.get_param('~arm_timeout', 10.0)


    def process_follow_trajectory(self, goal):
        '''
        This callback gets executed upon receiving a goal for the real arm trajectory controller
        '''
        rospy.loginfo('Real joint action trajectory goal received')

        # check if traj ctrler is running
        if self.running_ctler != 'trajectory':
            rospy.loginfo('Trajectory ctrl cmd received, but trajectory controller is not running, will try to switch now')
            self.switch_to_traj_ctrl()

        rospy.loginfo('Relaying action lib goal to left_arm_traj_controller')
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(int(self.arm_timeout)))
        # get result
        result = self.client.get_result()

        if result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
            rospy.logdebug("Follow joint trajectory server responded with success")
            self.action_server.set_succeeded(result=result, text='success')
        else:
            rospy.logerr("Follow joint trajectory server responded with error")
            self.action_server.set_aborted(result=result, text='error')


    def update_running_ctler(self):
        '''
        Query which controller is running
        Output: position, velocity or trajectory
        It assumes that if joint0 is in either position or control mode all joints are as well
        '''
        # call service list controllers (offered by controller manager) and analyze its response
        # to determine which controller is running
        for controller in self.list_ctrler_srv().controller:
            if controller.name == 'left_arm_traj_controller':
                if controller.state == 'running':
                    self.running_ctler = 'trajectory'
            elif controller.name == 'left_arm_joint0_position_controller':
                if controller.state == 'running':
                    self.running_ctler = 'position'
            elif controller.name == 'left_arm_joint0_velocity_controller':
                if controller.state == 'running':
                    self.running_ctler = 'velocity'
        # if no ctrler is running warn user
        if self.running_ctler == None:
            rospy.logerr('no controllers are running: position, velocity or trajectory, node should not reach this state')
            # depending upon situation if no controller is running we could switch to trajectory
            # for now we will not do it because this state actually should be impossible to reach
            # maybe killing the node and doing some intense debugging on why this state is reached


    def switch_to_pos_ctrl(self):
        '''
        call service /controller_manager/switch_controller
        start position ctrl, stop velocity + trajectory ctrl
        '''
        req = SwitchControllerRequest()
        req.start_controllers = self.position_ctrler_start_stop_list[0]
        req.stop_controllers = self.position_ctrler_start_stop_list[1]
        req.strictness = req.BEST_EFFORT
        self.switch_ctrler_srv(req)
        # to keep in memory the running controller
        self.running_ctler = 'position'
        rospy.loginfo('Automatically switched to position ctrl')


    def switch_to_vel_ctrl(self):
        '''
        call service /controller_manager/switch_controller
        start vel ctrl, stop position + trajectory ctrl
        '''
        req = SwitchControllerRequest()
        req.start_controllers = self.velocity_ctrler_start_stop_list[0]
        req.stop_controllers = self.velocity_ctrler_start_stop_list[1]
        req.strictness = req.BEST_EFFORT
        self.switch_ctrler_srv(req)
        # to keep in memory the running controller
        self.running_ctler = 'velocity'
        rospy.loginfo('Automatically switched to velocity ctrl')


    def switch_to_traj_ctrl(self):
        '''
        call service /controller_manager/switch_controller
        start trajectory ctrl, stop position + velocity ctrl
        '''
        req = SwitchControllerRequest()
        req.start_controllers = self.trajectory_ctrler_start_stop_list[0]
        req.stop_controllers = self.trajectory_ctrler_start_stop_list[1]
        req.strictness = req.BEST_EFFORT
        self.switch_ctrler_srv(req)
        # to keep in memory the running controller
        self.running_ctler = 'trajectory'
        rospy.loginfo('Automatically switched to trajectory ctrl')


    def start_automated_controller_switch(self):
        '''
        node main loop
        '''
        while not rospy.is_shutdown():
            self.count += 1 
            if self.position_ctrl_cmd_received:
                # lower flag
                self.position_ctrl_cmd_received = False
                # check position control status and if position ctrler is not running then switch to it
                if self.running_ctler != 'position':
                    rospy.loginfo('position ctrl cmd received, but position controller is not running, switching now automatically')
                    self.switch_to_pos_ctrl()
            elif self.velocity_ctrl_cmd_received:
                # lower flag
                self.velocity_ctrl_cmd_received = False
                # check position control status and if velocity ctrler is not running then switch to it
                if self.running_ctler != 'velocity':
                    rospy.loginfo('velocity ctrl cmd received, but velocity controller is not running, switching now automatically')
                    self.switch_to_vel_ctrl()

            # update controller status once in a while (to account for user switching ctrler manually)
            if self.count > 100:
                self.count = 0
                self.update_running_ctler()
            self.loop_rate.sleep()


    def posCtrlCB0(self, msg):
        self.position_ctrl_cmd_received = True

    def posCtrlCB1(self, msg):
        self.position_ctrl_cmd_received = True

    def posCtrlCB2(self, msg):
        self.position_ctrl_cmd_received = True
        
    def posCtrlCB3(self, msg):
        self.position_ctrl_cmd_received = True
        
    def posCtrlCB4(self, msg):
        self.position_ctrl_cmd_received = True
        
    def posCtrlCB5(self, msg):
        self.position_ctrl_cmd_received = True
        
    def posCtrlCB6(self, msg):
        self.position_ctrl_cmd_received = True

    def velCtrlCB0(self, msg):
        self.velocity_ctrl_cmd_received = True

    def velCtrlCB1(self, msg):
        self.velocity_ctrl_cmd_received = True

    def velCtrlCB2(self, msg):
        self.velocity_ctrl_cmd_received = True

    def velCtrlCB3(self, msg):
        self.velocity_ctrl_cmd_received = True

    def velCtrlCB4(self, msg):
        self.velocity_ctrl_cmd_received = True
        
    def velCtrlCB5(self, msg):
        self.velocity_ctrl_cmd_received = True
        
    def velCtrlCB6(self, msg):
        self.velocity_ctrl_cmd_received = True


def main():
    rospy.init_node('mbot_switch_ctrl_node', anonymous=False)
    mbot_switch_ctrl_node = AutomatedCtrlSwitch()
    mbot_switch_ctrl_node.start_automated_controller_switch()

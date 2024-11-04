import rospy

from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest


class ControllerManagerClient(object):
    def __init__(self):
        rospy.wait_for_service("controller_manager/switch_controller")
        self.switch_controller_client = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController
        )

    def switch_controller(self, start_controllers, stop_controllers):
        try:
            req = SwitchControllerRequest()
            req.start_controllers = start_controllers
            req.stop_controllers = stop_controllers
            req.strictness = 1
            self.switch_controller_client.call(req)
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

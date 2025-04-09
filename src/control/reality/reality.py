import time
import threading
import rospy
import math
from geometry_msgs.msg import TwistStamped, PoseStamped
from transforms3d.euler import quat2euler
import numpy as np
from include.rflysim.PX4MavCtrlV4 import PX4MavCtrler
import matplotlib.pyplot as plt
import subprocess

def get_command_output(cmd):
    result = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return result.stdout.decode('utf-8')

def limit_yaw(yaw):
    res = yaw
    if math.fabs(res) > math.pi:
        if res > 0:
            res = res - 2 * math.pi
        else:
            res = 2 * math.pi + res
    return res

def ping_website(website):  
    cmd = ["ping", "-c", "1", website]
    try:  
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)  
        stdout, stderr = process.communicate()  
        return process.returncode == 0  
    except Exception as e:    
        return False

class RflyDT:
    def __init__(self, ip, port, name, openloop):
        """
        port: 飞机端口号
        name: 动捕刚体名称
        openloop: 为True表示开环控制，即动捕消息不发送给飞机，对飞机的控制也不依赖飞机实时返回的信息
        """
        self.name = name
        self.ID = int(port) - 15500
        self.port = str(port)
        self.ip = ip
        self.openloop = openloop
        self.pose_topic = "/vrpn_client_node/" + self.name + "/pose"
        self.twist_topic = "/vrpn_client_node/" + self.name + "/twist"

        self.pos = np.array([0.0, 0.0, 0.0])
        self.vel = np.array([0.0, 0.0, 0.0])
        self.yaw = 0
        self.px4_yaw = 0
        self.offset_yaw = 0
        self.init_px4_yaw = 0
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.ini_yaw_success = False
        self.yaw_err = 0


        self.pose_rec_task = None
        self.twist_rec_task = None

        self.mav = PX4MavCtrler(self.ID, self.ip, 'direct:' + self.port)

        self.sum_dt = 0.0
        self.cnt_dt = 0.0

        self.check_ok = self.check_status()

    def check_status(self):
        res = get_command_output('rostopic list')
        if self.pose_topic not in res:
            print("Error, no pose_topic")
            return False
        if self.twist_topic not in res:
            print("Error, no twist_topic")
            return False
        if not ping_website(self.ip):
            print('{} unreachable'.format(self.ip))
            return False
        return True

    def init_connection(self):
        if not self.check_ok:
            return
        
        self.pose_rec_task = threading.Thread(target=self.update_pose, args=())
        self.twist_rec_task = threading.Thread(target=self.update_twist, args=())
        self.pose_rec_task.start()
        self.twist_rec_task.start()

        self.mav.InitMavLoop(UDPMode=2)
        time.sleep(1)

        self.init_yaw()
    
        self.mav.initOffboard()
        time.sleep(1)
    
    def init_yaw(self):    
        self.offset_yaw = self.yaw - self.px4_yaw
        self.px4_yaw_d = self.px4_yaw
        self.init_px4_yaw = self.px4_yaw
        self.ini_yaw_success = True
    
    def send_vision_capture(self):
        if not self.ini_yaw_success or self.openloop:
            return
        yaw = -self.offset_yaw - self.yaw
        yaw = limit_yaw(yaw)
        yaw_err =  self.offset_yaw
        x = self.pos[0] * math.cos(yaw_err) - self.pos[1] * math.sin(yaw_err)
        y = -self.pos[1] * math.cos(yaw_err) - self.pos[0] * math.sin(yaw_err)
        z = -self.pos[2]
        self.mav.send_vision_position(x, y, z, yaw)
        self.cnt = self.cnt + 1
        if self.cnt > 100:
            print(f"send_vision_capture{time.time()}")
            self.cnt = 0

    def update_pose(self):
        while True:
            if self.should_exit:
                break
            msg = rospy.wait_for_message(
                self.pose_topic, PoseStamped, timeout=None)    
            self.pos[0] = msg.pose.position.x
            self.pos[1] = msg.pose.position.y
            self.pos[2] = msg.pose.position.z
            self.quat[0] = msg.pose.orientation.w
            self.quat[1] = msg.pose.orientation.x
            self.quat[2] = msg.pose.orientation.y
            self.quat[3] = msg.pose.orientation.z
            [roll, pitch, self.yaw] = quat2euler(self.quat)
            self.px4_yaw = self.mav.uavAngEular[2]
            # self.battery = self.mav.batInfo[0]
            self.battery = 50

            offset = self.yaw + self.offset_yaw
            self.offset = limit_yaw(offset)

            self.send_vision_capture()
            
    def update_twist(self):
        while True:
            if self.should_exit:
                break
            msg = rospy.wait_for_message(
                self.twist_topic, TwistStamped, timeout=None)
            
            self.vel[0] = msg.twist.linear.x
            self.vel[1] = msg.twist.linear.y
            self.vel[2] = msg.twist.linear.z
    
    def run(self):
        self.start_time = time.time()
        while True:
            cnt = time.time() - self.start_time
            print(f"{round(cnt)}s: pos:{self.pos[0]:.2f}, {self.pos[1]:.2f}, {self.pos[2]:.2f}")
            print(f"{round(cnt)}s: vel:{self.vel[0]:.2f}, {self.vel[1]:.2f}, {self.vel[2]:.2f}")
            time.sleep(1)

    def stop(self):
        self.should_exit = True
        self.mav.stopRun()


if __name__ == "__main__":

    rospy.init_node('RflyVel', anonymous=False)

    '''
    drones = [
        [False,'192.168.151.101', 15501, 'droneyee01', False],
        [False, '192.168.151.105', 15505, 'droneyee05', False],
        [True,'192.168.151.103', 15503, 'droneyee03', False],
        [False,'192.168.151.109', 15509, 'droneyee09', True]
    ]
    '''
    drones = [
        [True,'192.168.151.103', 15503, 'droneyee03', False]
    ]
    mav_controls = []
    for drone in drones:
        if not drone[0]:
            continue
        mav = RflyDT(drone[1], drone[2], drone[3], drone[4])
        mav.init_connection()
        mav_controls.append(mav)
    
    time.sleep(5)
    for mav in mav_controls:
        mav.run()
    
    print("Press any key to exit")
    input()

    for mav in mav_controls:
        mav.stop()

    rospy.signal_shutdown('Closing the node')
    time.sleep(5)

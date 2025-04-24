#!/usr/bin/env python3

import rospy
import subprocess
import signal
import os
from std_srvs.srv import Trigger, TriggerResponse
from web_controller.srv import MapOperation, MapOperationResponse
import glob

class LaunchController:
    def __init__(self):
        self.active_process = None  # 存储当前激活的launch进程

    def start_launch(self, pkg, launch_file):
        if self.is_running():
            return False
        cmd = ["roslaunch", pkg, launch_file]
        self.active_process = subprocess.Popen(cmd, preexec_fn=os.setsid)
        return True

    def stop_launch(self):
        if self.is_running():
            os.killpg(os.getpgid(self.active_process.pid), signal.SIGINT)
            self.active_process = None
            return True
        return False

    def is_running(self):
        return self.active_process and (self.active_process.poll() is None)


def handle_start(req):
    success = controller.start_launch("demo_gmapping_slam", "rplidar_hector.launch")
    msg = "Mapping started" if success else "Mapping already running"
    return TriggerResponse(success=success, message=msg)

def handle_stop(req):
    success = controller.stop_launch()
    msg = "Mapping stopped" if success else "No active mapping process"
    return TriggerResponse(success=success, message=msg)

def handle_status(req):
    is_running = controller.is_running()
    return TriggerResponse(success=is_running, message=str(is_running))

def handle_map_operation(req):
    try:
        if req.operation == "save":
            save_path = os.path.expanduser(f"~/map/{req.map_name}")
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            cmd = ["rosrun", "map_server", "map_saver", "-f", save_path]
            subprocess.run(cmd, check=True)
            return MapOperationResponse(success=True, map_list=[], message=f"地图已保存为 {req.map_name}")
            
        elif req.operation == "list":
            # 获取所有地图文件
            map_path = os.path.expanduser("~/map")
            maps = glob.glob(f"{map_path}/*.yaml")
            map_names = [os.path.splitext(os.path.basename(m))[0] for m in maps]
            return MapOperationResponse(success=True, map_list=map_names, message="获取地图列表成功")
            
        elif req.operation == "load":
            # TODO: 实现地图加载功能
            pass
            
    except Exception as e:
        return MapOperationResponse(success=False, map_list=[], message=str(e))

if __name__ == "__main__":
    rospy.init_node("control_node")
    controller = LaunchController()
    
    # 创建服务
    start_service = rospy.Service("/start_mapping", Trigger, handle_start)
    stop_service = rospy.Service("/stop_mapping", Trigger, handle_stop)
    status_service = rospy.Service("/mapping_status", Trigger, handle_status)
    map_service = rospy.Service("/map_operation", MapOperation, handle_map_operation)
    
    rospy.loginfo("Node controller ready")
    rospy.spin()
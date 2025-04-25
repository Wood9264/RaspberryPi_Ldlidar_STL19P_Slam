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
        self.map_server_process = None  # 存储地图服务器进程

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

    def start_map_server(self, yaml_path):
        try:
            # 先确保清理所有旧的 map_server 进程
            self.stop_map_server()
            
            # 启动新的地图服务器
            cmd = ["rosrun", "map_server", "map_server", yaml_path]
            self.map_server_process = subprocess.Popen(cmd,
                                                     stdout=subprocess.PIPE,
                                                     stderr=subprocess.PIPE,
                                                     preexec_fn=os.setsid)
            
            # 等待确认进程启动
            rospy.sleep(0.5)
            if self.map_server_process.poll() is not None:
                raise Exception("地图服务器启动失败")
                
            return True
            
        except Exception as e:
            rospy.logerr(f"启动地图服务器失败: {str(e)}")
            return False

    def stop_map_server(self):
        try:
            # 终止当前跟踪的进程
            if self.map_server_process:
                try:
                    os.killpg(os.getpgid(self.map_server_process.pid), signal.SIGTERM)
                    self.map_server_process.wait(timeout=3)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    try:
                        os.killpg(os.getpgid(self.map_server_process.pid), signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                self.map_server_process = None
            
            # rospy.sleep(1)  # 等待节点完全关闭
            
            # 执行清理
            cleanup_cmd = "echo y | rosnode cleanup"
            subprocess.run(cleanup_cmd, 
                        shell=True,
                        check=False, 
                        stderr=subprocess.PIPE)
                
        except Exception as e:
            rospy.logerr(f"停止地图服务器时出错: {str(e)}")

def handle_start(req):
    try:
        controller.stop_map_server()  # 确保地图服务器已停止
        
        # 启动建图
        success = controller.start_launch("demo_gmapping_slam", "rplidar_hector.launch")
        msg = "Mapping started" if success else "Mapping already running"
        return TriggerResponse(success=success, message=msg)
    except Exception as e:
        return TriggerResponse(success=False, message=str(e))

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
            # 先检查是否正在建图
            if controller.is_running():
                return MapOperationResponse(
                    success=False,
                    map_list=[],
                    message="无法加载地图：建图功能正在运行"
                )
            # 检查地图文件是否存在
            map_path = os.path.expanduser(f"~/map/{req.map_name}")
            yaml_path = f"{map_path}.yaml"
            if not os.path.exists(yaml_path):
                return MapOperationResponse(
                    success=False,
                    map_list=[],
                    message=f"地图文件 {yaml_path} 不存在"
                )
            
            try:
                if controller.start_map_server(yaml_path):
                    return MapOperationResponse(
                        success=True,
                        map_list=[],
                        message=f"地图 {req.map_name} 加载成功"
                    )
                else:
                    return MapOperationResponse(
                        success=False,
                        map_list=[],
                        message="启动地图服务器失败"
                    )
                
            except Exception as e:
                return MapOperationResponse(
                    success=False,
                    map_list=[],
                    message=f"加载地图失败: {str(e)}"
                )
            
    except Exception as e:
        return MapOperationResponse(success=False, map_list=[], message=str(e))

if __name__ == "__main__":
    rospy.init_node("node_controller")
    controller = LaunchController()
    
    # 创建服务
    start_service = rospy.Service("/start_mapping", Trigger, handle_start)
    stop_service = rospy.Service("/stop_mapping", Trigger, handle_stop)
    status_service = rospy.Service("/mapping_status", Trigger, handle_status)
    map_service = rospy.Service("/map_operation", MapOperation, handle_map_operation)
    
    rospy.loginfo("Node controller ready")
    
    def shutdown_hook():
        controller.stop_map_server()
        controller.stop_launch()
    
    rospy.on_shutdown(shutdown_hook)
    rospy.spin()
#!/usr/bin/env python3

import os
import http.server
import socketserver
import rospy
import signal
import sys
import threading
import socket
import time
import netifaces  # 需要安装: pip install netifaces

class ThreadedHTTPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    """处理请求的线程化版本的HTTP服务器"""
    daemon_threads = True  # 使用守护线程处理请求
    allow_reuse_address = True  # 允许地址重用

class GracefulHTTPServer:
    def __init__(self):
        self.server_running = False  # 初始化时设置为False
        self.server_thread = None
        self.httpd = None
        
        try:
            # 获取包路径
            package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            self.html_dir = os.path.join(package_path, "html")
            
            # 检查html目录是否存在
            if not os.path.exists(self.html_dir):
                rospy.logerr(f"HTML目录不存在: {self.html_dir}")
                return
                
            rospy.loginfo(f"提供的HTML目录: {self.html_dir}")
            
            # 切换到HTML目录
            os.chdir(self.html_dir)
            
            # 设置服务器端口
            self.PORT = 8000
            self.Handler = http.server.SimpleHTTPRequestHandler
            
            # 检查端口是否已被占用
            self._ensure_port_available()
            
            # 创建线程化服务器
            self.httpd = ThreadedHTTPServer(("", self.PORT), self.Handler)
            
        except Exception as e:
            rospy.logerr(f"初始化HTTP服务器时出错: {str(e)}")
            self.httpd = None
    
    def _ensure_port_available(self):
        """确保端口可用，如果不可用则尝试杀死占用进程"""
        try:
            # 尝试绑定端口，检查是否可用
            test_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            test_socket.settimeout(1)
            
            # SO_REUSEADDR允许端口快速重用
            test_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            # 在某些OS上，还需要设置SO_REUSEPORT
            if hasattr(socket, 'SO_REUSEPORT'):
                test_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                
            test_socket.bind(("", self.PORT))
            test_socket.close()
            return True
        except socket.error:
            rospy.logwarn(f"端口 {self.PORT} 可能被占用，尝试释放...")
            
            # 在Linux系统上尝试使用系统命令终止占用端口的进程
            try:
                import subprocess
                cmd = f"lsof -ti:{self.PORT} | xargs kill -9"
                subprocess.run(cmd, shell=True)
                rospy.loginfo(f"已尝试释放端口 {self.PORT}")
                time.sleep(1)  # 等待端口释放
                
                # 再次检查端口
                test_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                test_socket.settimeout(1)
                test_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                if hasattr(socket, 'SO_REUSEPORT'):
                    test_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                test_socket.bind(("", self.PORT))
                test_socket.close()
                return True
            except Exception as e:
                rospy.logerr(f"无法释放端口 {self.PORT}: {str(e)}")
                # 尝试使用不同的端口
                for alt_port in range(8001, 8010):
                    try:
                        test_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        test_socket.bind(("", alt_port))
                        test_socket.close()
                        self.PORT = alt_port
                        rospy.loginfo(f"将使用替代端口: {self.PORT}")
                        return True
                    except:
                        continue
                
                rospy.logerr("无法找到可用端口，HTTP服务器将不会启动")
                return False
    
    def _get_ip_addresses(self):
        """获取所有网络接口的IP地址"""
        ip_addresses = []
        
        try:
            # 获取所有网络接口
            interfaces = netifaces.interfaces()
            
            for interface in interfaces:
                # 获取该接口的所有地址信息
                addrs = netifaces.ifaddresses(interface)
                
                # 只处理IPv4地址 (AF_INET)
                if netifaces.AF_INET in addrs:
                    for addr in addrs[netifaces.AF_INET]:
                        ip = addr['addr']
                        ip_addresses.append((interface, ip))
            
            return ip_addresses
            
        except Exception as e:
            rospy.logwarn(f"获取IP地址时出错: {str(e)}")
            # 如果netifaces出错，使用备用方法
            try:
                hostname = socket.gethostname()
                ip = socket.gethostbyname(hostname)
                ip_addresses.append(("default", ip))
                
                # 添加回环地址
                ip_addresses.append(("lo", "127.0.0.1"))
                
                return ip_addresses
            except Exception as e2:
                rospy.logerr(f"获取IP地址的备用方法也失败: {str(e2)}")
                return [("unknown", "获取IP地址失败")]
    
    def start(self):
        """在单独的线程中启动HTTP服务器"""
        if self.server_running or self.httpd is None:
            return
            
        self.server_running = True
        self.server_thread = threading.Thread(target=self._run_server)
        self.server_thread.daemon = True
        self.server_thread.start()
        
        # 获取所有IP地址并输出
        ip_addresses = self._get_ip_addresses()
        
        rospy.loginfo(f"HTTP服务器已启动，地址: 0.0.0.0:{self.PORT}")
        
        # 输出每个网卡的IP地址
        for interface, ip in ip_addresses:
            rospy.loginfo(f"请在浏览器中访问 http://{ip}:{self.PORT} ({interface})")
    
    def _run_server(self):
        """运行HTTP服务器的内部方法"""
        try:
            self.httpd.serve_forever()
        except Exception as e:
            if self.server_running:
                rospy.logerr(f"HTTP服务器运行时出错: {str(e)}")
    
    def shutdown(self):
        """优雅地关闭HTTP服务器"""
        if not self.server_running or self.httpd is None:
            return
            
        rospy.loginfo("正在关闭HTTP服务器...")
        
        # 标记为关闭状态
        self.server_running = False
        
        # 创建一个线程来关闭服务器，防止阻塞
        def shutdown_thread():
            try:
                # 关闭所有连接
                self.httpd.shutdown()
                self.httpd.server_close()
                rospy.loginfo("HTTP服务器已关闭")
            except Exception as e:
                rospy.logerr(f"关闭HTTP服务器时出错: {str(e)}")
        
        shutdown_thread = threading.Thread(target=shutdown_thread)
        shutdown_thread.daemon = True
        shutdown_thread.start()
        
        # 只等待很短的时间
        shutdown_thread.join(0.5)

def signal_handler(sig, frame):
    """处理信号以优雅地关闭"""
    rospy.loginfo("接收到终止信号，正在关闭...")
    rospy.signal_shutdown("用户中断")

if __name__ == "__main__":
    # 设置信号处理
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 初始化ROS节点
    rospy.init_node('web_control_server', anonymous=True)
    
    # 创建并启动HTTP服务器
    server = GracefulHTTPServer()
    server.start()
    
    # 添加关闭钩子
    rospy.on_shutdown(lambda: server.shutdown())
    
    # 保持节点运行，直到收到关闭信号
    rospy.spin()
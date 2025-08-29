#! /usr/bin/env python3 
# -*- coding: utf-8 -*-

# Description: monitor the alive status of vrpn clients
# Author: sujit-168 su2054552689@gmail.com

import asyncio
import subprocess
import rospy
from threading import Thread

class VRPNMonitor:
    def __init__(self):
        # 配置参数
        self.ping_target = rospy.get_param('~ping_target', '192.168.1.100')
        self.vrpn_topic = rospy.get_param('~vrpn_topic', '/vrpn_client_node/pose')
        self.check_interval = rospy.get_param('~check_interval', 2.0)
        self.ping_latency_threshold = rospy.get_param('~delay_threshold', 50)
        self.topic_frequency_threshold = rospy.get_param('~topic_hz_threshold', 20.0)
        
        # 统计相关
        self.is_network_connected = False
        self.is_passed = False
        self.window_size = 100
        self.delay_window = []  # 滑动窗口队列
        self.total_checks = 0   # 总检测次数
        self.passed_checks = 0  # 达标次数

    async def ping_checker(self):
        """单次 PING 检测"""
        try:
            proc = await asyncio.create_subprocess_shell(
                f'ping -c 1 {self.ping_target}',
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            stdout, _ = await proc.communicate()
            if proc.returncode == 0 and stdout:
                output = stdout.decode().strip()
                if 'time=' in output:
                    delay = float(output.split('time=')[1].split()[0])
                    rospy.loginfo(f"[PING] 动捕设备通信正常 延迟={delay}ms")
                    if delay > self.ping_latency_threshold:
                        rospy.logwarn(f"[PING] 传输延迟大于 {self.ping_latency_threshold}ms, 延迟过高：{delay}ms, 请切换至低延迟网络")
                else:
                    rospy.loginfo(f"[PING] {self.ping_target} 可达但未返回延迟数据")
                self.is_network_connected = True
            else:
                self.is_network_connected = False
                rospy.logwarn(f"[PING] {self.ping_target} 通信中断，请检查与动捕系统之间的网络链路是否正常连通！")
        except Exception as e:
            rospy.logerr(f"[PING] Error: {str(e)}")

    async def topic_checker(self):
        """单次话题检测 (含滑动窗口统计)"""
        last_time = rospy.Time.now()
        try:
            msg = rospy.wait_for_message(
                self.vrpn_topic, 
                rospy.AnyMsg,
                timeout=self.check_interval
            )
            delay = (rospy.Time.now() - last_time).to_sec()
            freq = 1.0 / delay
            
            self.is_passed = (freq >= self.topic_frequency_threshold)
            if not self.is_passed:
                self.is_passed = False
                rospy.logwarn(f"[TOPIC] {self.vrpn_topic} 当前频率 {freq:.2f}Hz，延迟过高：{delay:.2f}s")
        except rospy.ROSException:
            self.is_passed = False
            rospy.logerr(f"[TOPIC] {self.vrpn_topic} timeout")
            if self.is_network_connected:
                rospy.logwarn("[TOPIC] 请检查或重启 VRPN 服务器节点，当前 ROS 节点通信异常")
        except Exception as e:
            self.is_passed = False
            rospy.logerr(f"[TOPIC] Error: {str(e)}")

    async def sliding_window(self):
        """达标率增量统计"""
            
        self.total_checks += 1
        if self.is_passed:
            self.passed_checks += 1
        
        # 计算达标率
        total_pass_rate = self.passed_checks / self.total_checks * 100
        
        rospy.loginfo(f"[TOPIC] 累计达标率={total_pass_rate:.1f}%({self.passed_checks}/{self.total_checks})")
    def run(self):
        """启动异步检测"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        async def task_runner():
            while not rospy.is_shutdown():
                # rospy.loginfo("[TASK] 开始 PING 检测")
                await self.ping_checker()
                # rospy.loginfo("[TASK] 开始 TOPIC 检测")
                await self.topic_checker()
                # rospy.loginfo("[TASK] 开始滑动窗口统计")
                await self.sliding_window()
                await asyncio.sleep(1)  # 防止 CPU 占用过高
        
        try:
            loop.run_until_complete(task_runner())
        except Exception as e:
            rospy.logerr(f"[MAIN] Async loop error: {str(e)}")
        finally:
            loop.close()

if __name__ == '__main__':
    rospy.init_node('vrpn_alive_monitor', anonymous=True)
    monitor = VRPNMonitor()
    
    # 在独立线程中运行异步循环
    Thread(target=monitor.run, daemon=True).start()
    rospy.spin()

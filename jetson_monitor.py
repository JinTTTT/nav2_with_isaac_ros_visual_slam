#!/usr/bin/env python3
"""
Jetson Performance Monitor for Nav2 System
监控Jetson设备运行Nav2时的CPU、GPU和内存性能
"""

import subprocess
import re
import time
import json
import signal
import sys
from datetime import datetime
from typing import Dict, List, Tuple
import statistics

class JetsonMonitor:
    def __init__(self, interval=1):
        """
        初始化监控器
        :param interval: 采样间隔(秒)
        """
        self.interval = interval
        self.data = []
        self.running = False
        
    def parse_tegrastats_line(self, line: str) -> Dict:
        """解析tegrastats输出的一行数据"""
        try:
            data = {}
            
            # 解析时间戳
            timestamp_match = re.search(r'^(\d{2}-\d{2}-\d{4} \d{2}:\d{2}:\d{2})', line)
            if timestamp_match:
                data['timestamp'] = timestamp_match.group(1)
            
            # 解析RAM使用情况 - RAM 2807/31011MB
            ram_match = re.search(r'RAM (\d+)/(\d+)MB', line)
            if ram_match:
                used_ram = int(ram_match.group(1))
                total_ram = int(ram_match.group(2))
                data['ram_used_mb'] = used_ram
                data['ram_total_mb'] = total_ram
                data['ram_usage_percent'] = (used_ram / total_ram) * 100
            
            # 解析CPU使用率 - CPU [5%@2265,7%@2265,...]
            cpu_match = re.search(r'CPU \[(.*?)\]', line)
            if cpu_match:
                cpu_data = cpu_match.group(1)
                cpu_cores = []
                for core in cpu_data.split(','):
                    core_match = re.search(r'(\d+)%@(\d+)', core)
                    if core_match:
                        usage = int(core_match.group(1))
                        freq = int(core_match.group(2))
                        cpu_cores.append({'usage': usage, 'freq': freq})
                
                data['cpu_cores'] = cpu_cores
                if cpu_cores:
                    data['cpu_avg_usage'] = sum(core['usage'] for core in cpu_cores) / len(cpu_cores)
                    data['cpu_avg_freq'] = sum(core['freq'] for core in cpu_cores) / len(cpu_cores)
            
            # 解析GPU频率 - GR3D_FREQ 0%
            gpu_match = re.search(r'GR3D_FREQ (\d+)%', line)
            if gpu_match:
                data['gpu_usage'] = int(gpu_match.group(1))
            
            # 解析温度 - CPU@51C GPU@51C
            cpu_temp_match = re.search(r'CPU@([\d.]+)C', line)
            if cpu_temp_match:
                data['cpu_temp'] = float(cpu_temp_match.group(1))
                
            gpu_temp_match = re.search(r'GPU@([\d.]+)C', line)
            if gpu_temp_match:
                data['gpu_temp'] = float(gpu_temp_match.group(1))
            
            return data
        except Exception as e:
            print(f"解析错误: {e}")
            return {}
    
    def start_monitoring(self, duration=None):
        """开始监控"""
        print(f"开始监控Jetson性能 (间隔: {self.interval}秒)")
        if duration:
            print(f"监控时长: {duration}秒")
        
        self.running = True
        self.data = []
        
        # 设置信号处理器
        def signal_handler(signum, frame):
            print("\n收到停止信号，正在保存数据...")
            self.running = False
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        # 启动tegrastats
        cmd = ['tegrastats', f'--interval', str(int(self.interval * 1000))]
        
        try:
            with subprocess.Popen(cmd, stdout=subprocess.PIPE, 
                                stderr=subprocess.PIPE, 
                                universal_newlines=True,
                                bufsize=1) as proc:
                
                start_time = time.time()
                
                while self.running:
                    if duration and (time.time() - start_time) > duration:
                        break
                        
                    line = proc.stdout.readline()
                    if line:
                        parsed_data = self.parse_tegrastats_line(line.strip())
                        if parsed_data:
                            self.data.append(parsed_data)
                            print(f"采集数据点: {len(self.data)}", end='\r')
                    
                    if proc.poll() is not None:
                        break
                
                proc.terminate()
                
        except Exception as e:
            print(f"监控过程中出错: {e}")
        
        print(f"\n监控结束，共收集 {len(self.data)} 个数据点")
        return self.data
    
    def generate_report(self) -> Dict:
        """生成性能统计报告"""
        if not self.data:
            return {"error": "没有可用数据"}
        
        report = {
            "监控概况": {
                "数据点数量": len(self.data),
                "监控时长": f"{len(self.data) * self.interval:.1f}秒",
                "采样间隔": f"{self.interval}秒"
            }
        }
        
        # RAM统计
        ram_usage = [d['ram_usage_percent'] for d in self.data if 'ram_usage_percent' in d]
        ram_used = [d['ram_used_mb'] for d in self.data if 'ram_used_mb' in d]
        
        if ram_usage:
            report["内存使用情况"] = {
                "平均使用率": f"{statistics.mean(ram_usage):.1f}%",
                "最大使用率": f"{max(ram_usage):.1f}%",
                "最小使用率": f"{min(ram_usage):.1f}%",
                "平均使用量": f"{statistics.mean(ram_used):.0f}MB",
                "最大使用量": f"{max(ram_used)}MB",
                "总内存": f"{self.data[0].get('ram_total_mb', 0)}MB"
            }
        
        # CPU统计
        cpu_usage = [d['cpu_avg_usage'] for d in self.data if 'cpu_avg_usage' in d]
        cpu_freq = [d['cpu_avg_freq'] for d in self.data if 'cpu_avg_freq' in d]
        
        if cpu_usage:
            report["CPU使用情况"] = {
                "平均使用率": f"{statistics.mean(cpu_usage):.1f}%",
                "最大使用率": f"{max(cpu_usage):.1f}%",
                "最小使用率": f"{min(cpu_usage):.1f}%",
                "平均频率": f"{statistics.mean(cpu_freq):.0f}MHz",
                "最大频率": f"{max(cpu_freq)}MHz"
            }
        
        # GPU统计
        gpu_usage = [d['gpu_usage'] for d in self.data if 'gpu_usage' in d]
        if gpu_usage:
            report["GPU使用情况"] = {
                "平均使用率": f"{statistics.mean(gpu_usage):.1f}%",
                "最大使用率": f"{max(gpu_usage):.1f}%",
                "最小使用率": f"{min(gpu_usage):.1f}%"
            }
        
        # 温度统计
        cpu_temps = [d['cpu_temp'] for d in self.data if 'cpu_temp' in d]
        gpu_temps = [d['gpu_temp'] for d in self.data if 'gpu_temp' in d]
        
        if cpu_temps:
            report["温度情况"] = {
                "CPU平均温度": f"{statistics.mean(cpu_temps):.1f}°C",
                "CPU最高温度": f"{max(cpu_temps):.1f}°C"
            }
        
        if gpu_temps:
            report["温度情况"]["GPU平均温度"] = f"{statistics.mean(gpu_temps):.1f}°C"
            report["温度情况"]["GPU最高温度"] = f"{max(gpu_temps):.1f}°C"
        
        return report
    
    def save_data(self, filename=None):
        """保存原始数据到文件"""
        if filename is None:
            filename = f"jetson_performance_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump({
                "metadata": {
                    "collection_time": datetime.now().isoformat(),
                    "interval": self.interval,
                    "data_points": len(self.data)
                },
                "data": self.data
            }, f, ensure_ascii=False, indent=2)
        
        print(f"数据已保存到: {filename}")
        return filename
    
    def print_report(self, report):
        """打印格式化的报告"""
        print("\n" + "="*50)
        print("Jetson性能监控报告")
        print("="*50)
        
        for category, stats in report.items():
            print(f"\n【{category}】")
            if isinstance(stats, dict):
                for key, value in stats.items():
                    print(f"  {key}: {value}")
            else:
                print(f"  {stats}")

def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Jetson性能监控工具')
    parser.add_argument('-i', '--interval', type=float, default=1.0, 
                       help='采样间隔(秒), 默认1.0')
    parser.add_argument('-t', '--time', type=int, default=None,
                       help='监控时长(秒), 不指定则手动停止')
    parser.add_argument('-o', '--output', type=str, default=None,
                       help='输出文件名')
    
    args = parser.parse_args()
    
    monitor = JetsonMonitor(interval=args.interval)
    
    try:
        # 开始监控
        data = monitor.start_monitoring(duration=args.time)
        
        if data:
            # 生成报告
            report = monitor.generate_report()
            monitor.print_report(report)
            
            # 保存数据
            filename = monitor.save_data(args.output)
            
    except KeyboardInterrupt:
        print("\n监控已停止")
    except Exception as e:
        print(f"发生错误: {e}")

if __name__ == "__main__":
    main()
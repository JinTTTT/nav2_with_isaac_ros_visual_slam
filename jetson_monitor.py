#!/usr/bin/env python3
"""
Jetson Performance Monitor for Nav2 System
Monitors CPU, GPU and memory performance of Jetson device running Nav2
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
        Initialize monitor
        :param interval: Sampling interval in seconds
        """
        self.interval = interval
        self.data = []
        self.running = False
        
    def parse_tegrastats_line(self, line: str) -> Dict:
        """Parse one line of tegrastats output"""
        try:
            data = {}
            
            # Parse timestamp
            timestamp_match = re.search(r'^(\d{2}-\d{2}-\d{4} \d{2}:\d{2}:\d{2})', line)
            if timestamp_match:
                data['timestamp'] = timestamp_match.group(1)
            
            # Parse RAM usage - RAM 2807/31011MB
            ram_match = re.search(r'RAM (\d+)/(\d+)MB', line)
            if ram_match:
                used_ram = int(ram_match.group(1))
                total_ram = int(ram_match.group(2))
                data['ram_used_mb'] = used_ram
                data['ram_total_mb'] = total_ram
                data['ram_usage_percent'] = (used_ram / total_ram) * 100
            
            # Parse CPU usage - CPU [5%@2265,7%@2265,...]
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
            
            # Parse GPU frequency - GR3D_FREQ 0%
            gpu_match = re.search(r'GR3D_FREQ (\d+)%', line)
            if gpu_match:
                data['gpu_usage'] = int(gpu_match.group(1))
            
            # Parse temperature - CPU@51C GPU@51C
            cpu_temp_match = re.search(r'CPU@([\d.]+)C', line)
            if cpu_temp_match:
                data['cpu_temp'] = float(cpu_temp_match.group(1))
                
            gpu_temp_match = re.search(r'GPU@([\d.]+)C', line)
            if gpu_temp_match:
                data['gpu_temp'] = float(gpu_temp_match.group(1))
            
            return data
        except Exception as e:
            print(f"Parse error: {e}")
            return {}
    
    def start_monitoring(self, duration=None):
        """Start monitoring"""
        print(f"Starting Jetson performance monitoring (interval: {self.interval}s)")
        if duration:
            print(f"Monitoring duration: {duration}s")
        
        self.running = True
        self.data = []
        
        # Set signal handlers
        def signal_handler(signum, frame):
            print("\nReceived stop signal, saving data...")
            self.running = False
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        # Start tegrastats
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
                            print(f"Data points collected: {len(self.data)}", end='\r')
                    
                    if proc.poll() is not None:
                        break
                
                proc.terminate()
                
        except Exception as e:
            print(f"Error during monitoring: {e}")
        
        print(f"\nMonitoring finished, collected {len(self.data)} data points")
        return self.data
    
    def generate_report(self) -> Dict:
        """Generate performance statistics report"""
        if not self.data:
            return {"error": "No data available"}
        
        report = {
            "Monitoring Overview": {
                "Data Points": len(self.data),
                "Duration": f"{len(self.data) * self.interval:.1f}s",
                "Sampling Interval": f"{self.interval}s"
            }
        }
        
        # RAM statistics
        ram_usage = [d['ram_usage_percent'] for d in self.data if 'ram_usage_percent' in d]
        ram_used = [d['ram_used_mb'] for d in self.data if 'ram_used_mb' in d]
        
        if ram_usage:
            report["Memory Usage"] = {
                "Average Usage": f"{statistics.mean(ram_usage):.1f}%",
                "Max Usage": f"{max(ram_usage):.1f}%",
                "Min Usage": f"{min(ram_usage):.1f}%",
                "Average Used": f"{statistics.mean(ram_used):.0f}MB",
                "Max Used": f"{max(ram_used)}MB",
                "Total Memory": f"{self.data[0].get('ram_total_mb', 0)}MB"
            }
        
        # CPU statistics
        cpu_usage = [d['cpu_avg_usage'] for d in self.data if 'cpu_avg_usage' in d]
        cpu_freq = [d['cpu_avg_freq'] for d in self.data if 'cpu_avg_freq' in d]
        
        if cpu_usage:
            report["CPU Usage"] = {
                "Average Usage": f"{statistics.mean(cpu_usage):.1f}%",
                "Max Usage": f"{max(cpu_usage):.1f}%",
                "Min Usage": f"{min(cpu_usage):.1f}%",
                "Average Frequency": f"{statistics.mean(cpu_freq):.0f}MHz",
                "Max Frequency": f"{max(cpu_freq)}MHz"
            }
        
        # GPU statistics
        gpu_usage = [d['gpu_usage'] for d in self.data if 'gpu_usage' in d]
        if gpu_usage:
            report["GPU Usage"] = {
                "Average Usage": f"{statistics.mean(gpu_usage):.1f}%",
                "Max Usage": f"{max(gpu_usage):.1f}%",
                "Min Usage": f"{min(gpu_usage):.1f}%"
            }
        
        # Temperature statistics
        cpu_temps = [d['cpu_temp'] for d in self.data if 'cpu_temp' in d]
        gpu_temps = [d['gpu_temp'] for d in self.data if 'gpu_temp' in d]
        
        if cpu_temps:
            report["Temperature"] = {
                "CPU Average Temp": f"{statistics.mean(cpu_temps):.1f}°C",
                "CPU Max Temp": f"{max(cpu_temps):.1f}°C"
            }
        
        if gpu_temps:
            report["Temperature"]["GPU Average Temp"] = f"{statistics.mean(gpu_temps):.1f}°C"
            report["Temperature"]["GPU Max Temp"] = f"{max(gpu_temps):.1f}°C"
        
        return report
    
    def save_data(self, filename=None):
        """Save raw data to file"""
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
        
        print(f"Data saved to: {filename}")
        return filename
    
    def print_report(self, report):
        """Print formatted report"""
        print("\n" + "="*50)
        print("Jetson Performance Monitoring Report")
        print("="*50)
        
        for category, stats in report.items():
            print(f"\n【{category}】")
            if isinstance(stats, dict):
                for key, value in stats.items():
                    print(f"  {key}: {value}")
            else:
                print(f"  {stats}")

def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Jetson Performance Monitoring Tool')
    parser.add_argument('-i', '--interval', type=float, default=1.0, 
                       help='Sampling interval in seconds, default 1.0')
    parser.add_argument('-t', '--time', type=int, default=None,
                       help='Monitoring duration in seconds, manual stop if not specified')
    parser.add_argument('-o', '--output', type=str, default=None,
                       help='Output filename')
    
    args = parser.parse_args()
    
    monitor = JetsonMonitor(interval=args.interval)
    
    try:
        # Start monitoring
        data = monitor.start_monitoring(duration=args.time)
        
        if data:
            # Generate report
            report = monitor.generate_report()
            monitor.print_report(report)
            
            # Save data
            filename = monitor.save_data(args.output)
            
    except KeyboardInterrupt:
        print("\nMonitoring stopped")
    except Exception as e:
        print(f"Error occurred: {e}")

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
设备连接检查脚本
用于诊断bearcar遥控器和VESC连接问题
"""

import os
import subprocess
import serial
from pathlib import Path

def check_device_exists(device_path):
    """检查设备文件是否存在"""
    return Path(device_path).exists()

def check_device_permissions(device_path):
    """检查设备权限"""
    try:
        with open(device_path, 'r'):
            return True
    except PermissionError:
        return False
    except:
        return False

def find_usb_devices():
    """查找所有USB串口设备"""
    usb_devices = []
    for device in ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1', '/dev/USB_rc']:
        if check_device_exists(device):
            usb_devices.append(device)
    return usb_devices

def test_serial_connection(device_path, baud_rate=57600):
    """测试串口连接"""
    try:
        ser = serial.Serial(device_path, baud_rate, timeout=1)
        ser.close()
        return True
    except Exception as e:
        print(f"串口连接测试失败 {device_path}: {e}")
        return False

def main():
    print("🔍 Bearcar设备连接检查")
    print("=" * 50)
    
    # 检查常见的串口设备
    candidate_devices = ['/dev/ttyACM1', '/dev/USB_rc', '/dev/ttyUSB0', '/dev/ttyACM0']
    
    print("📋 设备检查结果:")
    available_devices = []
    
    for device in candidate_devices:
        exists = check_device_exists(device)
        permission = check_device_permissions(device) if exists else False
        serial_ok = test_serial_connection(device) if exists and permission else False
        
        status = "✅" if exists and permission and serial_ok else "❌"
        print(f"{status} {device}")
        print(f"   存在: {'是' if exists else '否'}")
        print(f"   权限: {'是' if permission else '否'}")
        print(f"   串口: {'正常' if serial_ok else '异常'}")
        print()
        
        if exists and permission and serial_ok:
            available_devices.append(device)
    
    # 查找所有USB设备
    print("🔌 发现的USB设备:")
    usb_devices = find_usb_devices()
    for device in usb_devices:
        print(f"   {device}")
    
    # 建议
    print("\n💡 建议:")
    if available_devices:
        print(f"✅ 可用设备: {available_devices}")
        print(f"建议在参数文件中使用: {available_devices[0]}")
    else:
        print("❌ 没有找到可用的串口设备")
        print("请检查:")
        print("1. 设备是否正确连接")
        print("2. 用户是否在dialout组中: sudo usermod -a -G dialout $USER")
        print("3. 设备权限: sudo chmod 666 /dev/ttyACM*")
    
    # 检查ROS2环境
    print("\n🚀 ROS2环境检查:")
    try:
        result = subprocess.run(['ros2', 'pkg', 'list'], capture_output=True, text=True)
        if 'remote_control' in result.stdout:
            print("✅ remote_control包已安装")
        else:
            print("❌ remote_control包未找到")
            
        if 'vesc_ackermann' in result.stdout:
            print("✅ vesc_ackermann包已安装")
        else:
            print("❌ vesc_ackermann包未找到")
            
    except Exception as e:
        print(f"❌ ROS2环境检查失败: {e}")

if __name__ == '__main__':
    main() 
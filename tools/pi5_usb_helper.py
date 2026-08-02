"""
Pi 5 USB Device Helper
======================
Auto-detect which /dev/ttyUSB* or /dev/ttyACM* corresponds to
the CAN2USB adapter vs the IMU, based on vendor/product IDs.

Usage:
    from pi5_usb_helper import find_motor_port, find_imu_port
"""

import subprocess
import glob
import os


def list_usb_serial_ports():
    """List all USB serial ports with their vendor/product info."""
    ports = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
    results = []
    for port in ports:
        info = {'port': port, 'vendor': '', 'product': '', 'serial': ''}
        # Try to get device info from sysfs
        devname = os.path.basename(port)
        syspath = f'/sys/class/tty/{devname}/device'
        if os.path.islink(syspath):
            real = os.path.realpath(syspath)
            # Walk up to find USB device info
            parts = real.split('/')
            for i in range(len(parts) - 1, 0, -1):
                candidate = '/'.join(parts[:i + 1])
                vid_path = os.path.join(candidate, 'idVendor')
                if os.path.exists(vid_path):
                    try:
                        info['vendor'] = open(vid_path).read().strip()
                        info['product'] = open(os.path.join(candidate, 'idProduct')).read().strip()
                        serial_path = os.path.join(candidate, 'serial')
                        if os.path.exists(serial_path):
                            info['serial'] = open(serial_path).read().strip()
                    except:
                        pass
                    break
        results.append(info)
    return results


def find_port_by_vid_pid(vid, pid):
    """Find a serial port matching a USB vendor:product ID."""
    for info in list_usb_serial_ports():
        if info['vendor'] == vid and info['product'] == pid:
            return info['port']
    return None


def find_motor_port(fallback='/dev/ttyUSB0'):
    """
    Try to auto-detect the CAN2USB adapter port.
    
    Common CAN2USB adapters use:
      - CH340:  VID=1a86, PID=7523
      - CP2102: VID=10c4, PID=ea60
      - FTDI:   VID=0403, PID=6001
    
    Adjust the VID/PID below to match YOUR adapter.
    Run `pi5_usb_helper.py` to see what's connected.
    """
    # Try common CAN2USB chipsets (edit these for your hardware)
    for vid, pid in [('1a86', '7523'), ('10c4', 'ea60'), ('0403', '6001')]:
        port = find_port_by_vid_pid(vid, pid)
        if port:
            return port
    return fallback


def find_imu_port(fallback='/dev/ttyUSB1'):
    """
    Try to auto-detect the IMU port.
    
    Common IMU USB chipsets:
      - CP2102: VID=10c4, PID=ea60
      - CH340:  VID=1a86, PID=7523
    
    Adjust the VID/PID below to match YOUR IMU.
    """
    # If motor is already found, return the OTHER port
    motor = find_motor_port()
    ports = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
    for p in ports:
        if p != motor:
            return p
    return fallback


def print_all_ports():
    """Print all detected USB serial ports with details."""
    ports = list_usb_serial_ports()
    if not ports:
        print("No USB serial ports found!")
        print("  - Check cables are connected")
        print("  - Run: sudo dmesg | tail -20")
        print("  - You may need: sudo apt install python3-serial")
        return
    
    print(f"Found {len(ports)} USB serial port(s):\n")
    for info in ports:
        print(f"  {info['port']}")
        print(f"    VID:PID = {info['vendor']}:{info['product']}")
        if info['serial']:
            print(f"    Serial  = {info['serial']}")
        print()
    
    print("Tip: Use these VID:PID values in find_motor_port() / find_imu_port()")
    print("     to enable auto-detection for your specific hardware.\n")


if __name__ == '__main__':
    print("=" * 50)
    print("  Pi 5 USB Serial Port Scanner")
    print("=" * 50 + "\n")
    print_all_ports()

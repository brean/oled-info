#!/usr/bin/env python3
# MIT license
# Based on the extended system info by Richard Hull and contributors
# https://github.com/rm-hull/luma.examples/blob/main/examples/

# regex to parse wireless network
import re

# read ROS_DOMAIN_ID from os.environ
import os

# datetime to display uptime human-readable
from datetime import datetime

# ROS 2 for battery
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import BatteryState

import subprocess

try:
    # vcgen-cmd for raspberry pi undervoltage/throtteling
    from vcgencmd import Vcgencmd
    VCGENCMD = True
except ModuleNotFoundError:
    VCGENCMD = False

from luma_display import Display

# psutil to get system information (CPU, memory, temperature, ...)
import psutil
# socket for ip address
import socket
from collections import OrderedDict


# https://www.raspberrypi.com/documentation/computers/os.html#vcgencmd
UNDERVOLTAGE_DETECTED = '0'
ARM_FREQ_CAPPED = '1'
CURRENTLY_THROTTLED = '2'
SOFT_TEMPERATURE_LIMIT = '3'
UNDERVOLTAGE_OCCURED = '16'
ARM_FREQ_CAPP_OCCURED = '17'
THROTTLING_OCCURED = '18'
SOFT_TEMPERATURE_LIMIT_OCCURED = '19'


def get_temp_psutil():
    temperatures = psutil.sensors_temperatures()
    if 'coretemp' in temperatures:
        for temp in temperatures['coretemp']:
            return temp.current
    elif 'thermal_zone' in temperatures:
        for temp in temperatures['thermal_zone']:
            return temp.current
    else:
        return 0.0


def get_temp(vcgm):
    if not vcgm:
        return get_temp_psutil()
    try:
        return float(vcgm.measure_temp())
    except subprocess.CalledProcessError:
        # fallback to psutil if vcgm isn't working or not installed
        return get_temp_psutil()


def get_cpu():
    return psutil.cpu_percent()


def get_mem():
    return (psutil.virtual_memory().percent, psutil.virtual_memory().used)


def get_disk_usage():
    usage = psutil.disk_usage('/')
    return usage.used / usage.total * 100


def get_throttle_data(vcgm):
    if not vcgm:
        return None
    try:
        return vcgm.get_throttled()['breakdown']
    except subprocess.CalledProcessError:
        return None


def get_uptime():
    uptime = f'{datetime.now() - datetime.fromtimestamp(psutil.boot_time())}'
    return uptime.split('.')[0]


def ros_id():
    if 'ROS_DOMAIN_ID' in os.environ:
        return os.environ['ROS_DOMAIN_ID']
    else:
        return '0'


def _find_single_ipv4_address(addrs):
    for addr in addrs:
        if addr.family == socket.AddressFamily.AF_INET:  # IPv4
            return addr.address


def get_ipv4_address(interface_name=None):
    if_addrs = psutil.net_if_addrs()

    if isinstance(interface_name, str) and interface_name in if_addrs:
        addrs = if_addrs.get(interface_name)
        address = _find_single_ipv4_address(addrs)
        return address if isinstance(address, str) else ''
    else:
        if_stats = psutil.net_if_stats()
        # remove loopback
        if_stats_filtered = {
            key: if_stats[key]
            for key, stat in if_stats.items()
            if 'loopback' not in stat.flags
        }
        # sort interfaces by
        # 1. Up/Down
        # 2. Duplex mode (full: 2, half: 1, unknown: 0)
        if_names_sorted = [
            stat[0] for stat in sorted(
                if_stats_filtered.items(),
                key=lambda x: (x[1].isup, x[1].duplex),
                reverse=True)]
        if_addrs_sorted = OrderedDict(
            (key, if_addrs[key]) for key in if_names_sorted if key in if_addrs)

        for _, addrs in if_addrs_sorted.items():
            address = _find_single_ipv4_address(addrs)
            if isinstance(address, str):
                return address

        return ''


def get_net_addr(devices=None):
    if not devices:
        devices = ['wlan0', 'eth0']
    for dev in devices:
        ip = get_ipv4_address(dev)
        # TODO: switch every few seconds if all devices are connected?
        if ip != '':
            # we only return the first device we find
            return f'IP{ip:>15}'
    # No ip device (yet)
    return ''


def throttle_emojis(throttle_data):
    txt = ''
    # stuff that happened in general
    # bolt for underwoltage
    txt += '⚡' if throttle_data[UNDERVOLTAGE_OCCURED] else '-'
    # frowning face for throttling
    txt += '☹️' if throttle_data[THROTTLING_OCCURED] else '-'
    # C for cpu _F_requency capped
    txt += 'F' if throttle_data[ARM_FREQ_CAPP_OCCURED] else '-'
    # T for _T_emperature
    txt += 'T' if throttle_data[SOFT_TEMPERATURE_LIMIT_OCCURED] else '-'

    txt += '|'

    # stuff that is currently happending
    txt += '⚡' if throttle_data[UNDERVOLTAGE_DETECTED] else '-'
    txt += '☹️' if throttle_data[CURRENTLY_THROTTLED] else '-'
    txt += 'F' if throttle_data[ARM_FREQ_CAPPED] else '-'
    txt += 'T' if throttle_data[SOFT_TEMPERATURE_LIMIT] else '-'
    return txt


def get_wifi_strength():
    with open('/proc/net/wireless', 'r', encoding='utf-8') as fd:
        text = fd.read()
        text = text.split('\n')
        if len(text[2]) < 1:
            return -1
        first_line = re.sub(r'[\s\|]{2,}', ' ', text[1]).split(' ')
        sec_line = re.sub(r'[\s\|]{2,}', ' ', text[2]).split(' ')
        index = first_line.index('link')
        wifi_strength = sec_line[index]
        return float(wifi_strength)


def _get_wifi_mode(device='wlan0'):
    try:
        ap_txt = subprocess.check_output(['iwgetid', '-m', device])
        if not ap_txt:
            return None
        ap_txt = ap_txt.decode('utf-8')
        if ap_txt:
            txt = ap_txt.strip()
            if txt.endswith('Managed'):
                return 'managed'
            elif txt.endswith('Master'):
                # Access Point mode
                return 'master'
    except subprocess.CalledProcessError:
        return None


def _get_wifi_name(device='wlan0'):
    try:
        ap_txt = subprocess.check_output(['iw', 'dev', device, 'info'])
        if not ap_txt:
            return None
        m = re.search('ssid (?P<ssid>.*)', ap_txt.decode('utf-8'))
        if m:
            return m.group('ssid')
    except subprocess.CalledProcessError:
        return None


def get_wifi_name():
    ap_name = _get_wifi_name()
    if ap_name:
        return ap_name
    else:
        return '(not connected)'


class BatteryInfoNode(Node):
    def __init__(self, display):
        super().__init__('display_info_node')
        if VCGENCMD:
            self.vcgm = Vcgencmd()
        else:
            self.vcgm = None
        self.display = display
        self.create_timer(1.0, self.update_stats)
        self.battery_percentage = 0.0
        self.subscription = self.create_subscription(
            BatteryState,
            'battery_state',
            self.listener_callback,
            10)
        self.update_stats()

    def listener_callback(self, msg):
        self.battery_percentage = msg.percentage

    def update_stats(self):
        # TODO: configure this using a YAML-file and/or a ROS 2 service :-)
        # TODO: add a second ROS2-service that sends only the data.
        # TODO: switch between different network devices if multiple devices
        # have a wifi-connection

        wifi_strength = get_wifi_strength()
        wifi = (
            {
                'type': 'text',
                'value': 'WIFI: (not connected)'
            }
            if wifi_strength == -1 else
            {
                'type': 'int',
                'value': wifi_strength,
                'max': 70,
                'unit': '/70',
                'text': 'WIFI'
            }
        )
        data = [
            {
                'type': 'text',
                'value': get_net_addr()
            }, {
                'type': 'text',
                'value': get_wifi_name()
            }, {
                'type': 'percentage',
                'text': 'BAT',
                'unit': '%',
                'value': self.battery_percentage
            },
            wifi
        ]
        _throttle = get_throttle_data(self.vcgm)
        if _throttle:
            data.append({
                'type': 'text',
                'value': throttle_emojis(_throttle)
            })
        _ros_id = ros_id()
        if _ros_id:
            data.append({
                'type': 'text',
                'line': 4,
                'left': 77,
                'value': f'ROS:{_ros_id}'
            })

        # additional options of data to display:
        # Name of connected WiFi AccessPoint:
        # {
        #     'type': 'text',
        #     'value': f'AP: {get_wifi_name()}'
        # }
        # Temperature:
        # 'type': 'percentage',
        # 'text': 'TEMP',
        # 'unit': "'C",
        # 'value': get_temp(self.vcgm)
        # memory usage:
        # {
        #     'type': 'percentage',
        #     'value': get_mem(),
        #     'text': 'MEM',
        #     'unit': ' mb',
        # }
        # CPU usage
        # {
        #     'type': 'percentage',
        #     'value': get_cpu(),
        #     'text': 'CPU',
        #     'unit': ' %',
        # }
        # uptime:
        # {
        #     'type': 'text',
        #     'value': f'UP: {get_uptime()}'
        # }
        self.display.display_stats(data)


def main(args=None):
    rclpy.init(args=args)

    node = BatteryInfoNode(Display())

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

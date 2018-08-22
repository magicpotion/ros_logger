#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This script is responsible for various data collection,
used as a system health check and first point debugging.

"""
import rospy
import rosnode
import rosparam
# import rosservice
import sys_monitor.srv as srv
from hr_msgs.msg import audiodata
from dynamixel_msgs.msg import MotorStateList
import json
import glob
import logging
import os
import psutil
import re
import socket
import subprocess
import time
import warnings
try:
    from rospy_message_converter import message_converter
except ImportError:
    warnings.warn('Error importing rospy_message_converter', UserWarning)

logger = logging.getLogger('hr.sys_monitor')

ALERT_LOG_ENABLED = True
ALERT_LOG_INTERVAL = 30
MOTOR_STATES_LOG_ENABLED = os.environ.get('DEV_MODE', False) == False
MOTOR_STATES_LOG_INTERVAL = 30


class MonitoringController:
    def __init__(self):
        self.run_cycle = 0  # for performance, skipping checks
        self.robot_name = self.get_robot_name()
        self.robot_body = self.get_robot_body()
        self.nodes_yaml = self._read_nodes_yaml()
        self.rosparam_pololu = self.get_pololu_params()
        self.rosparam_motors = rosparam.get_param('/'+self.robot_name+'/motors')
        self.dynamixel_names_by_id = {m['motor_id']: m['name'] for m in self.rosparam_motors.values()
                                      if m['hardware'] == 'dynamixel'}
        self.motors_max_load = {}  # max/min calculated within MOTOR_STATES_LOG_INTERVAL
        self.motors_min_load = {}
        self.rostop_motor_states = []
        # In case multiple
        self.rostop_motor_topic_states = {}
        self.audio_lvl = []
        self.system_status = []

        self.cache = {
            'blender': {'cur_alert': {}, 'fps': []},
            'cpu_heat': {'cur_alert': {}},
            'camera': {'cur_alert': {}, 'disconnected': [], 'cams': []},
            'dynamixel': {'cur_alert': {}, 'failed': [], 'last_update': 0},
            'dxl_voltage': {'cur_alert': {}},
            'dxl_temperature': {'cur_alert': {}},
            'hd': {'cur_alert': {}},
            'internet': {'cur_alert': {}},
            'node': {'cur_alert': {}, 'disconnected': [], 'nodes': []},
            'pololu': {'cur_alert': {}}
        }

        rospy.init_node('hr_monitoring')
        rospy.Service('~get_info', srv.Json, self.get_monitoring_info)
        rospy.Service('~get_status', srv.Json, self.get_system_status)

        rospy.Subscriber('/{}/safe/motor_states/default'.format(self.robot_name), MotorStateList,
                         self._update_motor_states)
        # Add arms topic
        rospy.Subscriber('/{}/safe/motor_states/arms'.format(self.robot_name), MotorStateList,
                         lambda msg: self._update_motor_states(msg, 'arms'))

        rospy.Subscriber('/{}/audio_sensors'.format(self.robot_name), audiodata, self._update_audio_lvl)

    def _update_motor_states(self, msg, topic='default'):
        # TODO: store states using message_converter
        try:
            self.rostop_motor_topic_states[topic] = msg.motor_states
            # Combine states for all topics
            t = time.time()
            self.rostop_motor_states = [state for topic in self.rostop_motor_topic_states.values() for state in topic
                                            if state.timestamp > t-1]

            self.cache['dynamixel']['last_update'] = t
            if MOTOR_STATES_LOG_ENABLED:
                for state in msg.motor_states:
                    try:
                        if state.load > self.motors_max_load[state.id]:
                            self.motors_max_load[state.id] = round(float(state.load), 3)
                        if state.load < self.motors_min_load[state.id]:
                            self.motors_min_load[state.id] = round(float(state.load), 3)
                    except KeyError:
                        self.motors_max_load[state.id] = round(float(state.load), 3)
                        self.motors_min_load[state.id] = round(float(state.load), 3)
        except Exception as e:
            logger.error('_update_motor_states: {}'.format(e))

    def _update_audio_lvl(self, msg):
        try:
            self.audio_lvl = int(msg.Decibel)
        except Exception as e:
            self.audio_lvl = 0
            logger.error('_update_audio_lvl: {}'.format(e))

    def _read_nodes_yaml(self):
        nodes = []
        try:
            config_file = rosparam.get_param('/robots_config_dir')+'/heads/'+self.robot_name+'/nodes.yaml'
            nodes = rosparam.load_file(config_file)[0][0]['nodes']
        except rosparam.RosParamException as e:
            logger.error('_read_nodes_yaml: {}'.format(str(e)))
        return nodes

    def _run_cycle(self):
        if self.run_cycle < 60:
            self.run_cycle += 1
        else:
            self.run_cycle = 1

    def check_system_status(self):
        """ WebUI System Status """
        status = []
        if (self.run_cycle and self.run_cycle % 5) == 0:  # set cur_alert if using run_cycles
            dxls = self.get_dynamixels()
            self.cache['node']['cur_alert'] = self.alert_check_nodes()
            self.cache['dynamixel']['cur_alert'] = self.alerts_check_dynamixel(dxls)
            self.cache['dxl_voltage']['cur_alert'] = self.alerts_check_dxl_voltage(dxls)
            self.cache['dxl_temperature']['cur_alert'] = self.alerts_check_dxl_temperature(dxls)
            self.cache['pololu']['cur_alert'] = self.alert_check_pololu()
            self.cache['internet']['cur_alert'] = self.alerts_check_internet()
            self.cache['cpu_heat']['cur_alert'] = self.alerts_check_cpu()
            self.cache['camera']['cur_alert'] = self.alert_check_cams()
            self.get_blender_fps()
            if len(self.cache['blender']['fps']) == 5 and all(int(i) < 30 for i in self.cache['blender']['fps']):
                self.cache['node']['cur_alert'] = self.get_alert('blender')

        if (self.run_cycle and self.run_cycle % 20) == 0:  # set cur_alert if using run_cycles
            self.cache['hd']['cur_alert'] = self.alerts_check_hdd()

        status.append(self.cache['dynamixel']['cur_alert']) if self.cache['dynamixel']['cur_alert'] else None
        status.append(self.cache['dxl_voltage']['cur_alert']) if self.cache['dxl_voltage']['cur_alert'] else None
        status.append(self.cache['dxl_temperature']['cur_alert']) if self.cache['dxl_temperature']['cur_alert'] else None
        status.append(self.cache['hd']['cur_alert']) if self.cache['hd']['cur_alert'] else None
        status.append(self.cache['node']['cur_alert']) if self.cache['node']['cur_alert'] else None
        status.append(self.cache['blender']['cur_alert']) if self.cache['blender']['cur_alert'] else None
        status.append(self.cache['internet']['cur_alert']) if self.cache['internet']['cur_alert'] else None
        status.append(self.cache['cpu_heat']['cur_alert']) if self.cache['cpu_heat']['cur_alert'] else None
        status.append(self.cache['camera']['cur_alert']) if self.cache['camera']['cur_alert'] else None
        status.append(self.cache['pololu']['cur_alert']) if self.cache['pololu']['cur_alert'] else None
        # status.append(self.get_alert('test', self.cache['blender']['fps']))

        if (self.run_cycle and ALERT_LOG_ENABLED):
            if self.run_cycle % ALERT_LOG_INTERVAL == 0:
                logger.info('alert_log', extra={'data': status})

        if (self.run_cycle and MOTOR_STATES_LOG_ENABLED):
            if self.run_cycle % MOTOR_STATES_LOG_INTERVAL == 0:
                states = MotorStateList(motor_states=self.rostop_motor_states)
                states = message_converter.convert_ros_message_to_dictionary(states)
                for state in states['motor_states']:
                    state['name'] = self.dynamixel_names_by_id.get(state['id'], 'n/a')
                    state['max_load'] = self.motors_max_load[state['id']]
                    state['min_load'] = self.motors_min_load[state['id']]
                    logger.info('motorstates_log', extra={'data': state})
                self.motors_max_load = {}  # reset calculations
                self.motors_min_load = {}

        self._run_cycle()
        self.system_status = status

    def get_system_status(self, req):
        return srv.JsonResponse(success=True, response=json.dumps(self.system_status))

    def get_monitoring_info(self, req):
        """ WebUI Monitoring Page with system details """
        mem = psutil.virtual_memory()
        data = {
            'system': {
                'audio_lvl': self.audio_lvl,
                'blender_fps': self.get_blender_fps(),
                'cpu_count': psutil.cpu_count(),
                'cpu_percent': psutil.cpu_percent(),
                'cpu_temperature': self.get_cpu_temperature(),
                'hd_used': self.get_disk_space_used(),
                'internet': self.get_internet_status(),
                'mem': bytes2human(mem.total),
                'mem_used': mem.percent,
            },
            'robot': {
                'robot_name': self.robot_name,
                'robot_body': self.robot_body,
            },
            'hardware': {
                'cams': self.get_cams(),
                'pololu': self.get_pololu(),
                'dynamixel': self.get_dynamixel_status(),
                # 'cams': [{'name':'WideAngle', 'status': 1}, {'name':'InteliSens', 'status': 0}],
                # 'pololu': [{'name':'pololu_body', 'status': 0}, {'name': 'pololu_head', 'status': 1}],
            },
            'nodes': self.get_node_status(),
        }
        return srv.JsonResponse(success=True, response=json.dumps(data))

    def get_robot_name(self):
        return rosparam.get_param('/robot_name').strip('/')

    def get_robot_body(self):
        assemblies = rosparam.get_param('/assemblies')
        for ass in assemblies:
            if 'bodies' in ass:
                return ass.split('/')[-1]

    def get_cpu_temperature(self):
        coretemp = {}
        rawdata = psutil.sensors_temperatures()
        if rawdata and 'coretemp' in rawdata:
            for item in rawdata['coretemp']:
                if 'Core' in item.label:
                    coretemp[item.label] = [item.current, item.critical]
        return coretemp

    def alerts_check_cpu(self):
        coretemp = self.get_cpu_temperature()
        for core in coretemp:
            if coretemp[core][0] > 90:
                return self.get_alert('cpu_hot')

    def get_disk_space_used(self):
        disk_usage = psutil.disk_usage('/').percent
        return disk_usage

    def alerts_check_hdd(self):
        disk_usage = self.get_disk_space_used()
        if disk_usage > 95:
            return self.get_alert('hddcrit', disk_usage)
        elif disk_usage > 85:
            return self.get_alert('hddwarn', disk_usage)

    def get_internet_status(self, host='8.8.8.8', port=53, timeout=1):
        # Uncomment when in China:
        # host, port = ('www.baidu.com', '80')
        try:
            socket.create_connection((host, port), timeout)
        except Exception:
            return False
        return True

    def alerts_check_internet(self):
        active = self.get_internet_status()
        if not active:
            return self.get_alert('internet')

    def get_blender_fps(self):
        fps = 0
        try:
            fps = subprocess.check_output(['rosservice', 'call', '/blender_api/get_param', """'bpy.data.scenes["Scene"].evaFPS'"""])
            fps = re.findall(r'\d+', fps)[0]
            self.cache['blender']['fps'].append(fps)
            if len(self.cache['blender']['fps']) > 5:
                self.cache['blender']['fps'] = self.cache['blender']['fps'][1:]
        except Exception:
            pass
        return fps

    def get_cams(self):
        cams = []
        perception_params = rosparam.list_params('/{}/perception'.format(self.robot_name))
        realsense_enabled = rospy.get_param('/{}/realsense_enabled'.format(self.robot_name), False)
        try:
            if realsense_enabled:
                # overwritten, because device configs for realsense are usualy wrong
                dev_path = '/dev/v4l/by-id/usb-Intel_R__RealSense*'
                try:
                    status = os.path.exists(glob.glob('/dev/v4l/by-id/usb-Intel_R__RealSense*')[0])
                except IndexError:
                    status = False
                cams.append({
                    'name': 'RealSense',
                    'status': status
                })

            for param in perception_params:
                if 'wideangle/camera/video_device' in param:
                    dev_path = rosparam.get_param(param)
                    cams.append({
                        'name': param.split('/')[3].capitalize(),
                        'status': os.path.exists(dev_path)
                    })
        except Exception as e:
            logger.error('get_cams: {}'.format(str(e)))
        return cams

    def alert_check_cams(self):
        new_failed = []
        try:
            cams = self.get_cams()
            cams_failed = [n['name'] for n in cams if not n.get('status')]

            for n in self.cache['camera']['cams']:
                if n in cams_failed:
                    new_failed.append(n)

            self.cache['camera']['cams'] = [n['name'] for n in cams if n.get('status')]

            # remove disconnected cams when status ok
            for n in self.cache['camera']['disconnected']:
                if n in self.cache['camera']['cams']:
                    self.cache['camera']['disconnected'].remove(n)

            # merge disconnected cams with cached ones
            if new_failed:
                self.cache['camera']['disconnected'] = list(set(self.cache['camera']['disconnected'] + new_failed))

            if self.cache['camera']['disconnected']:
                return self.get_alert('camera', ', '.join(str(n) for n in self.cache['camera']['disconnected']))
            elif 'Realsense' in self.cache['camera']['cams']:
                return self.alert_check_usb_realsense()

        except (rosnode.ROSNodeException, IndexError) as e:
            logger.error('alert_check_cams: {}'.format(str(e)))

    def alert_check_usb_realsense(self):
        # check for common usb cable bandwith error
        try:
            usb = subprocess.check_output(["lsusb", "-t"])
            if 'uvcvideo, 5000M' not in usb:
                return self.get_alert('realsense_usb')
        except Exception:
            pass

    def get_node_status(self):
        try:
            nodes = rosnode.get_node_names()
            nodes = [node.split('/')[-1] for node in nodes]
            for line in self.nodes_yaml:
                node_name = line['node'].split('/')[-1]
                if node_name in nodes:
                    line.update({'status': 1})
                else:
                    line.update({'status': 0})
        except rosnode.ROSNodeException as e:
            logger.error('get_node_status: {}'.format(str(e)))
        return self.nodes_yaml

    def alert_check_nodes(self):
        new_failed = []
        try:
            nds = self.get_node_status()
            nds_failed = [n['label'] for n in nds if not n.get('status')]

            for n in self.cache['node']['nodes']:
                if n in nds_failed:
                    new_failed.append(n)

            self.cache['node']['nodes'] = [n['label'] for n in nds if n.get('status')]

            # remove disconnected nodes when status ok
            for n in self.cache['node']['disconnected']:
                if n in self.cache['node']['nodes']:
                    self.cache['node']['disconnected'].remove(n)

            # merge disconnected nodes with cached ones
            if new_failed:
                self.cache['node']['disconnected'] = list(set(self.cache['node']['disconnected'] + new_failed))

            if self.cache['node']['disconnected']:
                return self.get_alert('node', ', '.join(str(n) for n in self.cache['node']['disconnected']))

        except (rosnode.ROSNodeException, IndexError) as e:
            logger.error('get_node_status: {}'.format(str(e)))

    def get_pololu_params(self):
        params = rosparam.list_params(self.robot_name)
        params = filter(lambda x: 'pololu' in x and 'port_name' in x, params)
        return params

    def get_pololu(self):
        res = []
        for param in self.rosparam_pololu:
            dev_path = rosparam.get_param(param)
            res.append({
                'name': param.lstrip('/').split('/')[1],
                'status': os.path.exists(dev_path)
            })
        return res

    def alert_check_pololu(self):
        try:
            pololu = self.get_pololu()
            pololu_failed = [n['name'] for n in pololu if not n.get('status')]
            if pololu_failed:
                return self.get_alert('pololu', ', '.join(str(n) for n in pololu_failed))

        except (rosnode.ROSNodeException, IndexError) as e:
            logger.error('alert_check_pololu: {}'.format(str(e)))

    def get_dynamixel_manager_status(self):
        try:
            dev_name = rosparam.get_param('/{}/safe/dynamixel_manager/serial_ports/default/port_name'.format(self.robot_name))
            return os.path.exists(dev_name)
        except Exception as e:
            # logger.error('get_dynamixel_manager_status: {}'.format(e))
            return False

    def get_dynamixel_status(self):
        res = []
        for m in self.rosparam_motors.values():
            if m['hardware'] == 'dynamixel':
                d = {'id': m['motor_id'], 'name': m['name'], 'status': 0}
                for state in self.rostop_motor_states:
                    if state.id == m['motor_id']:
                        d.update({'status': 1,
                                  'voltage': state.voltage,
                                  'temperature': state.temperature})
                        break
                res.append(d)
        return res

    def get_dynamixels(self):
        dynamixels = []
        if self.cache['dynamixel']['last_update'] > time.time() - 0.5:
            dynamixels = self.get_dynamixel_status()
        return dynamixels

    def alerts_check_dynamixel(self, dxls=[]):
        """
        alert level: usb2dynamixel > alldynamixels > single dynamixels
        - for Single dxls failing, status is cached and alerts triggered when failed for more than a second
        """
        if not self.get_dynamixel_manager_status():
            return self.get_alert('usb2dynamixel')

        dxl_failed = [dxl for dxl in dxls if dxl['status'] == 0]
        dxl_failed_permanent = [dxl for dxl in dxl_failed if dxl['name'] in self.cache['dynamixel']['failed']]

        # update cache
        self.cache['dynamixel']['failed'] = [x['name'] for x in dxl_failed]

        if len(dxl_failed) == len(dxls):
            return self.get_alert('alldynamixels')

        if len(dxl_failed_permanent):
            return self.get_alert('dynamixel', len(dxl_failed_permanent), ', '.join(str(x['name']) for x in dxl_failed_permanent))

    def alerts_check_dxl_voltage(self, dxls=[]):
        dxl_active = [dxl for dxl in dxls if 'voltage' in dxl]
        dxl_voltage_low = [dxl for dxl in dxl_active if float(dxl['voltage']) < 11.5]

        if len(dxl_voltage_low):
            return self.get_alert('dxlvoltage', ', '.join(
                '{}({})'.format(str(x['name']), str(x['voltage'])) for x in dxl_voltage_low))

    def alerts_check_dxl_temperature(self, dxls=[]):
        dxl_active = [dxl for dxl in dxls if 'temperature' in dxl]
        dxl_temp_high = [dxl for dxl in dxl_active if int(dxl['temperature']) > 75]

        if len(dxl_temp_high):
            return self.get_alert('dxltemperature', ', '.join(
                '{}({})'.format(str(x['name']), str(x['temperature'])) for x in dxl_temp_high))

    def get_alert(self, error, arg1='', arg2=''):
        """ Predefined alert messages """
        alerts = {
            'test': {
                'status': 'critical',
                'info': 'This is a test - {}'.format(arg1)},
            'usb2dynamixel': {
                'status': 'critical',
                'info': 'USB2Dynamixel is disconnected'},
            'alldynamixels': {
                'status': 'critical',
                'info': 'Dynamixels failed to initialize'},
            'dynamixel': {
                'status': 'critical',
                'info': '{} Dynamixels not initialized: {}'.format(arg1, arg2)},
            'dxlvoltage': {
                'status': 'critical',
                'info': 'Dynamixels voltage too low: {}'.format(arg1)},
            'dxltemperature': {
                'status': 'critical',
                'info': 'Dynamixels temperature very hot: {}'.format(arg1)},
            'pololu': {
                'status': 'critical',
                'info': 'Pololu boards disconnected: {}'.format(arg1)},
            'internet': {
                'status': 'critical',
                'info': 'There is no internet connection detected'},
            'cpu_hot': {
                'status': 'critical',
                'info': 'CPU overheating, temperature above 90°C'},
            'hddcrit': {
                'status': 'critical',
                'info': 'Disk space is almost full: {}%'.format(arg1)},
            'camera': {
                'status': 'critical',
                'info': 'Camera disconnected: {}'.format(arg1)},
            'node': {
                'status': 'warning',
                'info': 'Ros Nodes failed after start: {}'.format(arg1)},
            'hddwarn': {
                'status': 'warning',
                'info': 'Running out of disk space: {}%'.format(arg1)},
            'realsense_usb': {
                'status': 'warning',
                'info': 'Realsense USB3 bandwidth issue detected'},
            'blender': {
                'status': 'warning',
                'info': 'Blender frame rate very low'},
        }
        return alerts[error]


def bytes2human(n):
    # >>> bytes2human(100001221)
    # '95.4M'
    symbols = ('K', 'M', 'G', 'T', 'P', 'E', 'Z', 'Y')
    prefix = {}
    for i, s in enumerate(symbols):
        prefix[s] = 1 << (i + 1) * 10
    for s in reversed(symbols):
        if n >= prefix[s]:
            value = float(n) / prefix[s]
            return '%.1f%s' % (value, s)
    return "%sB" % n


if __name__ == '__main__':
    controller = MonitoringController()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        controller.check_system_status()
        rate.sleep()

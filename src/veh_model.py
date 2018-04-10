#-*-coding:UTF-8-*-
'''
Created on 2018Äê4ÔÂ9ÈÕ
@author: mipapapa
'''
import json
import numpy as np
from numpy import tan
from numpy import pi as PI

class LOC_VEH_DATA:
    def __init__(self):
        self.steering_angle = 0
        self.shift_pos = 0
        
        self.vehicle_speed = 0
        self.wheel_speed_fl = 0
        self.wheel_speed_fr = 0
        self.wheel_speed_rl = 0
        self.wheel_speed_rr = 0
        
        self.yaw_rate = 0

        self.pulse_fl = 0
        self.pulse_fr = 0
        self.pulse_rl = 0
        self.pulse_rr = 0
        
        self.timestamp = 0

def load_veh_logs(json_path):
    veh_lst = []
    with open(json_path, "r") as fr:
        for s in fr:
            json_data = json.loads(s)
            veh = LOC_VEH_DATA()
            veh.shift_pos = json_data["Sft"]
            veh.steering_angle = json_data["Steer"]
            veh.yaw_rate = json_data["Yawrate"]

            veh.pulse_fl = json_data["Pulse"][0]
            veh.pulse_fr = json_data["Pulse"][1]
            veh.pulse_rl = json_data["Pulse"][2]
            veh.pulse_rr = json_data["Pulse"][3]
            
            veh.vehicle_speed = json_data["Speed"]
            veh.wheel_speed_fl = json_data["Wheel"][0]
            veh.wheel_speed_fr = json_data["Wheel"][1]
            veh.wheel_speed_rl = json_data["Wheel"][2]
            veh.wheel_speed_rr = json_data["Wheel"][3]
            
            veh.timestamp = json_data["time"]
            
            veh_lst.append(veh)
            
    return veh_lst

class VehMdl:
    def __init__(self):
        self.veh = np.zeros((16))
        self.pulse_curent = np.zeros((4))
        self.pulse_pre = np.zeros((4))
        self.pulse_delta = np.zeros((4))
        
    def init(self, veh_axis_length, veh_width, veh_params, shf_forward, pulse_distance, max_pulse):
        self.veh[:] = np.array(veh_params)
        self.shf_forward = shf_forward
        self.veh_axis_length = veh_axis_length
        self.veh_width = veh_width
        self.pulse_distance = pulse_distance
        self.max_speed = 30
        self.max_pulse = max_pulse
        self.init_pulse = False

    def steeringwheel_radius(self,whl, shft_pos):
        if self.shf_forward == shft_pos:
            if whl < 0: # RF
                radius = self.veh_axis_length *1.0 / tan((self.veh[0] * whl*whl*whl + self.veh[1] * whl*whl + self.veh[2] * whl + self.veh[3])*PI / 180) - self.veh_width / 2.0
            else:       # LF
                radius = self.veh_axis_length *1.0 / tan((self.veh[4] * whl*whl*whl + self.veh[5] * whl*whl + self.veh[6] * whl + self.veh[7])*PI / 180) + self.veh_width / 2.0
        else:
            if whl < 0:    # LR
                radius = self.veh_axis_length *1.0 / tan((self.veh[8] * whl*whl*whl + self.veh[9] * whl*whl + self.veh[10] * whl + self.veh[11])*PI / 180) - self.veh_width / 2.0
            else:        # RR
                radius = self.veh_axis_length *1.0 / tan((self.veh[12] * whl*whl*whl + self.veh[13] * whl*whl + self.veh[14] * whl + self.veh[15])*PI / 180) + self.veh_width / 2.0
        return radius
    
    def get_distance_pulse(self, v_data):
        self.pulse_curent[0] = v_data.pulse_fl
        self.pulse_curent[1] = v_data.pulse_fr
        self.pulse_curent[2] = v_data.pulse_rl
        self.pulse_curent[3] = v_data.pulse_rr
        if self.init_pulse == False:
            self.pulse_pre[0] = self.pulse_curent[0]
            self.pulse_pre[1] = self.pulse_curent[1]
            self.pulse_pre[2] = self.pulse_curent[2]
            self.pulse_pre[3] = self.pulse_curent[3]
            self.init_pulse = True;
        
        for i in range(4):
            if self.pulse_curent[i] < self.pulse_pre[i]:
                self.pulse_delta[i] = -self.pulse_pre[i] + self.max_pulse + self.pulse_curent[i]
            else:
                self.pulse_delta[i] = -self.pulse_pre[i] + self.pulse_curent[i]
        
        distance = (self.pulse_delta[2] + self.pulse_delta[3])*self.pulse_distance
        
        self.pulse_pre[0] = self.pulse_curent[0]
        self.pulse_pre[1] = self.pulse_curent[1]
        self.pulse_pre[2] = self.pulse_curent[2]
        self.pulse_pre[3] = self.pulse_curent[3]
        return distance
    
    def get_distance_wheel(self, v_data, time_offset):
        speed = (v_data.wheel_speed_rr + v_data.wheel_speed_rl) / 2.0 / 3.6
        if np.abs(speed) > self.max_speed:
            speed = self.last_speed
        self.last_speed = speed
        distance = speed * time_offset
        
        return distance
    
    def get_drive_distance(self,v_data, time_offset,use_pulse=False):
        if use_pulse:
            distance = self.get_distance_wheel(v_data, time_offset)
        else:
            distance = self.get_distance_pulse(v_data)
        return distance
    
def main():
    json_path = r"D:\test_data\kalman_data\0002\0002_CAN.log"
    veh_lst = load_veh_logs(json_path)
    print len(veh_lst)
    
if __name__=="__main__":
    pass
    main()


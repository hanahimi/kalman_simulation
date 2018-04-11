#-*-coding:UTF-8-*-
'''
Created on 2018年4月10日
@author: mipapapa
'''
from comm_kalman import Kalman
from veh_model import VehMdl, LOC_VEH_DATA
import json
import numpy as np
from sub_observer import LOC_SubNav
from common import LOC_POSTION

class LOC_InteNav:
    def __init__(self):
        self.i64TimeStampLast = 0
        self.f64Timeoffset = 0.0
        self.f64TimeoffsetLast = 0.0
        self.OriginInited = False
        
        self.kalman_core = Kalman(4, 4)
        self.veh_model = VehMdl()
        self.sub_modules = []

        self.tLocResult = LOC_POSTION()

        self.accumlate_distance = 0
        
    def init(self):
        # 初始化kalman参数的默认值
        for i in range(4):
            self.kalman_core.transition_mat[i,i] = 1
            self.kalman_core.measure_mat[i,i] = 1
            # 噪声初始化
            self.kalman_core.process_noise_cov[i,i] = 100.
            self.kalman_core.measure_noise_cov[i,i] = 100.
        
        # 初始化运动模型参数
        with open("smc.json", "r") as fp:
            smc_data = json.load(fp)
        self.veh_model.init(smc_data["veh_axis_length"],
                            smc_data["veh_width"],
                            smc_data["veh_params"],
                            smc_data["shf_forward"],
                            smc_data["pulse_distance"],
                            smc_data["max_pulse"])

        vps = LOC_SubNav()
        self.sub_modules.append(vps)
    
    def release(self):
        self.kalman_core.reset()
        self.OriginInited = False
    
    def predict_traject(self, tVehData, i64TimeStamp):
        if self.i64TimeStampLast != 0:
            self.f64Timeoffset = float(i64TimeStamp - self.i64TimeStampLast)/ 1000000.0
            self.i64TimeStampLast = i64TimeStamp
            
            veh_traject = self.veh_model.get_drive_distance(tVehData, 
                                                            self.f64Timeoffset, True)
            veh_turn_radius = self.veh_model.steeringwheel_radius(tVehData.steering_angle, 
                                                                  tVehData.shift_pos)
            dH = veh_traject / veh_turn_radius
            
            a = [0,0,0]
            a[0] = 1 if tVehData.steering_angle < 0 else -1
            a[1] = 1 if tVehData.shift_pos == 2 else -1
            a[2] = a[0] * a[1]

            dX = a[0] * veh_turn_radius * (1 - np.cos(np.abs(dH)));
            dY = a[1] * veh_turn_radius * np.sin(np.abs(dH));
            dH = a[2] * dH
            # 更新状态转移矩阵
            A = self.kalman_core.transition_mat
            A[0,2] = dX;    A[0,3] = dY
            A[1,2] = dY;    A[1,3] = -dX
            A[2,2] = np.cos(dH);    A[2,3] = -np.sin(dH)
            A[3,2] = np.sin(dH);    A[3,3] = np.cos(dH)
            self.kalman_core.state_pre = self.kalman_core.transition_mat * self.kalman_core.state_post
            self.kalman_core.state_post = self.kalman_core.state_pre
            self.f64TimeoffsetLast = self.f64Timeoffset
        
        else:
            self.f64Timeoffset = float(self.i64TimeStamp - self.i64TimeStampLast) / 1000000.0
            self.i64TimeStampLast = self.i64TimeStamp
            self.f64TimeoffsetLast = self.f64Timeoffset
            
            
            
    def odometryPrediction(self, tVehData, i64TimeStamp):
        if self.i64TimeStampLast != 0:
            self.f64Timeoffset = float(i64TimeStamp - self.i64TimeStampLast)/ 1000000.0
            self.i64TimeStampLast = i64TimeStamp
            
            veh_traject = self.veh_model.get_drive_distance(tVehData, 
                                                            self.f64Timeoffset, True)
            veh_turn_radius = self.veh_model.steeringwheel_radius(tVehData.steering_angle, 
                                                                  tVehData.shift_pos)
            dH = veh_traject / veh_turn_radius
            
            a = [0,0,0]
            a[0] = 1 if tVehData.steering_angle < 0 else -1
            a[1] = 1 if tVehData.shift_pos == 2 else -1
            a[2] = a[0] * a[1]

            dX = a[0] * veh_turn_radius * (1 - np.cos(np.abs(dH)));
            dY = a[1] * veh_turn_radius * np.sin(np.abs(dH));
            dH = a[2] * dH
            # 更新状态转移矩阵
            A = self.kalman_core.transition_mat
            A[0,2] = dX;    A[0,3] = dY
            A[1,2] = dY;    A[1,3] = -dX
            A[2,2] = np.cos(dH);    A[2,3] = -np.sin(dH)
            A[3,2] = np.sin(dH);    A[3,3] = np.cos(dH)

            # TODO:更新过程噪声矩阵
            
            self.kalman_core.predict()
            
            self.f64TimeoffsetLast = self.f64Timeoffset
        else:
            self.f64Timeoffset = float(self.i64TimeStamp - self.i64TimeStampLast) / 1000000.0
            self.i64TimeStampLast = self.i64TimeStamp
            self.f64TimeoffsetLast = self.f64Timeoffset
    
    def multiSensorCorrect(self):
        for i in range(len(self.sub_modules)):
            sub_nav = self.sub_modules[i]
            sub_nav.updateMeasurement()
            # 复制参数到kalman内存当中
            self.kalman_core.measure_mat[:] = sub_nav.MeasureMat[:]
            self.kalman_core.measurement[:] = sub_nav.Measurement[:]
            self.kalman_core.measure_noise_cov[:] = sub_nav.MeasureNoiseCov[:]
            # 默认方式进行校正
            self.kalman_core.correct()

    def get_result(self):
        self.tLocResult.f32Xm = self.kalman_core.state_post[0,0]
        self.tLocResult.f32Ym = self.kalman_core.state_post[1,0]
        a = self.kalman_core.state_post[2,0]
        b = self.kalman_core.state_post[3,0]
        n = np.sqrt(a*a + b*b)
        self.tLocResult.f32YawRad = np.arccos(a/n)
        if b < 0:
            self.tLocResult.f32YawRad *= -1
        return self.tLocResult
        

    def setStartPoint(self, i64TimeStamp):
        self.f64Timeoffset = (i64TimeStamp - self.i64TimeStampLast) / 1000000.0
        self.i64TimeStampLast = i64TimeStamp
        self.f64TimeoffsetLast = self.f64Timeoffset
        ret = self.sub_modules[0].updateMeasurement()
        if ret == 1:
            self.kalman_core.state_post = self.sub_modules[0].MeasureMat * self.sub_modules[0].Measurement
            self.OriginInited = True
        else:
            self.OriginInited = False
            
def main():
    a = LOC_InteNav()
    a.init()
    
if __name__=="__main__":
    pass
    main()


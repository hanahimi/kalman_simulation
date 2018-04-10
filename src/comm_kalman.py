#-*-coding:UTF-8-*-
'''
Created on 2018年4月9日
@author: mipapapa
'''
import numpy as np
import numpy.linalg as LA

class Kalman:
    def __init__(self,nState, nMeasure):
        self.I = np.eye(nState, nState)
        self.nState = nState
        self.nMeasure = nMeasure

        self.transition_mat = np.zeros((nState, nState),np.float32)        # 状态转移矩阵 A
        self.measure_mat = np.zeros((nMeasure, nState),np.float32)           # 观测矩阵 H
        self.state_pre = np.zeros((nState, 1),np.float32)             # 状态的先验估计X'(k)
        self.state_post = np.zeros((nState, 1),np.float32)            # 校正状态 X(k)，其本质就是前一时刻的状态
        self.measurement = np.zeros((nMeasure, 1),np.float32)           # 观测变量 Z(k)
        self.measure_noise_cov = np.zeros((nMeasure, nMeasure),np.float32)     # 测量噪声协方差矩阵 R
        self.process_noise_cov = np.zeros((nState, nState),np.float32)     # 过程噪声协方差矩阵 Q
        self.err_cov_post = np.zeros((nState, nState),np.float32)          # 误差协方差 P
        self.err_cov_pre = np.zeros((nState, nState),np.float32)           # 前一时刻的误差协方差 P(k-1)
        self.kalman_gain = np.zeros((nState, nMeasure),np.float32)           # Kalman增益
        
        # 转换为矩阵计算
        self.transition_mat = np.matrix(self.transition_mat)
        self.measure_mat = np.matrix(self.measure_mat)
        self.state_pre = np.matrix(self.state_pre)
        self.state_post = np.matrix(self.state_post)
        self.measurement = np.matrix(self.measurement)
        self.measure_noise_cov = np.matrix(self.measure_noise_cov)
        self.process_noise_cov = np.matrix(self.process_noise_cov)
        self.err_cov_post = np.matrix(self.err_cov_post)
        self.err_cov_pre = np.matrix(self.err_cov_pre)
        self.kalman_gain = np.matrix(self.kalman_gain)
        
        
    def reset(self):
        self.transition_mat[:] = 0
        self.measure_mat[:] = 0
        self.state_pre[:] = 0
        self.state_post[:] = 0
        self.measurement[:] = 0
        self.measure_noise_cov[:] = 0
        self.process_noise_cov[:] = 0
        self.err_cov_post[:] = 0
        self.err_cov_pre[:] = 0
        self.kalman_gain[:] = 0

        
    def predict(self):
        # 状态更新 x'(k) = A * x(k-1)
        # 更新误差协方差 P'(k) = A * P(k-1) * At + Q
        self.state_pre = self.transition_mat * self.state_post
        self.err_cov_pre = self.transition_mat * self.err_cov_post * self.transition_mat.T \
                    + self.process_noise_cov
                    
        
    def correct(self):
        # 计算Kalman增益 K(k) = P'(k) * Ht * inv(H * P'(k) * Ht + R)
        # 由观测变量Z(k)更新估计  X(k) = X'(k) - K(k) * ( Z(k) - H * X'(k) ) 
        # 更新误差协方差 P(k) = P'(k) - K(k) * H * P'(k) 
        
        self.kalman_gain = self.err_cov_pre * self.measure_mat.T * \
            LA.inv(self.measure_mat * self.err_cov_pre * self.measure_mat.T + self.measure_noise_cov);
        
        self.state_post = self.state_pre + self.kalman_gain * (self.measurement - self.measure_mat * self.state_pre);
        
        self.err_cov_post = (self.I - self.kalman_gain * self.measure_mat) * self.err_cov_pre;



def main():
    kal = Kalman(4,4)
    kal.reset()
    
if __name__=="__main__":
    pass
    main()


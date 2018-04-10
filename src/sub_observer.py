#-*-coding:UTF-8-*-
'''
Created on 2018��4��10��
@author: mipapapa
'''
import numpy as np
from common import LOC_POSTION

class LOC_SubNav:
    def __init__(self):
        self.MeasureBuffer = []
        self.nState = 4
        self.nMeasure = 4
        self.Measurement = np.matrix(np.zeros((self.nMeasure,1),np.float32))
        self.MeasureMat = np.matrix(np.eye(self.nMeasure,self.nState),np.float32)
        self.MeasureNoiseCov = np.matrix(np.zeros((self.nMeasure,self.nMeasure),np.float32))
        self.reset()
        
        self.tRawMeasureData = LOC_POSTION()
    
    def reset(self):
        self.Measurement[:] = 0
        self.MeasureNoiseCov[0,0] = 50.
        self.MeasureNoiseCov[1,1] = 50.
        self.MeasureNoiseCov[2,2] = 0.8;  self.MeasureNoiseCov[2,3] = 0.5
        self.MeasureNoiseCov[3,2] = 0.5;  self.MeasureNoiseCov[3,3] = 0.8
        
        self.MeasureBuffer = []
        
    def fetchData(self, measure_val):
        self.MeasureBuffer.append(measure_val)
    
    def updateMeasurement(self):
        if len(self.MeasureBuffer) > 0:
            self.tRawMeasureData = self.MeasureBuffer[-1]
            self.MeasureBuffer = []
            
            # ���й۲�ʱ���²���ֵ
            self.Measurement[0,0] = self.tRawMeasureData.f32Xm;
            self.Measurement[1,0] = self.tRawMeasureData.f32Ym;
            self.Measurement[2,0] = np.cos(self.tRawMeasureData.f32YawRad);
            self.Measurement[3,0] = np.sin(self.tRawMeasureData.f32YawRad);

            # ��������
            for row in range(4):
                for col in range(4):
                    self.MeasureNoiseCov[row,col] = \
                        self.tRawMeasureData.cov_world[row * 4 + col] * 10.
            
            return 1
        else:
            #  ���޹۲�ʱ��������(����һ���ܴ����)
            for row in range(4):
                self.MeasureNoiseCov[row,row] = np.Inf
        
            return 0
        
def main():
    pass
    
if __name__=="__main__":
    pass
    main()


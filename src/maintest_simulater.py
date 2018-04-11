#-*-coding:UTF-8-*-
'''
Created on 2018��4��10��
@author: mipapapa
'''
from common import LOC_POSTION, load_postion_logs
from veh_model import LOC_VEH_DATA, load_veh_logs
from sync_data import SyncData, create_sync_data
import numpy as np
import matplotlib.pyplot as plt
import json
from integrate_nav import LOC_InteNav
from collections import defaultdict

class Simulater:
    def __init__(self, sample_step = 4):
        with open("config.json","r") as f:
            self.config = json.load(f)
        pose_lst = load_postion_logs(self.config["pose_path"])
        can_lst = load_veh_logs(self.config["can_path"])
        self.sync_data_lst = create_sync_data(can_lst, pose_lst, sample_step)
    
    def disp_observe_pose(self):
        plt.figure()
        nframe = len(self.sync_data_lst)
        for i in range(nframe):
            sync_data = self.sync_data_lst[i]
            if sync_data.lock_pose == False:
                for p in range(len(sync_data.pose_data)):
                    pose  = sync_data.pose_data[p]
                    plt.plot(pose.f32Xm, pose.f32Ym, "b.")
        plt.show()

    
    def process(self):
        main_nav = LOC_InteNav()
        main_nav.init()
        nframe = len(self.sync_data_lst)

        display = defaultdict(list)
        for i in range(nframe):
            # ��ȡͬ�����ݼ���ǰ֡ʱ�����ѭ���м������е����ݾ�Ϊ˲�䴦����ɣ���������ʱ
            sync_data = self.sync_data_lst[i]
            time_stamp = sync_data.time
            
            # ��ȡ�����˶���Ϣ
            veh_data = sync_data.veh_data
            if veh_data is None: 
                continue

            # ��ȡ�۲���λ����Ϣ, ���в�������Ӧ�Ĺ۲��ߴ���������Ӧ
            if len(sync_data.pose_data) > 0 and sync_data.lock_pose == False:
                pose_data = sync_data.pose_data[0]
                main_nav.sub_modules[0].fetchData(pose_data)
            else:
                pose_data = None

            # ����Ƿ��������ʼ��������kalmanѭ��
            if main_nav.OriginInited == True:
                main_nav.odometryPrediction(veh_data, time_stamp)
                main_nav.multiSensorCorrect()
                result = main_nav.get_result()
                display["kalman"].append((result.f32Xm, result.f32Ym))
            
            else:
                main_nav.setStartPoint(time_stamp)
        
        plt.figure()
        display["kalman"] = np.array(display["kalman"])
        plt.plot(display["kalman"][:,0],display["kalman"][:,1], 'r-')
        plt.show()
        

def main():
    simu = Simulater(sample_step=8)
    simu.process()
    
if __name__=="__main__":
    pass
    main()


#-*-coding:UTF-8-*-
'''
Created on 2018年4月10日
@author: mipapapa
'''
from common import LOC_POSTION, load_postion_logs
from veh_model import LOC_VEH_DATA, load_veh_logs
import matplotlib.pyplot as plt

class SyncData:
    def __init__(self):
        self.time = 0
        self.pose_data = []
        self.veh_data = None
        self.lock_pose = False

def create_sync_data(can_lst, pose_lst, pose_step = 1):
    nCan = len(can_lst)
    nPos = len(pose_lst)
    # 获取CAN所有时间作为参考量
    time_can = []
    for i in range(nCan):
        time_can.append(can_lst[i].timestamp)
    
    time_pos = []
    for i in range(nPos):
        time_pos.append(pose_lst[i].i64TimeStamp)

    # 合并所有时间戳序列
    idx_can, idx_pos = 0, 0
    sync_data_lst, sync_time_lst = [], []

    for i in range(nCan + nPos):
        sync_data = SyncData()
        if time_can[idx_can] < time_pos[idx_pos]:
            sync_data.time = time_can[idx_can]
            sync_data.veh_data = can_lst[idx_can]
            idx_can += 1
            
        elif time_can[idx_can] > time_pos[idx_pos]:
            sync_data.time = time_pos[idx_pos]
            sync_data.pose_data.append(pose_lst[idx_pos]) 
            idx_pos += 1
        else:
            sync_data.time = time_can[idx_can]
            sync_data.veh_data = can_lst[idx_can]
            sync_data.pose_data.append(pose_lst[idx_pos])
            idx_can += 1
            idx_pos += 1
        sync_time_lst.append(sync_data.time)
        if idx_pos % pose_step != 0:
            sync_data.lock_pose = True
        sync_data_lst.append(sync_data)

        if idx_can == nCan and idx_pos == nPos:
            break
    
    return sync_data_lst

def main():
    pose_path = r"D:\test_data\kalman_data\0002\0002_VPS.log"
    veh_path = r"D:\test_data\kalman_data\0002\0002_CAN.log"
    pose_lst = load_postion_logs(pose_path)
    veh_lst = load_veh_logs(veh_path)
    sync_data_lst = create_sync_data(veh_lst, pose_lst, 2)

    plt.figure()
    for i in range(len(sync_data_lst)):
        data = sync_data_lst[i]
        if data.veh_data == None:
            plt.plot(i,0, "bx")
        elif len(data.pose_data) == 0:
            plt.plot(i,0, "r.")
        else:
            if data.lock_pose == False:
                plt.plot(i,0, "b.")
            else:
                plt.plot(i,0, "rx")
    plt.show()
    
    
if __name__=="__main__":
    pass
    main()


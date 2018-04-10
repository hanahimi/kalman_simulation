#-*-coding:UTF-8-*-
'''
Created on 2018Äê4ÔÂ10ÈÕ
@author: mipapapa
'''
import json

class LOC_POSTION:
    def __init__(self):
        self.f32Xm = 0
        self.f32Ym = 0
        self.f32YawRad = 0
        self.i64TimeStamp = 0
    
        self.cov_world = []
        self.status = 0

def load_postion_logs(json_path):
    pose_lst = []
    with open(json_path, "r") as fr:
        for s in fr:
            json_data = json.loads(s)
            pose = LOC_POSTION()
            pose.f32Xm = json_data["Xm"]
            pose.f32Ym = json_data["Ym"]
            pose.f32YawRad = json_data["Yawr"]
            pose.i64TimeStamp = json_data["time"]
            pose.cov_world = json_data["Cov"]
            pose_lst.append(pose)
    return pose_lst
            
def main():
    json_path = r"D:\test_data\kalman_data\0002\0002_VPS.log"
    pose_lst = load_postion_logs(json_path)
    print len(pose_lst)
    
if __name__=="__main__":
    pass
    main()


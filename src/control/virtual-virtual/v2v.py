import os,sys
sys.path.append(os.getcwd())

import numpy as np
import time
import threading
from src.control.utils.RflyDB import RflyDB
import src.control.utils.RflySoftwarm as RflySW
import src.control.utils.RflyMav as RflyMav
import src.control.utils.RflyDtrain as RflyDtrain

data_event = threading.Event()
end_event = threading.Event()
start_event = threading.Event()

class RflyDT:
    def __init__(self, mavs, sw) -> None:
        self.mavs = mavs
        self.sw = sw
        self.round_num = mavs[0].caseNum
        self.data_event_cnt = 0
        self.start_event_cnt = 0
        self.end_event_cnt = 0

    def all_mavOver(self):
        result = 1 if all(mav.round_over for mav in self.mavs) else 0
        return result
    
    def reendSim(self):
        if self.all_mavOver():
            self.sw.End()
    
    def restartSim(self):
        self.sw.Start()

    def resetEvent(self):
        for mav in self.mavs:
            self.data_event_cnt += mav.data_event_cnt
            self.start_event_cnt += mav.start_event_cnt
            self.end_event_cnt += mav.end_event_cnt
        
        if self.start_event_cnt == mav_num:
            start_event.clear()
            self.start_event_cnt = 0
        
        if self.data_event_cnt == mav_num:
            data_event.clear()
            self.data_event_cnt = 0

        if self.end_event_cnt == mav_num:
            end_event.clear()
            self.end_event_cnt = 0


def generate_faulty_mission(oscillate_axis='x', amplitude=1, frequency=1, total_time=10, time_step=0.1):

    time_points = np.arange(0, total_time, time_step)
    mission = []
    vx, vy, vz, yaw = 0, 0, 0, 0
    oscillation = amplitude * np.sign(np.sin(2 * np.pi * frequency * time_points))
    
    for osc in oscillation:
        if oscillate_axis == 'x':
            vx = osc
        elif oscillate_axis == 'y':
            vy = osc
        elif oscillate_axis == 'z':
            vz = osc
        
        mission.append([vx, vy, vz, yaw])
    
    return mission

if __name__ == "__main__":

    path = sys.path[0] + './db.json'
    db = RflyDB(path)

    mission = generate_faulty_mission(oscillate_axis='x', amplitude=1, frequency=1, total_time=5, time_step=0.01)
    
    
    drones = [20100,20102]
    missions = [mission, mission]
    mav_num = len(drones)
    sw = RflySW.RflySW(mav_num)
    sw.Start()
    start_event.set()

    mavs = []
    threads = []
    event = [data_event, start_event, end_event]
    for drone,mission in zip(drones, missions):
        mav = RflyMav.RflyMav(drone, event, mission)
        thread = threading.Thread(target=mav.run)
        mavs.append(mav)
        threads.append(thread)
    
    mavDT = RflyDT(mavs, sw)
    
    for thread in threads:
        thread.start()
    
    round_index = 0
    while True:
        if mavDT.all_mavOver():
            print('All mav finished! Closing sim!')
            mavDT.reendSim()
            end_event.set()
            round_index += 1
            print('Starting collect train data!')
            # 1) get train data
            for mav in mavs:
                Dtrain = RflyDtrain.RflyDtrain()
                Dtrain.get_normal_train_data(mav.dp.LogPath, mav.dp.NTrainDataPath, mav.ctrlSeq)
            
            # 2) get DT err data
            basePath = mavs[0].dp.datap
            savePath = basePath + '/err'
            DTtrain = RflyDtrain.RflyDtrain()
            DTtrain.get_and_save_DT(basePath, savePath)

            data_event.set()
 
            if round_index < mavDT.round_num:
                print('Start a new round of simulation!')
                sw = RflySW.RflySW(mav_num)
                mavDT.sw = sw
                mavDT.restartSim()
                start_event.set()
                time.sleep(5)
            else:
                print("All mav test finished! exit")
                break
            
            
        mavDT.resetEvent()


                

    '''
    # step4:生成误差数据,用于实时诊断
    while True:
        alive = True
        for mav in mavs:
            if mav.is_alive is not True:
                alive = False
        
        if alive:
            full = False
            for mav in mavs:
                if mav.mavPosNED.is_pool_full():
                    full = True
                else:
                    full = False
            
            if full:
                mavPosNED_err = get_err(mav1_data=mavs[0].mavPosNED, mav2_data=mavs[1].mavPosNED)
                print('mavPosNED_err:',mavPosNED_err)
                time.sleep(2)
    '''
    


        






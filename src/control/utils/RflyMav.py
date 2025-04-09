import os,sys
import src.control.utils.PX4MavCtrlV4 as PX4MavCtrl
import numpy as np
import time
import src.control.utils.RflyDB as RflyDB
import src.control.utils.RflyDP as RflyDP
import src.control.utils.RflyCtrl as RflyCtrl
import re
import collections
import UE4CtrlAPI
ue = UE4CtrlAPI.UE4CtrlAPI()

class MAVREG:
    json_path = os.path.join(sys.path[0],'db.json')
    mav_num = 0


class DataPool:
    def __init__(self, pool_size = 80):
        self.max_size = pool_size
        self.pool = collections.deque(maxlen=pool_size)

    def add_data(self, timestamp, data, flag=None):
        timestamp = time.time()  
        self.pool.append({'timestamp': timestamp, 'data': data, 'flag': flag})

    def is_pool_full(self):
        return len(self.pool) == self.max_size

    def __iter__(self):
        return iter(self.pool)

class RflyMav:
    def __init__(self, port, event, mission = None):
        # Basic connection config
        self.ID = int((port-20100)/2+1)
        self.port = port
        self.mav = PX4MavCtrl.PX4MavCtrler(self.port)
        self.mission = mission

        # Init software obj
        self.frame = 1
        self.event = event
        MAVREG.mav_num += 1
        
        # data received HZ
        self.hz = 500
        self.is_alive = False
        self.round_over = False
        self.data_event_cnt = 0
        self.start_event_cnt = 0
        self.end_event_cnt = 0

        # Init test case
        self.caseIndex = 0
        self.caseID, self.caseNum, self.caseInfo, self.ctrlSeq = self.get_cmd()
        print(f'mav{self.ID} case {self.caseID} ctrlSeq: ',self.ctrlSeq)

        self.lastTime = time.time()

    def reinit(self):
        # Init test case
        self.startTime = time.time()
        self.endTime = time.time()
        self.lastTime = time.time()
        self.MavCmdInd = 0
        self.MavCmdNum = len(self.ctrlSeq)

        # Init control sequence class object
        self.CFID = RflyCtrl.CmdCtrl(self.mav,self.frame,self.ID) 
        self.CID1OBJ = self.CFID.CID1
        self.CID2OBJ = self.CFID.CID2

        # Init data pool which received from px4 
        self.mavPosNED = DataPool()      # Estimated Local Pos (related to takeoff position) from PX4 in NED frame
        self.mavVelNED = DataPool()      # Estimated local velocity from PX4 in NED frame
        self.mavAccB = DataPool()        # Estimated acc from PX4
        self.mavGyro = DataPool()        # Estimated Gyro from PX4
        self.mavMag = DataPool()         # Estimated Mag from PX4
        self.mavVibr = DataPool()        # Estimated vibration xyz from PX4
        self.mavAngEular = DataPool()    # Estimated Eular angles from PX4
        self.mavAngRate = DataPool()     # Estimated angular rate from PX4
        self.mavAngQuatern = DataPool()  # Estimated AngQuatern from PX4

        self.normalPos = np.array([0,0,0]) 
        self.normalVel = np.array([0,0,0]) 
        self.normalAccB = np.array([0,0,0])
        self.normalGyro = np.array([0,0,0])
        self.normalMag = np.array([0,0,0])
        self.normalVibr = np.array([0,0,0])
        self.normalAngEular = np.array([0,0,0])
        self.normalAngRate = np.array([0,0,0])
        self.normalAngQuatern = np.array([0,0,0,0])

        self.faultPos = np.array([0,0,0]) 
        self.faultVel = np.array([0,0,0]) 
        self.faultAccB = np.array([0,0,0])
        self.faultGyro = np.array([0,0,0])
        self.faultMag = np.array([0,0,0])
        self.faultVibr = np.array([0,0,0])
        self.faultAngEular = np.array([0,0,0])
        self.faultAngRate = np.array([0,0,0])
        self.faultAngQuatern = np.array([0,0,0,0])
        
        # Init flight index variable
        self.FLYIND = 0 
        self.INJECTIND = 0 
        self.INJECTFLAG = False
        self.INJECTTIME = 0
        self.EXITIND = 0 
        self.EXITFLAG = False
        self.EXITTIME = 0

        self.is_alive = False
        self.round_over = False
        self.data_event_cnt = 0
        self.end_event_cnt = 0
        self.start_event_cnt = 0


    def get_cmd(self):
        db = RflyDB.RflyDB(MAVREG.json_path)
        caseNum = len(db.GET_CASEID()[self.ID - 1])
        caseID = db.GET_CASEID()[self.ID - 1][self.caseIndex]
        caseInfo = db.GET_CASEINFO(caseID)
        ctrlSeq = caseInfo.get('ControlSequence')
        case = re.split(';',ctrlSeq)
        cmd = np.array([])
        for i in range(len(case)):
            cmd = np.append(cmd,case[i])
        
        return caseID, caseNum, caseInfo, cmd

    def init_connection(self):
        self.mav.InitMavLoop(UDPMode=2)
        time.sleep(0.5)
        self.mav.InitTrueDataLoop()
        time.sleep(0.5)
        self.mav.initOffboard()
        time.sleep(0.5)

        self.mav.SendMavArm(1)
    
    def end_connection(self):
        self.mav.EndTrueDataLoop()
        time.sleep(0.5)
        self.mav.endMavLoop() 
        time.sleep(1)
    
    def trigger(self,ctrlseq):
        cmdseq = ctrlseq # '2,3,0,0,-20'
        cmdseq = re.findall(r'-?\d+\.?[0-9]*',cmdseq) # ['2', '3', '0', '0', '-20']
        cmdCID = cmdseq[0]
        if  cmdCID in self.CFID.CID:
            FID = self.CFID.FIDPro(cmdCID)
            # if has param
            if len(cmdseq) > 2:
                # get param
                param = cmdseq[2:len(cmdseq)]
                param = [float(val) for val in param]
                FID[cmdseq[1]](param)

            else:
                '''mission mode'''
                if cmdseq[1] == '8':
                    FID[cmdseq[1]](self.mission)
                else:
                    FID[cmdseq[1]]()
        else:
            print(f'mav{self.ID} Command input error, please re-enter')
    
    def run(self):
        # Start time and end time (unlock after startup to prevent the ground station from not starting timing)
        ue.sendUE4Cmd('RflyShowTextTime "Sim Start" 10')
        print(f'mav{self.ID} Sim start')
        while True: 
            self.lastTime = self.lastTime + (1.0/self.hz)
            sleepTime = self.lastTime - time.time()
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                self.lastTime = time.time()

            if self.caseIndex >= self.caseNum:
                print(f'mav{self.ID} all case test finish!')
                break

            # 1.1): Block and wait for the main thread to open the simulation software
            '''self.event[1] --> start_event'''
            if self.caseIndex >= 1:
                self.mav = PX4MavCtrl.PX4MavCtrler(self.port)
                self.event[1].wait()
            # step1: init_connection
            self.reinit()
            self.start_event_cnt += 1
            self.init_connection()
            self.is_alive = True

            # step2: run
            while True:
                # 250HZ receiving data
                self.lastTime = self.lastTime + (1.0/self.hz)
                sleepTime = self.lastTime - time.time()
                if sleepTime > 0:
                    time.sleep(sleepTime)
                else:
                    self.lastTime = time.time()
                
                # Starting receiving data at 250HZ using fault diagnoise
                mavTimestamp = self.mav.uavTimeStmp
                mavPosNED = self.mav.uavPosNED
                mavVelNED = self.mav.uavVelNED  
                mavAccB = self.mav.uavAccB   
                mavGyro = self.mav.uavGyro        
                mavMag = self.mav.uavMag   
                mavVibr = self.mav.uavVibr   
                mavAngEular = self.mav.uavAngEular   
                mavAngRate = self.mav.uavAngRate   
                mavAngQuatern = self.mav.uavAngQuatern

                self.mavPosNED.add_data(timestamp=mavTimestamp,data=mavPosNED)
                self.mavVelNED.add_data(timestamp=mavTimestamp,data=mavVelNED)
                self.mavAccB.add_data(timestamp=mavTimestamp,data=mavAccB)
                self.mavGyro.add_data(timestamp=mavTimestamp,data=mavGyro)
                self.mavMag.add_data(timestamp=mavTimestamp,data=mavMag)
                self.mavVibr.add_data(timestamp=mavTimestamp,data=mavVibr)
                self.mavAngEular.add_data(timestamp=mavTimestamp,data=mavAngEular)
                self.mavAngRate.add_data(timestamp=mavTimestamp,data=mavAngRate)
                self.mavAngQuatern.add_data(timestamp=mavTimestamp,data=mavAngQuatern)             

                # Starting receiving data at 250HZ using normal data collecting
                if self.CID2OBJ.INJECTFLAG is not True:
                    self.normalPos = np.row_stack((self.normalPos,mavPosNED))
                    self.normalVel = np.row_stack((self.normalVel,mavVelNED))
                    self.normalAccB = np.row_stack((self.normalAccB,mavAccB))
                    self.normalGyro = np.row_stack((self.normalGyro,mavGyro))
                    self.normalMag = np.row_stack((self.normalMag,mavMag))
                    self.normalVibr = np.row_stack((self.normalVibr,mavVibr))
                    self.normalAngEular = np.row_stack((self.normalAngEular,mavAngEular))
                    self.normalAngRate = np.row_stack((self.normalAngRate,mavAngRate))
                    self.normalAngQuatern = np.row_stack((self.normalAngQuatern,mavAngQuatern))
                    pass
                else:
                    self.faultPos = np.row_stack((self.faultPos,mavPosNED))
                    self.faultVel = np.row_stack((self.faultVel,mavVelNED))
                    self.faultAccB = np.row_stack((self.faultAccB,mavAccB))
                    self.faultGyro = np.row_stack((self.faultGyro,mavGyro))
                    self.faultMag = np.row_stack((self.faultMag,mavMag))
                    self.faultVibr = np.row_stack((self.faultVibr,mavVibr))
                    self.faultAngEular = np.row_stack((self.faultAngEular,mavAngEular))
                    self.faultAngRate = np.row_stack((self.faultAngRate,mavAngRate))
                    self.faultAngQuatern = np.row_stack((self.faultAngQuatern,mavAngQuatern))
                
                

                # Processing instruction sequence
                self.trigger(self.ctrlSeq[self.MavCmdInd])
                # If one instruction sequence is completed, the next instruction is processed
                if re.findall(r'-?\d+\.?[0-9]*',self.ctrlSeq[self.MavCmdInd])[0] == '1' and self.CID1OBJ.isDone == 1 or re.findall(r'-?\d+\.?[0-9]*',self.ctrlSeq[self.MavCmdInd])[0] == '2' and self.CID2OBJ.isDone == 1:
                    self.MavCmdInd = self.MavCmdInd + 1

                if self.CID2OBJ.INJECTFLAG == True and self.INJECTFLAG == False:
                    self.INJECTIND = self.FLYIND 
                    self.INJECTTIME = round(time.time() - self.startTime)
                    self.INJECTFLAG = True
                
                if self.MavCmdInd >= self.MavCmdNum and self.EXITFLAG == False:
                    self.EXITIND = self.FLYIND 
                    self.EXITTIME = round(time.time() - self.startTime)
                    self.EXITFLAG = True
                    print(f'mav{self.ID}: CaseID {self.caseID} test completed')
                    ue.sendUE4Cmd('RflyShowTextTime "CaseID %d test completed!" 10'%(self.caseID))
                    self.endTime = time.time()
                    break
                
            self.caseIndex += 1
            self.end_connection()
            

            '''Start data processing thread'''
            Info = self.ctrlSeq
            print(f"mav{(self.ID)}:Start data processing")
            self.dp = RflyDP.RflyDP(self.ID, Info)
            self.dp.run(MAVREG.mav_num)
            print(f"mav{(self.ID)}:End data processing")
            
            # this round is over, starting next round
            self.round_over = True
            '''self.event[2] --> end_event'''
            self.event[2].wait()
            self.end_event_cnt += 1

            # wating main threading overing train data process
            print(f'mav{self.ID} wating unified train data process!')
            '''self.event[0] --> data_event'''
            self.event[0].wait()
            self.data_event_cnt += 1



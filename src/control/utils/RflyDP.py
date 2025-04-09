import os,sys
import datetime
import pandas as pd
import shutil
import csv
import time
import threading

date = datetime.date.today()
dp = os.path.join(os.getcwd(), 'data', f'{date}')
if not os.path.exists(dp):
    os.makedirs(dp)  
else:
    pass  


def get_num(directory):
    files = os.listdir(directory)
    file_numbers = []
    for file_name in files:
        try:
            file_number = int(file_name)
            file_numbers.append(file_number)
        except (ValueError, IndexError):
            pass
    
    if not file_numbers:
        return 0
    
    return max(file_numbers)


class DataREG():
    lock = threading.Lock()
    RFLY_SPVO_MODE = False
    end_cnt = 0
    CMD_MAP = {
    '1': 'Armed!',
    '2': 'Disarmed!',
    '3': 'Send takeoff command, fly target position [{},{},{}]',
    '4': 'Send fixed-pos flight command, fly target pos [{},{},{}]',
    '5': 'Send fixed-vel flight command, fly target vel [{},{},{}]',
    '6': 'Send hover command',
    '7': 'Send Landing command',
    '8': 'Send Mission command',
    '9': 'Send fault injection command \n Fault injection type:{} \n Fault injection parameters:{}',
    '10': 'Send stopFlag command:[{},{}]'
    }

    cnt = 0
    
    
    def InfoDowm(InfoPath, ID, Info, DataPath):
        DataREG.lock.acquire()
        if not os.path.exists(InfoPath):
            header = [
                ['Date', 'CaseID', 'TestInfo','DataPath']
            ]
            df = pd.DataFrame(header)
            df.to_csv(InfoPath, index=False, header=False)

        date = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
        caseID = ID
        testinfo = DataREG.CMDAna(Info) 
        datapath = DataPath
        
        dataInfo = [
            [date, caseID, testinfo, datapath]
        ]

        ff = pd.DataFrame(dataInfo)
        ff.to_csv(InfoPath, mode='a', index=False, header=False)
        DataREG.lock.release()
    
    def CMDAna(cmd):
        subcmd = ';'.join(cmd)
        subcmd = subcmd.split(';')

        mtime = 0
        info = ''

        if subcmd[0][0] == '2':
            for index, cmd in enumerate(subcmd):
                cmd = cmd.split(',')
                if cmd[0] == '2':
                    info += f'{mtime}s: '
                    if cmd[1] == '1' or cmd[1] == '2' or cmd[1] == '6' or cmd[1] == '7' or cmd[1] == '8':
                        info += DataREG.CMD_MAP[cmd[1]]
                    elif cmd[1] == '3' or cmd[1] == '4' or cmd[1] == '5':
                        if not DataREG.RFLY_SPVO_MODE:
                            info += DataREG.CMD_MAP[cmd[1]].format(cmd[2],cmd[3],cmd[4])  
                        else:
                            info += DataREG.CMD_MAP[cmd[1]].format('SPVO','SPVO','SPVO')
                    elif cmd[1] == '9':
                        fp = [float(str) for str in cmd[3:] if str != cmd[2]]
                        info += DataREG.CMD_MAP[cmd[1]].format(cmd[2],fp)
                    elif cmd[1] == '10':
                        info += DataREG.CMD_MAP[cmd[1]].format(cmd[2],cmd[3])
                    info += ' \n'

                if cmd[0] == '1':
                    mtime += float(cmd[2])
                    if index == len(subcmd) - 1:
                        info += f'{mtime}s: Exit test!'
                
        elif subcmd[0][0] == '1':
            index = 0
            for i in range(len(subcmd)):
                if subcmd[i][0] != '1':
                    index = i
                    break
            
            for index, cmd in enumerate(subcmd[index:], start=index):
                cmd = cmd.split(',')
                if cmd[0] == '2':
                    info += f'{mtime}s: '
                    if cmd[1] == '1' or cmd[1] == '2' or cmd[1] == '6' or cmd[1] == '7' or cmd[1] == '8':
                        info += DataREG.CMD_MAP[cmd[1]]
                    elif cmd[1] == '3' or cmd[1] == '4' or cmd[1] == '5':
                        if not DataREG.RFLY_SPVO_MODE:
                            info += DataREG.CMD_MAP[cmd[1]].format(cmd[2],cmd[3],cmd[4])  
                        else:
                            info += DataREG.CMD_MAP[cmd[1]].format('SPVO','SPVO','SPVO')
                    elif cmd[1] == '9':
                        fp = [float(str) for str in cmd[3:] if str != cmd[2]]
                        info += DataREG.CMD_MAP[cmd[1]].format(cmd[2],fp)
                    elif cmd[1] == '10':
                        info += DataREG.CMD_MAP[cmd[1]].format(cmd[2],cmd[3])
                    info += ' \n'

                if cmd[0] == '1':
                    mtime += float(cmd[2])
                    if index == len(subcmd) - 1:
                        info += f'{mtime}s: Exit test!'
        return info

    def reinit(mavnum):
        if DataREG.end_cnt == mavnum:
            DataREG.end_cnt = 0
            DataREG.cnt = 0

class RflyDP():
    def __init__(self,mavID,Info):
        self.ID = mavID
        self.Info = Info
        
        self.datap = self.get_datapath()
        
        self.DataPath = self.datap + f'\mav{self.ID}'
        self.LogPath = self.DataPath + '\log'
        self.NTrainDataPath = self.DataPath + '/ntrain data'
        self.FTrainDataPath = self.DataPath + '/ftrain data'
        self.ErrorDataPath = self.DataPath + '/error data'
        self.InfoPath = self.datap + '\info.csv'
        
    def get_datapath(self):
        DataREG.lock.acquire()
        if DataREG.cnt == 0:
            datap = os.path.join(f'{dp}', f'{get_num(dp)+1}')
        else:
            datap = os.path.join(f'{dp}', f'{get_num(dp)}')
        self.creatFile(datap)
        DataREG.cnt += 1
        DataREG.lock.release()
        return datap

    def creatFile(self, path):
        if not os.path.exists(path):
            os.makedirs(path)  
        else:
            pass

    def run(self, mavnum):
        self.LogDown()
        self.InfoDown()
        DataREG.end_cnt += 1
        DataREG.reinit(mavnum)
        
    def LogDown(self):
        '''
        # QGC下载接口
        qgc = QGCCtrlAPI.QGCCtrlAPI()
        logName = qgc.ReqQgcLog(AutoREG.LOG_TIMEOUT, self.Info[2])
        if logName!='':
                shutil.copyfile(qgc.QGCPath+'\\'+logName, self.logp + '\\' + logName)
                print('Download log ' + logName+ f' for mav{self.Info[2]} successfully.')
        '''
        DataREG.lock.acquire()
        self.PlatFormpath = 'C:/PX4PSP'
        # 1、List the directories under the file
        log_path = self.PlatFormpath + f'/Firmware/build/px4_sitl_default/instance_{self.ID}/log'

        PlatForm_log_dirs = os.listdir(log_path) 
        log_data = PlatForm_log_dirs[len(PlatForm_log_dirs)-1]
        path = os.path.join(log_path,log_data) 
        dirs = os.listdir(path) 

        # 2、Get the latest ulg file
        ulg = dirs[len(dirs)-1]
        ulgPath = os.path.join(path,ulg) 

        # 3、Copy the ulg file to the log folder
        TargetPath_log = self.LogPath + '/{}'.format(ulg)

        self.creatFile(self.LogPath)
        shutil.copyfile(ulgPath, TargetPath_log) 
        # print(f'Successfully download mav{self.ID} log file to {TargetPath_log} path')

        for root, dirs, files in os.walk(self.LogPath):
            for file in files:
                if file.endswith('.ulg'):
                    current_dir = os.path.abspath(root)
                    os.chdir(current_dir)
                    cmd = f"ulog2csv {file}"
                    os.system(cmd)
        DataREG.lock.release()
    
    def px4NDataDown(self, normaldata):
        self.creatFile(self.NormalDataPath)
        normaldata_csv_path = self.NormalDataPath + '/normal.csv'
        with open(normaldata_csv_path, 'w', newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z', 'mag_x', 'mag_y', 'mag_z'])
            for row in normaldata:
                writer.writerow(row)
    
    def px4FDataDown(self, faultdata):
        self.creatFile(self.FaultDataPath)
        faultdata_csv_path = self.FaultDataPath + '/fault.csv'
        with open(faultdata_csv_path, 'w', newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z', 'mag_x', 'mag_y', 'mag_z'])
            for row in faultdata:
                writer.writerow(row)
                
    def InfoDown(self):
        DataREG.InfoDowm(self.InfoPath, self.ID, self.Info, self.DataPath)
        # print(f'Successfully download mav{self.ID} info file to {self.InfoPath} path')

    
        

# import re
# import numpy as np
# import RflyDB

# json_path = os.path.join(sys.path[0], '..','virtual-virtual','db.json')
# db = RflyDB.RflyDB(json_path)
# caseID = db.GET_CASEID()[0][0]
# caseInfo = db.GET_CASEINFO(caseID)
# ctrlSeq = caseInfo.get('ControlSequence')
# case = re.split(';',ctrlSeq)
# cmd = np.array([])
# for i in range(len(case)):
#     cmd = np.append(cmd,case[i])        

# a = np.random.rand(10,3)
# b = np.random.rand(10,3)
# c = np.random.rand(10,3)
# Info = cmd
# Data = [a,b,c]
# dp = RflyDP(1, Data, Info)





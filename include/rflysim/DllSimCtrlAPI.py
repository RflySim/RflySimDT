import socket
import threading
import time
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2
import struct
import math
import sys
import copy
import os
import numpy as np
import ctypes

class DllSimCtrlAPI:

    # constructor function
    def __init__(self, ip='127.0.0.1'):
        self.ip = ip
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create socket
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) 

        
    # 发送一个16维double型数据到指定端口，本接口可用于与Simulink通信
    # struct CustomData{
    #     int checksum;
    #     int CopterID;
    #     double data
    # } ii16d 136包长
    def sendCustomData(self,CopterID=0,data=[0]*16,checksum=123456,port=50000,IP='127.0.0.1'):
        """ Send 16d message to desired IP and port
        """
        buf = struct.pack("ii16d",checksum,CopterID,*data)
        self.udp_socket.sendto(buf, (IP, port))   
   
    # 发送到DLL模型的inSILInts and inSILFLoats接口
    #    struct PX4SILIntFloat{
    #     int checksum;//1234567897
    #     int CopterID;
    #     int inSILInts[8];
    #     float inSILFLoats[20];
    #    }
    # inSILInts and inSILFLoats will send to DLL model's inputs
    # CopterID is the vehicle you want to send, if copterID=-1 then it will send to yourself.
    def sendSILIntFloat(self,inSILInts=[0]*8,inSILFLoats=[0]*20,copterID=-1):
        checkSum=1234567897
        ID=copterID
        if copterID<=0:
            ID=self.CopterID
        PortNum = 30100+(ID-1)*2 
        buf = struct.pack("10i20f",checkSum,ID,*inSILInts,*inSILFLoats)
        self.udp_socket.sendto(buf, (self.ip, PortNum))


    # 发送到DLL模型的InCtrlExt系列接口
    # CopterID is the vehicle you want to send, if copterID=-1 then it will send to yourself.
    def sendInCtrlExt(self,inSILInts=[0]*8,inSILFLoats=[0]*20,iDxNum=1,copterID=-1):
        checkSum=1234567800+iDxNum
        ID=copterID
        if copterID<=0:
            ID=self.CopterID
        PortNum = 30100+(ID-1)*2 
        # pack for SOut2SimulatorSimpleTime
        buf = struct.pack("10i20f",checkSum,ID,*inSILInts,*inSILFLoats)
        self.udp_socket.sendto(buf, (self.ip, PortNum))

        
    # //输出到CopterSim DLL模型的FaultParamAPI.FaultInParams[32]参数
    # struct PX4ModelInParams{
    #     int checksum;//1234567891
    #     uint32 Bitmask;
    #     double InParams[32];
    # };
    #struct.pack iI32d
    
    # CopterID is the vehicle you want to send, if copterID then it will send to yourself.
    # bitmask has 32bits, each bit == 1 means modify the params in ModelInParams
    def sendModelInParams(self,Bitmask=0,InParams=[0]*32,copterID=-1):
        checkSum=1234567891
        Bitmask=ctypes.c_uint32(Bitmask).value
        ID=copterID
        if copterID<=0:
            ID=self.CopterID
        PortNum = 30100+(ID-1)*2 
        buf = struct.pack("iI32d",checkSum,Bitmask,*InParams)
        self.udp_socket.sendto(buf, (self.ip, PortNum))

    # //输出到CopterSim DLL模型的FaultParamAPI.InitInParams参数
    # struct PX4ModelInParams{
    #     int checksum;//1234567892 for InitInParams //注意，这里主要是checksum的区别
    #     uint32 Bitmask;
    #     double InParams[32];
    # };
    #struct.pack iI32d
    # CopterID is the vehicle you want to send, if copterID then it will send to yourself.
    # bitmask has 32bits, each bit == 1 means modify the params in ModelInParams
    def sendInitInParams(self,Bitmask=0,InParams=[0]*32,copterID=-1):
        checkSum=1234567892 # 这里主要是checksum的区别
        Bitmask=ctypes.c_uint32(Bitmask).value
        ID=copterID
        if copterID<=0:
            ID=self.CopterID
        PortNum = 30100+(ID-1)*2 
        buf = struct.pack("iI32d",checkSum,Bitmask,*InParams)
        self.udp_socket.sendto(buf, (self.ip, PortNum))

    # //输出到CopterSim DLL模型的InFromUE接口
    # struct UEToCopterDataD{
    #     int checksum; //1234567899为校验ID
    #     int CopterID; //发出本消息的飞机ID
    #     double inFromUE[32]; //通过蓝图发出的数据
    # }
    #struct.pack ii32d
    # CopterID is the vehicle you want to send, if copterID then it will send to yourself.
    def sendUE2Coptersim(self,inFromUE=[0]*32,copterID=-1):
        checkSum=1234567899 # 这里主要是checksum的区别
        ID=copterID
        if copterID<=0:
            ID=self.CopterID
        PortNum = 30100+(ID-1)*2 
        buf = struct.pack("ii32d",checkSum,copterID,*inFromUE)
        self.udp_socket.sendto(buf, (self.ip, PortNum))


    # //输出到CopterSim DLL模型的inFloatsCollision接口
    # struct Ue4RayTraceDrone {
    #     int checksum;//校验码1234567890
    #     int CopterID;
    #     float size;
    #     float velE[3];
    #     float ray[6];//前后左右上下
    #     float posE[3];
    # }
    #struct.pack ii13f
    # CopterID is the vehicle you want to send, if copterID then it will send to yourself.
    def sendUE2Coptersim(self,size=0,velE=[0,0,0],ray=[0,0,0,0,0,0],posE=[0,0,0],copterID=-1):
        checkSum=1234567899 # 这里主要是checksum的区别
        ID=copterID
        if copterID<=0:
            ID=self.CopterID
        PortNum = 30100+(ID-1)*2 
        buf = struct.pack("ii13f",checkSum,copterID,size,*velE,*ray,*posE)
        self.udp_socket.sendto(buf, (self.ip, PortNum))


class RflySimCP:
    """
    RflySim 综合模型控制协议
    """
    # inSILInts数据对应的下
    ILen = 8
    ICmd = 0
    IOffboard = 1
    # 整数型纬度
    ILat = 6
    # 整数型高度
    ILon = 7

    # inSILInts[0]指令标志
    CmdEn = 1
    CmdSIL = 1 << 1
    CmdArmed = 1 << 2
    CmdTakeoff = 1 << 8
    CmdPosition = 1 << 9
    CmdLand = 1 << 10
    CmdReturn = 1 << 11
    CmdOffboard_Pos = 1 << 16
    CmdOffboard_Att = 1 << 17
    CmdBase = 3

    # inSILInts[1] Offboard标志
    HasPos = 1
    HasVel = 1 << 1
    HasAcc = 1 << 2
    HasYaw = 1 << 3
    HasYawRate = 1 << 4
    HasAtt = 1 << 8
    HasRollRate = 1 << 9
    HasPitchRate = 1 << 10
    HasThrust = 1 << 11
    HasFull = 1 << 15
    NED = 1 << 16
    Global = 1 << 17

    # inSILFloats位置、速度、加速度、姿态、角速率、油门数据对应的起始下标
    FLen = 20
    FPos = 0
    FVel = 3
    FAcc = 6
    FAtt = 9
    FAttRate = 12
    FThrust = 15

    @staticmethod
    def IsBitSet(num, bit):
        mask = 1 << bit
        return (num & mask) != 0

    @staticmethod
    def IsPosSet(num):
        return RflySimCP.IsBitSet(num, 0)

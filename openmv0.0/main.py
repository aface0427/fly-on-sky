#************************************ (C) COPYRIGHT 2019 ANO ***********************************#
import sensor, image, time, math, struct
import json
from pyb import LED,Timer
from struct import pack, unpack
import Message,LineFollowing,DotFollowing,find_apriltags01,molcircle,pole

'''
#初始化镜头
sensor.reset()
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_transpose(False)
sensor.set_pixformat(sensor.RGB565)#设置相机模块的像素模式
sensor.set_framesize(sensor.QVGA)#设置相机分辨率240*160
Message.Ctr.WorkMode=0 #测试
'''

'''2
#sensor.reset()
#sensor.set_vflip(True)
#sensor.set_hmirror(True)
#sensor.set_transpose(False)
#sensor.set_pixformat(sensor.GRAYSCALE)
#sensor.set_framesize(sensor.HQVGA) # we run out of memory if the resolution is much bigger...
#sensor.set_windowing((160,160))
#sensor.skip_frames(10)
#sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
#sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...
#clock = time.clock()
#Message.Ctr.WorkMode=2 #测试AprilTag检测用
'''

'''3
sensor.reset() # 初始化 sensor.#初始化摄像头
sensor.set_pixformat(sensor.GRAYSCALE) # or sensor.RGB565#设置图像色彩格式，有RGB565色彩图和GRAYSCALE灰度图两种
sensor.set_framesize(sensor.QQVGA) # or sensor.QVGA (or others)
sensor.set_windowing((160,160))#设置图像像素大小
sensor.skip_frames(10) # 让新的设置生效
clock = time.clock() # 跟踪FPS帧率
# 在OV7725 sensor上, 边缘检测可以通过设置sharpness/edge寄存器来增强。
# 注意:这将在以后作为一个函数实现
if (sensor.get_id() == sensor.OV7725):
    sensor.__write_reg(0xAC, 0xDF)
    sensor.__write_reg(0x8F, 0xFF)
Message.Ctr.WorkMode=3 #测试Molcircle用
'''

sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # use RGB565.
sensor.set_framesize(sensor.VGA) # use QQVGA for speed.
sensor.set_windowing(300,70)
sensor.skip_frames(10) # Let new settings take affect.
sensor.set_auto_whitebal(False) # turn this off.
sensor.set_auto_gain(True)
clock = time.clock() # Tracks FPS.
Message.Ctr.WorkMode=4 #测试红色杆检测用

#主循环
while(True):
    clock.tick()#时钟初始化
    #接收串口数据
    if Message.Ctr.WorkMode==0:#点检测
        DotFollowing.DotCheck()
    elif Message.Ctr.WorkMode==1:#线检测
        LineFollowing.LineCheck()
    elif Message.Ctr.WorkMode==2:#AprilTag检测
        find_apriltags01.Find_Apriltags()
    elif Message.Ctr.WorkMode==3:#摩尔环
        molcircle.Molcircle()
    elif Message.Ctr.WorkMode==4:#红色杆
        pole.findingpole()

    #Message.Ctr.WorkMode += 1
    #Message.Ctr.WorkMode = Message.Ctr.WorkMode %3


    #LastWorkMode = Message.Ctr.WorkMode
    #用户数据发送
    #Message.UartSendData(Message.UserDataPack(127,127,32767,32767,65536,65536,65536,65536,65536,65536))
    #计算程序运行频率
'''
    if Message.Ctr.IsDebug == 1:
        fps=int(clock.fps())
        Message.Ctr.T_ms = (int)(1000/fps)
        print('fps',fps,'T_ms',Message.Ctr.T_ms)
'''

#************************************ (C) COPYRIGHT 2019 ANO ***********************************#

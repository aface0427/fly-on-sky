#************************************ (C) COPYRIGHT 2019 ANO ***********************************#
import sensor, image, time, math, struct
import json
from pyb import LED,Timer
from struct import pack, unpack
import Message,LineFollowing,DotFollowing,ColorRecognition,QRcode,Photography,find_apriltags01,molcircle
#初始化镜头
sensor.reset()
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_transpose(False)
#sensor.set_pixformat(sensor.RGB565)#设置相机模块的像素模式
#sensor.set_framesize(sensor.QVGA)#设置相机分辨率240*160


sensor.set_pixformat(sensor.GRAYSCALE) # or sensor.RGB565#设置图像色彩格式，有RGB565色彩图和GRAYSCALE灰度图两种
sensor.set_framesize(sensor.HQVGA) # or sensor.QVGA (or others)
sensor.set_windowing((160,160))#设置图像像素大小
sensor.skip_frames(10) # 让新的设置生效


#sensor.skip_frames(30)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)#若想追踪颜色则关闭白平衡
clock = time.clock()#初始化时钟
Message.Ctr.WorkMode=6 #测试Molcircle用
if (sensor.get_id() == sensor.OV7725):
    sensor.__write_reg(0xAC, 0xDF)
    sensor.__write_reg(0x8F, 0xFF)

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
    elif Message.Ctr.WorkMode==3:#颜色识别
        ColorRecognition.ColorRecognition()
    elif Message.Ctr.WorkMode==4:#二维码识别
        QRcode.ScanQRcode()
    elif Message.Ctr.WorkMode==5:#拍照
        Photography.Photography('IMG.jpg',10)
    elif Message.Ctr.WorkMode==6:#摩尔环
        molcircle.Molcircle()

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

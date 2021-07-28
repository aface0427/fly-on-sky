#************************************ (C) COPYRIGHT 2019 ANO ***********************************#
from pyb import UART
import LineFollowing
uart = UART(1,115200)#初始化串口 波特率 500000

# WorkMode=1为寻点模式
# WorkMode=2为寻线模式 包括直线 转角
# WorkMode=3为颜色识别模式
# WorkMode=4为识别二维码模式
# WorkMode=5为拍照模式
class Ctrl(object):
    WorkMode = 2   #工作模式
    IsDebug = 1     #不为调试状态时关闭某些图形显示等，有利于提高运行速度
    T_ms = 0
#类的实例化
Ctr=Ctrl()

def UartSendData(Data):
    uart.write(Data)

#点检测数据打包
def DotDataPack(color,flag,x,y,T_ms):
    print("found: x=",x,"  y=",-y)
    pack_data=bytearray([0xAA,0x29,0x05,0x41,0x00,color,flag,x>>8,x,(-y)>>8,(-y),T_ms,0x00])
    lens = len(pack_data)#数据包大小
    pack_data[4] = 7;#有效数据个数
    i = 0
    sum = 0
    #和校验
    while i<(lens-1):
        sum = sum + pack_data[i]
        i = i+1
    pack_data[lens-1] = sum;
    return pack_data

#线检测数据打包
def LineDataPack(flag,angle,distance,crossflag,crossx,crossy,T_ms):
    if (flag == 0):
        print("found: angle",angle,"  distance=",distance,"   线状态   未检测到直线")
    elif (flag == 1):
        print("found: angle",angle,"  distance=",distance,"   线状态   直线")
    elif (flag == 2):
        print("found: angle",angle,"  distance=",distance,"   线状态   左转")
    elif (flag == 3):
        print("found: angle",angle,"  distance=",distance,"   线状态   右转")

    line_data=bytearray([0xAA,0x29,0x05,0x42,0x00,flag,angle>>8,angle,distance>>8,distance,crossflag,crossx>>8,crossx,(-crossy)>>8,(-crossy),T_ms,0x00])
    lens = len(line_data)#数据包大小
    line_data[4] = 11;#有效数据个数
    i = 0
    sum = 0
    #和校验
    while i<(lens-1):
        sum = sum + line_data[i]
        i = i+1
    line_data[lens-1] = sum;
    return line_data

#AprilTag检测数据打包
def AprilTagDataPack(flag,crossx,crossy):
    AprilTag_data=bytearray([0xAA,0x29,0x05,0x43,0x00,flag,crossx>>8,crossx,(-crossy)>>8,(-crossy),0x00])
    lens = len(AprilTag_data)#数据包大小
    AprilTag_data[4] = 5;#有效数据个数
    i = 0
    sum = 0
    #和校验
    while i<(lens-1):
        sum = sum + AprilTag_data[i]
        i = i+1
    AprilTag_data[lens-1] = sum;
    return AprilTag_data

#用户数据打包
def UserDataPack(data0,data1,data2,data3,data4,data5,data6,data7,data8,data9):
    UserData=bytearray([0xAA,0x05,0xAF,0xF1,0x00
                        ,data0,data1,data2>>8,data2,data3>>8,data3
                        ,data4>>24,data4>>16,data4>>8,data4
                        ,data5>>24,data5>>16,data5>>8,data5
                        ,data6>>24,data6>>16,data6>>8,data6
                        ,data7>>24,data7>>16,data7>>8,data7
                        ,data8>>24,data8>>16,data8>>8,data8
                        ,data9>>24,data9>>16,data9>>8,data9
                        ,0x00])
    lens = len(UserData)#数据包大小
    UserData[4] = lens-6;#有效数据个数
    i = 0
    sum = 0
    #和校验
    while i<(lens-1):
        sum = sum + UserData[i]
        i = i+1
    UserData[lens-1] = sum;
    return UserData

#************************************ (C) COPYRIGHT 2019 ANO ***********************************#

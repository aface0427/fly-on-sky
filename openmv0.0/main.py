#************************************ (C) COPYRIGHT 2019 ANO ***********************************#
import sensor, image, time, math, struct
import json
from pyb import LED,Timer
from struct import pack, unpack
import Message,LineFollowing,DotFollowing,find_apriltags01,molcircle,pole,pole2

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

'''4
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # use RGB565.
sensor.set_framesize(sensor.VGA) # use QQVGA for speed.
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_windowing(300,70)
sensor.skip_frames(10) # Let new settings take affect.
sensor.set_auto_whitebal(False) # turn this off.
sensor.set_auto_gain(True)
clock = time.clock() # Tracks FPS.
Message.Ctr.WorkMode=4 #测试红色杆检测用
'''

Message.Ctr.WorkMode = 5
rp=[30, 70, 45, 82, -15, 38]
blue_threshold   = [30, 70, 45, 82, -15, 38]
White=(0,0,0,0,0,0)
T = 0
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # use RGB565.
sensor.set_framesize(sensor.VGA) # use QQVGA for speed.
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_windowing(300,70)
rr=[0,0,300,70]
sensor.skip_frames(10) # Let new settings take affect.
sensor.set_auto_whitebal(False) # turn this off.
sensor.set_auto_gain(True)
clock = time.clock() # Tracks FPS.
llength=0
K=60*44#the value should be measured
r = [(320//2)-(50//2), (240//2)-(50//2), 50, 50]
lx=0
ly=0
lw=0
lh=0
ll1=0
ll2=0
aa1=0
aa2=0
bb1=0
bb2=0
#主循环
while(True):
    clock.tick()#时钟初始化
    flag = 0
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
    elif Message.Ctr.WorkMode==5:#红色杆
        if(T>30):
            blue_threshold[0]=rp[0]
            blue_threshold[1]=rp[1]
            blue_threshold[2]=rp[2]
            blue_threshold[3]=rp[3]
            blue_threshold[4]=rp[4]
            blue_threshold[5]=rp[5]
            lx=0
            ly=0
            lw=0
            lh=0
            ll1=0
            ll2=0
            aa1=0
            aa2=0
            bb1=0
            bb2=0
        clock.tick() # Track elapsed milliseconds between snapshots().
        img = sensor.snapshot().lens_corr(1.8) # Take a picture and return the image.
        img.mean(5)
        blobs = img.find_blobs([blue_threshold],roi=rr,merge=True,x_margin=5)
        p=0
        T+=1
        if(blobs):
            for b in blobs:
                if(b.w()<20 or b.h()<50):
                    continue
                elif((lx>0 or ly >0 or lw>0 or lh>0)and(abs(lx-b.x())>30 or abs(ly-b.y()>5) or abs(lw-b.w())>10 or abs(lh-b.h())>10)):
                    continue
                r=[b.x(),b.y(),b.w(),b.h()]
                rr=[max(0,b.x()-15),max(0,b.y()-10),min(b.w()+30,300-b.x()+15),min(b.h()+40,70-b.y()+10)]
                hist = img.get_histogram(roi=r)
                lo = hist.get_percentile(0.01) # 获取1％范围的直方图的CDF（根据需要调整）！
                hi = hist.get_percentile(0.99) # 获取99％范围的直方图的CDF（根据需要调整）！# 平均百分位值。
                st=img.get_statistics(roi=r)
                if((lx==0 and ly==0 and lw==0 and lh==0) or (abs(lx-b.x())<=30 and abs(ly-b.y()<=5) and abs(lw-b.w())<=10 and abs(lh-b.h())<=10)):
                    if((ll1==0 and ll2==0 and aa1==0 and aa2==0 and bb1==0 and bb2==0)or(abs(ll1-st.l_mean())<=5 and abs(ll2-st.l_mode())<=10 and abs(aa1-st.a_mean())<=5 and abs(aa2-st.a_mode())<=10 and abs(bb1-st.b_mean())<=5 and abs(bb2-st.b_mode())<=10)):
                        lx=b.x()
                        ly=b.y()
                        lw=b.w()
                        lh=b.h()
                        ll1=st.l_mean()
                        ll2=st.l_mode()
                        aa1=st.a_mean()
                        aa2=st.a_mode()
                        bb1=st.b_mean()
                        bb2=st.b_mode()
                l1 = (blue_threshold[0] + lo.l_value()) // 2
                l2 = (blue_threshold[1] + hi.l_value()) // 2
                l3=blue_threshold[2]
                l4=blue_threshold[3]
                l5=blue_threshold[4]
                l6=blue_threshold[5]
                if(l2-l1<=40 and abs(l1-blue_threshold[0])<=4 and abs(l2-blue_threshold[1])<=4 and abs(l3-blue_threshold[2])<=4 and abs(l4-blue_threshold[3])<=4 and abs(l5-blue_threshold[4])<=4 and abs(l6-blue_threshold[5])<=4):
                    blue_threshold[0]=l1
                    blue_threshold[1]=l2
                    blue_threshold[2]=l3
                    blue_threshold[3]=l4
                    blue_threshold[4]=l5
                    blue_threshold[5]=l6
                if(llength==0 or abs(llength-length<=3)):
                    img.draw_rectangle(b[0:4]) # rect
                    img.draw_cross(b[5], b[6]) # cx, cy
                    Lm = b.w()
                    length = K/Lm
                    print(length,b.x()+b.w()/2)
                    LED(2).on()
                    LED(3).on()
                    Message.UartSendData(Message.PoleDataPack(0,int(length),int(b.x()+b.w()/2)))
                    flag = 1
                    llength=length
                    T=0
                else:
                    continue
    if flag == 0 :
        LED(2).off()
        LED(3).off()
        Message.UartSendData(Message.PoleDataPack(1,0,0))
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


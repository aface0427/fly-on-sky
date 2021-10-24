# Measure the distance
#
# This example shows off how to measure the distance through the size in imgage
# This example in particular looks for yellow pingpong ball.

import sensor, image, time
from pyb import LED
import Message
llength=0
length=0
T = 0
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
rp=(30, 70, 45, 82, -15, 38)
blue_threshold   = [30, 70, 45, 82, -15, 38]
White=(0,0,0,0,0,0)
rr=[0,0,300,70]
K=60*44#the value should be measured
r = [(320//2)-(50//2), (240//2)-(50//2), 50, 50]


def findingpole2():
    global T,lx,ly,lw,lh,ll1,ll2,aa1,aa2,bb1,bb2,rp,blue_threshold,White,rr,K,r,llength,flag,length
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


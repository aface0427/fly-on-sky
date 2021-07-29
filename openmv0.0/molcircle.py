# 边缘检测例子:
#
# 这个程序示范了在图像上使用morph函数来进行边缘检测。
# 然后在进行阈值和滤波
from pyb import LED
import sensor, image, time
import Message

#设置核函数滤波，核内每个数值值域为[-128,127],核需为列表或元组
kernel_size = 1 # kernel width = (size*2)+1, kernel height = (size*2)+1
kernel = [-1, -1, -1,\
          -1, +8, -1,\
          -1, -1, -1]
# 这个一个高通滤波器。见这里有更多的kernel
# http://www.fmwconcepts.com/imagemagick/digital_image_filtering.pdf
thresholds = [(100, 255)] # grayscale thresholds设置阈值

#sensor.reset() # 初始化 sensor.
#初始化摄像头

#sensor.set_pixformat(sensor.GRAYSCALE) # or sensor.RGB565
#设置图像色彩格式，有RGB565色彩图和GRAYSCALE灰度图两种

#sensor.set_framesize(sensor.QQVGA) # or sensor.QVGA (or others)
#sensor.set_windowing((120,120))
#设置图像像素大小
#sensor.skip_frames(10) # 让新的设置生效
#clock = time.clock() # 跟踪FPS帧率

# 在OV7725 sensor上, 边缘检测可以通过设置sharpness/edge寄存器来增强。
# 注意:这将在以后作为一个函数实现
#if (sensor.get_id() == sensor.OV7725):
    #sensor.__write_reg(0xAC, 0xDF)
    #sensor.__write_reg(0x8F, 0xFF)
def cmp(x):
    return x[0]+x[1]


def Molcircle():
    img = sensor.snapshot() # 拍一张照片，返回图像
    img.morph(kernel_size,kernel,mul=1.9)
    #morph(size, kernel, mul=Auto, add=0)，morph变换，mul根据图像对比度
    #进行调整，mul使图像每个像素乘mul；add根据明暗度调整，使得每个像素值加上add值。
    #如果不设置则不对morph变换后的图像进行处理。
    img.binary(thresholds)
    #利用binary函数对图像进行分割

    # Erode pixels with less than 2 neighbors using a 3x3 image kernel
    # 腐蚀像素小于2邻居使用3x3图像内核
    img.erode(1, threshold = 2)
    #侵蚀函数erode(size, threshold=Auto)，去除边缘相邻处多余的点。threshold
    #用来设置去除相邻点的个数，threshold数值越大，被侵蚀掉的边缘点越多，边缘旁边
    #白色杂点少；数值越小，被侵蚀掉的边缘点越少，边缘旁边的白色杂点越多。
    p=0
    a=[]
    a.append((0,0))
    for c in img.find_circles(threshold = 4800, x_margin = 2, y_margin = 2, r_margin = 2,r_min = 20, r_max = 100, r_step = 2):
        p=p+1
        a.append((c.x(),c.y()))
        img.draw_circle(c.x(), c.y(), c.r(), color = (255, 0, 0))
        #print(c)
    b = sorted(a, key=cmp)
    if(p<10):
        print("%f,%f" %(0,0))
        Message.UartSendData(Message.DotDataPack(1,0,0))
        LED(2).off()
    else:
        q1=0
        q2=0
        for i in range(5,p-5):
            q1=q1+b[i][0]
            q2=q2+b[i][1]
        q1=int(q1/(p-9))
        q2=int(q2/(p-9))
        print("%f,%f" %(q1,q2))
        Message.UartSendData(Message.DotDataPack(0,q1,q2))
        LED(2).on()



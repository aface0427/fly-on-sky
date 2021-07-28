# AprilTags Example
#
# This example shows the power of the OpenMV Cam to detect April Tags
# on the OpenMV Cam M7. The M4 versions cannot detect April Tags.

from pyb import LED,Timer
import sensor, image, time, math, struct
import Message


# f_x 是x的像素为单位的焦距。对于标准的OpenMV，应该等于2.8/3.984*656，这个值是用毫米为单位的焦距除以x方向的感光元件的长度，乘以x方向的感光元件的像素（OV7725）
# f_y 是y的像素为单位的焦距。对于标准的OpenMV，应该等于2.8/2.952*488，这个值是用毫米为单位的焦距除以y方向的感光元件的长度，乘以y方向的感光元件的像素（OV7725）

# c_x 是图像的x中心位置
# c_y 是图像的y中心位置

f_x = (2.8 / 3.984) * 160 # 默认值
f_y = (2.8 / 2.952) * 120 # 默认值
c_x = 160 * 0.5 # 默认值(image.w * 0.5)
c_y = 120 * 0.5 # 默认值(image.h * 0.5)

def degrees(radians):
    return (180 * radians) / math.pi

def Find_Apriltags() :
    img = sensor.snapshot().lens_corr(strength=1.65, zoom=1)
    tags = img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y)
    if len(tags)==1:
        for tag in tags : # 默认为TAG36H11
            img.draw_rectangle(tag.rect(), color = (255, 0, 0))
            img.draw_cross(tag.cx(), tag.cy(), color = (0, 255, 0))
            print("%f %f",tag.cx(),tag.cy())
            Message.UartSendData(Message.AprilTagDataPack(0,tag.cx(),tag.cy()))
    else:
        Message.UartSendData(Message.AprilTagDataPack(1,1,1))

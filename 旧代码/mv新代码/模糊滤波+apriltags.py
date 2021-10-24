# AprilTags Example
#
# This example shows the power of the OpenMV Cam to detect April Tags
# on the OpenMV Cam M7. The M4 versions cannot detect April Tags.

import sensor, image, time, math

sensor.reset()
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_transpose(False)
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.HQVGA) # we run out of memory if the resolution is much bigger...
sensor.set_windowing((160,160))
sensor.skip_frames(10)
sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...
clock = time.clock()
# f_x 是x的像素为单位的焦距。对于标准的OpenMV，应该等于2.8/3.984*656，这个值是用毫米为单位的焦距除以x方向的感光元件的长度，乘以x方向的感光元件的像素（OV7725）
# f_y 是y的像素为单位的焦距。对于标准的OpenMV，应该等于2.8/2.952*488，这个值是用毫米为单位的焦距除以y方向的感光元件的长度，乘以y方向的感光元件的像素（OV7725）

# c_x 是图像的x中心位置
# c_y 是图像的y中心位置

f_x = (2.8 / 3.984) * 240 # 默认值
f_y = (2.8 / 2.952) * 160 # 默认值
c_x = 160 * 0.5 # 默认值(image.w * 0.5)
c_y = 160 * 0.5 # 默认值(image.h * 0.5)

def degrees(radians):
    return (180 * radians) / math.pi

while(True):
    clock.tick()
    img = sensor.snapshot()
    img.gaussian(1)
    for tag in img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y): # 默认为TAG36H11
        img.draw_rectangle(tag.rect(), color = (255, 0, 0))
        img.draw_cross(tag.cx(), tag.cy(), color = (0, 255, 0))
        print_args = (tag.x_translation(), tag.y_translation(), tag.z_translation(), \
            degrees(tag.x_rotation()), degrees(tag.y_rotation()), degrees(tag.z_rotation()))
        # 位置的单位是未知的，旋转的单位是角度
        # *4 cm
        tx=4*tag.x_translation()
        ty=4*tag.x_translation()
        print(tx,ty)
    print(clock.fps())

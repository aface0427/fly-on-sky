# Measure the distance
#
# This example shows off how to measure the distance through the size in imgage
# This example in particular looks for yellow pingpong ball.

import sensor, image, time
from pyb import LED
# For color tracking to work really well you should ideally be in a very, very,
# very, controlled enviroment where the lighting is constant...
blue_threshold   = (29, 68, 22, 89, -5, 37)
# You may need to tweak the above settings for tracking green things...
# Select an area in the Framebuffer to copy the color settings.
White=(0,0,0,0,0,0)
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
llength=0
K=60*44#the value should be measured
T=0
while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot().lens_corr(1.8) # Take a picture and return the image.
    img.mean(3)
    blobs = img.find_blobs([blue_threshold],merge=True,x_margin=5)
    p=0
    if(blobs):
        for b in blobs:
            if(b.w()<36 or b.h()<50):
                continue
            p+=1
            img.draw_rectangle(b[0:4]) # rect
            img.draw_cross(b[5], b[6]) # cx, cy
            Lm = b.w()
            length = K/Lm
            if(llength==0 or abs(llength-length<=3)):
                print(length,b.x()+b.w()/2)
                llength=length
            else:
                continue
            #if(length>100):
                #print(b.h(),b.w())
            #print(length,b.x()+b.w()/2)

    #print(clock.fps()) # Note: Your OpenMV Cam runs about half as fast while
    # connected to your computer. The FPS should increase once disconnected.

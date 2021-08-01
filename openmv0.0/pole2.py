# Measure the distance
#
# This example shows off how to measure the distance through the size in imgage
# This example in particular looks for yellow pingpong ball.

import sensor, image, time
from pyb import LED
import Message


# For color tracking to work really well you should ideally be in a very, very,
# very, controlled enviroment where the lighting is constant...

# You may need to tweak the above settings for tracking green things...
# Select an area in the Framebuffer to copy the color settings.




global r
r = [(320//2)-(50//2), (240//2)-(50//2), 50, 50]



    #print(clock.fps()) # Note: Your OpenMV Cam runs about half as fast while
    # connected to your computer. The FPS should increase once disconnected.

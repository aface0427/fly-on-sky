# 线段检测例程
#
# 这个例子展示了如何在图像中查找线段。对于在图像中找到的每个线对象，
# 都会返回一个包含线条旋转的线对象。

# find_line_segments()找到有限长度的线（但是很慢）。
# Use find_line_segments()找到非无限的线（而且速度很快）。

enable_lens_corr = 1 # turn on for straighter lines...打开以获得更直的线条…

import sensor, image, time

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE) # 灰度更快
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
clock = time.clock()
min_degree=-5
max_degree=5
# 所有线段都有 `x1()`, `y1()`, `x2()`, and `y2()` 方法来获得他们的终点
# 一个 `line()` 方法来获得所有上述的四个元组值，可用于 `draw_line()`.
T=0
mx=0
mn=9999
lmx=70
lmn=90
def maxx(a,b):
    if(a>b):
        return a
    else:
        return b
def minn(a,b):
    if(a<b):
        return a
    else:
        return b
while(True):
    clock.tick()
    img = sensor.snapshot()
    img.laplacian(1,sharpen=True)
    if enable_lens_corr: img.lens_corr(1.8) # for 2.8mm lens...

    # `merge_distance`控制附近行的合并。 在0（默认），没有合并。
    # 在1处，任何距离另一条线一个像素点的线都被合并...等等，
    # 因为你增加了这个值。 您可能希望合并线段，因为线段检测会产生大量
    # 的线段结果。

    # `max_theta_diff` 控制要合并的任何两线段之间的最大旋转差异量。
    # 默认设置允许15度。
    p=0
    a = [[0 for i in range(30)] for j in range(17)]
    cnt=[]
    for i in range(0,16):
        cnt.append(0)
    for l in img.find_line_segments(roi=(0,50,160,20),merge_distance = 0, max_theta_diff = 5):
        if (min_degree <= l.theta()) and (l.theta() <= max_degree):
            #print(l.x1())
            cnt[int(l.x1()/10)]=cnt[int(l.x1()/10)]+1
            a[int(l.x1()/10)][cnt[int(l.x1()/10)]]=l.x1()
            mx=maxx(mx,l.x1())
            mn=minn(mn,l.x1())
            img.draw_line(l.line(), color = (255, 0, 0))
        # print(l)
    mxx=0
    mxnum=0
    for i in range(0,15):
        if(mxnum<cnt[i]):
            mxnum=cnt[i]
            mxx=i
    print(mxnum)
    if(mxnum==0):
        continue
    ave=0
    for i in range(1,cnt[mxx]):
        ave=ave+a[mxx][i]
    ave=ave/cnt[mxx]
    #ave应在80处，lmx-lmn应等于15左右
    print(ave,lmn,lmx)
    if(T%3==0):
        lmx=mx
        lmn=mn
        mx=0
        mn=9999
    print("FPS %f" % clock.fps())

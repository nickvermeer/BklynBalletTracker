#!/usr/bin/python
import numpy as np
import math
import cv2

size=(800,600)
pts=[]
#pts.append([(5,237),(37,203),(69,175),(120,122),(183,61),(228,16)])  #cam1 day2
#pts.append([(513,8),(546,27),(599,60),(676,109),(749,154)])  #cam2 day2
pts.append([(201,529),(242,481),(296,412),(369,325),(439,239),(525,133)])
pts.append([(625,500),(568,449),(482,375),(392,297),(300,218),(161,100)])
H=[]

for idx in range(0,2):
    angles=[]
    x_prev=-1
    y_prev=-1
    if (idx==0):
        direction=1.0
    else:
        direction=-1.0
        
    for x,y in pts[idx]:
        if (x_prev != -1 and y_prev != -1):
            dx=x_prev-x
            dy=y_prev-y
            angles.append(math.atan(abs(float(dy)/float(dx)))*direction)
        x_prev=x
        y_prev=y

    for angle in angles:    
        print "estimated angle = {}".format(angle)

    avg_angle=sum(angles)/float(len(angles))
    print "avg angle = {}".format(avg_angle)

    

    sin_angle=math.sin(avg_angle)
    cos_angle=math.cos(avg_angle)

    H.append(np.array([[cos_angle, -1.0*sin_angle, 0.0],
                            [sin_angle, cos_angle,      0.0],
                            [0.0,       0.0,              1]],np.double))

    rotx_pts=[0]
    roty_pts=[0]

    x,y,z=H[idx].dot([size[0],0,1])
    rotx_pts.append(x)
    roty_pts.append(y)

    x,y,z=H[idx].dot([0,size[1],1])
    rotx_pts.append(x)
    roty_pts.append(y)

    x,y,z=H[idx].dot([size[0],size[1],1])
    rotx_pts.append(x)
    roty_pts.append(y)

    X_T=abs(min(rotx_pts))
    Y_T=abs(min(roty_pts))
    H[idx][0,2]=X_T
    H[idx][1,2]=Y_T
    print
    print "translation of X={}".format(X_T)
    print "translation of Y={}".format(Y_T)
    print H[idx]

#perspective_pts_cam1=np.array([(264,309),(152,419),(276,671),(389,404)],np.float32)
#perspective_pts_cam2=np.array([(799,304),(664,396),(727,675),(920,429)],np.float32)
perspective_pts_cam1=np.array([(276,289),(152,388),(249,650),(389,397)],np.float32)
perspective_pts_cam2=np.array([(768,282),(651,396),(760,657),(908,386)],np.float32)

perspective_tf=cv2.getPerspectiveTransform(perspective_pts_cam2,perspective_pts_cam1)

print
print perspective_tf
print
H[1]=perspective_tf.dot(H[1])
print
print
print
for idx in range(0,2):
    for H_row in range(0,3):
        for H_col in range(0,3):
            print "H{0}.at<double>({1:d},{2:d})={3:8.8f};".format(idx+1,H_row,H_col,H[idx][H_row,H_col])


            

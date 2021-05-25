# -*- coding: utf-8 -*-
'''
@Time    : 2021/3/7 17:34
@Author  : Junfei Sun
@Email   : sjf2002@sohu.com
@File    : limelight.py
'''

import cv2
from time import sleep
import numpy
from networktables import NetworkTables
import threading
from networktables import NetworkTablesInstance
#import cscore as cs
import math
import logging
import re

logging.basicConfig(level = logging.DEBUG)

NetworkTables.initialize(server='roborio-7280-frc.local')
table=NetworkTables.getTable("limelight")
table_c=NetworkTables.getTable("SmartDashboard")

cam_url='http://10.72.80.11:5800'
ha=2.320
hc=0.70859
radius=0.31439
ac=table_c.getNumber("cameraAngle")
aconst=0
Kp=1
thresh=0
kpd=1
desired_distance=1
#vi=table_c.getNumber("initialSpeed")


#instance=NetworkTablesInstance.getDefault()

def distance_estimation(radius,ac,ha,hc,table):
    '''
    :param ac: the angle between the crosshair of the camera and the horizonal surface
    :param ha: height of the aim
    :param hc: height of the camera crosshair
    :param table: networktable
    :return: distance
    '''
    ty = float(table.getNumber('ty',0))
    distance=(ha-hc*(1+radius*math.sin(ac)))/math.tan(-ty*2*math.pi/360+ac)
    return distance

def Aiming(table,aconst,Kp,thresh):
    '''
    :param table: networktable
    :param aconst: the minimum command constant that help the machine to turn at small error angles
    :param Kp: the proportional control constant (should be a negative float)
    :param thresh: the threshold deciding when the aconst is going to be added to the calculation
    :return: adjustamount
    '''
    tx=float(table.getNumber('tx',0))
    if tx>=thresh:
        a_adjustment=Kp*tx-aconst
    elif tx<thresh:
        a_adjustment=Kp*tx+aconst
    return a_adjustment

def ranging(table,ac,ha,hc,kpd,desired_distance,radius):
    '''
        :param ac: the angle between the crosshair of the camera and the horizonal surface
        :param ha: height of the aim
        :param hc: height of the camera crosshair
        :param table: networktable
        :param kpd: the proportional control constant for distance
        :param desired_distance: the desired distance
        :return: distance
    '''
    current_distance=distance_estimation(radius,ac,ha,hc,table)
    distance_error=current_distance-desired_distance
    d_adjustment=kpd*distance_error
    return d_adjustment

'''
def tossing(distance,ha):
    # if you want to use parabola
    angle, t = symbols('angle t')
    f1 = vi * cos(angle) * t - distance
    f2 = vi * sin(angle) * t - g * t * t / 2 + ha
    result = nonlinsolve([f1, f2], [angle, t])
    result = re.search('2\*_n\*pi \+ \d\.\d*', str(result)).group()
    result = re.split('[+]', result)
    toss_adjustment = float(result[1]) * 360 / (2 * math.pi) 
    print(toss_adjustment)
    return toss_adjustment
'''

capture = cv2.VideoCapture(cam_url)
while True:
    ret,cap=capture.read()
    cv2.imshow('limelight',cap)

    #table.putNumber('tx', tx.getDouble(0))
    #table.putNumber('ty', ty.getDouble(0))
    #table.putNumber('ta', ta.getDouble(0))
    #table.putNumber('ts', ts.getDouble(0))
    #print([tx,ty,ta,ts])
    txg = table.getNumber('tx',None)
    tyg = table.getNumber('ty',None)
    tag = table.getNumber('ta',None)
    tsg = table.getNumber('ts',None)
    print(NetworkTables.isConnected())
    print([txg,tyg,tag,tsg])
    if txg==None or tyg==None or tag==None or tsg==None:
        table_c.putNumber('d_adjustment', None)
        table_c.putNumber('a_adjustment', None)
        table_c.putNumber('toss_adjustment', None)
    else:
        distance=distance_estimation(radius,ac, ha, hc, table)
        d_adjustment=ranging(table,ac,ha,hc,kpd,desired_distance,radius)
        a_adjustment=Aiming(table,aconst,Kp,thresh)
        toss_adjustment=tyg
        print('distance:', distance_estimation(radius,ac, ha, hc, table))
        print('ranging:', ranging(table,ac,ha,hc,kpd,desired_distance,radius))
        print('Aiming:', Aiming(table,aconst,Kp,thresh))
        table_c.putNumber('distance',distance)
        table_c.putNumber('a_adjustment',a_adjustment)
        table_c.putNumber('toss_adjustment',toss_adjustment)

    if cv2.waitKey(1)==27:
        break
capture.release()

cv2.destroyAllWindows()






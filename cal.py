#!/bin/env python3

import serial
s = serial.Serial('/dev/ttyUSB2', baudrate=31250)

# valves, arms, fan motors
def w( v1,v2,v3,v4, a1,a2,a3,a4, f1,f2,f3,f4):
  msg = [0xFF, 0xAA,
    v1>>8,v1&0xFF, v2>>8,v2&0xFF, v3>>8,v3&0xFF, v4>>8,v4&0xFF,
    a1,a2,a3,a4,
    f1>>8,f1&0xFF, f2>>8,f2&0xFF, f3>>8,f3&0xFF, f4>>8,f4&0xFF ]

  s.write(bytearray( msg + [~sum(msg)&0xFF] ))

vopen = [1250,1200,1310,1150]
vclosed = [1150,1100,1220,1050]

def setspeed(whistle, angle):
  if whistle==1:
    return int( 650 + ((angle/250)**2)*600 )
  if whistle==2:
    return int( 640 + ((angle/250)**1.5)*450 )
  if whistle==3:
    return int( 640 + ((angle/250)**1.5)*450 ) # not measured properly yet
  if whistle==4:
    return int( 610 + ((angle/250)**1.8)*550 )


def setpos(whistle, angle, speed=-1):
  if speed==-1:
    speed = setspeed(whistle, angle)

  if whistle==1:
    w( vopen[0],vclosed[1],vclosed[2],vclosed[3], angle,45,45,45, speed,0,0,0)
  elif whistle==2:
    w( vclosed[0],vopen[1],vclosed[2],vclosed[3], 45,angle,45,45, 0,speed,0,0)
  elif whistle==3:
    w( vclosed[0],vclosed[1],vopen[2],vclosed[3], 45,45,angle,45, 0,0,speed,0)
  elif whistle==4:
    w( vclosed[0],vclosed[1],vclosed[2],vopen[3], 45,45,45,angle, 0,0,0,speed)

def off():
  w( 1150,1100,1220,1050,  45,45,45,45, 0,0,0,0)

if __name__ == "__main__":
  w( 0,0,0,0,  90,90,90,90, 0,0,0,0)
  #w(0,1150,0,0,145,145,145,145,0,0,0,1100)

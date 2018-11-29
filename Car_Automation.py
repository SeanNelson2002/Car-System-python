import math
import IMU
import datetime
import time
import RPi.GPIO as GPIO
import board
import busio
import adafruit_vl6180x
"""Add Comments EVERYWHERE"""
IMU_UPSIDE_DOWN = 0
RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070
AA =  0.40
magXmin =  0
magYmin =  0
magZmin =  0
magXmax =  0
magYmax =  0
magZmax =  0
Q_angle = 0.02
Q_gyro = 0.0015
R_angle = 0.005
y_bias = 0.0
x_bias = 0.0
XP_00 = 0.0
XP_01 = 0.0
XP_10 = 0.0
XP_11 = 0.0
YP_00 = 0.0
YP_01 = 0.0
YP_10 = 0.0
YP_11 = 0.0
KFangleX = 0.0
KFangleY = 0.0
class Gyroscope():
    def kalmanFilterY ( accAngle, gyroRate, DT):
        y=0.0
        S=0.0
        global KFangleY
        global Q_angle
        global Q_gyro
        global y_bias
        global YP_00
        global YP_01
        global YP_10
        global YP_11
        KFangleY = KFangleY + DT * (gyroRate - y_bias)
        YP_00 = YP_00 + ( - DT * (YP_10 + YP_01) + Q_angle * DT )
        YP_01 = YP_01 + ( - DT * YP_11 )
        YP_10 = YP_10 + ( - DT * YP_11 )
        YP_11 = YP_11 + ( + Q_gyro * DT )
        y = accAngle - KFangleY
        S = YP_00 + R_angle
        K_0 = YP_00 / S
        K_1 = YP_10 / S
        KFangleY = KFangleY + ( K_0 * y )
        y_bias = y_bias + ( K_1 * y )
        YP_00 = YP_00 - ( K_0 * YP_00 )
        YP_01 = YP_01 - ( K_0 * YP_01 )
        YP_10 = YP_10 - ( K_1 * YP_00 )
        YP_11 = YP_11 - ( K_1 * YP_01 )
        return KFangleY
    def kalmanFilterX ( accAngle, gyroRate, DT):
        x=0.0
        S=0.0
        global KFangleX
        global Q_angle
        global Q_gyro
        global x_bias
        global XP_00
        global XP_01
        global XP_10
        global XP_11
        KFangleX = KFangleX + DT * (gyroRate - x_bias)
        XP_00 = XP_00 + ( - DT * (XP_10 + XP_01) + Q_angle * DT )
        XP_01 = XP_01 + ( - DT * XP_11 )
        XP_10 = XP_10 + ( - DT * XP_11 )
        XP_11 = XP_11 + ( + Q_gyro * DT )
        x = accAngle - KFangleX
        S = XP_00 + R_angle
        K_0 = XP_00 / S
        K_1 = XP_10 / S
        KFangleX = KFangleX + ( K_0 * x )
        x_bias = x_bias + ( K_1 * x )
        XP_00 = XP_00 - ( K_0 * XP_00 )
        XP_01 = XP_01 - ( K_0 * XP_01 )
        XP_10 = XP_10 - ( K_1 * XP_00 )
        XP_11 = XP_11 - ( K_1 * XP_01 )
        return KFangleX
    def GYRO(activated):
        if activated==True:
            a = datetime.datetime.now()
            IMU.detectIMU()
            IMU.initIMU()
            gyroXangle = 0.0
            gyroYangle = 0.0
            gyroZangle = 0.0
            CFangleX = 0.0
            CFangleY = 0.0
            kalmanX = 0.0
            kalmanY = 0.0
            ACCx = IMU.readACCx()
            ACCy = IMU.readACCy()
            ACCz = IMU.readACCz()
            GYRx = IMU.readGYRx()
            GYRy = IMU.readGYRy()
            GYRz = IMU.readGYRz()
            MAGx = IMU.readMAGx()
            MAGy = IMU.readMAGy()
            MAGz = IMU.readMAGz()   
            MAGx -= (magXmin + magXmax) /2 
            MAGy -= (magYmin + magYmax) /2 
            MAGz -= (magZmin + magZmax) /2 
            b = datetime.datetime.now() - a
            a = datetime.datetime.now()
            LP = b.microseconds/(1000000*1.0)
            rate_gyr_x =  GYRx * G_GAIN
            rate_gyr_y =  GYRy * G_GAIN
            rate_gyr_z =  GYRz * G_GAIN
            gyroXangle+=rate_gyr_x*LP
            gyroYangle+=rate_gyr_y*LP
            gyroZangle+=rate_gyr_z*LP
            if not IMU_UPSIDE_DOWN:
                AccXangle =  (math.atan2(ACCy,ACCz)*RAD_TO_DEG)
                AccYangle =  (math.atan2(ACCz,ACCx)+M_PI)*RAD_TO_DEG
            else:
                AccXangle =  (math.atan2(-ACCy,-ACCz)*RAD_TO_DEG)
                AccYangle =  (math.atan2(-ACCz,-ACCx)+M_PI)*RAD_TO_DEG
            if AccYangle > 90:
                AccYangle -= 270.0
            else:
                AccYangle += 90.0
            CFangleX=AA*(CFangleX+rate_gyr_x*LP) +(1 - AA) * AccXangle
            CFangleY=AA*(CFangleY+rate_gyr_y*LP) +(1 - AA) * AccYangle
            kalmanY = Gyroscope.kalmanFilterY(AccYangle, rate_gyr_y,LP)
            kalmanX = Gyroscope.kalmanFilterX(AccXangle, rate_gyr_x,LP)
            if IMU_UPSIDE_DOWN:
                MAGy = -MAGy
            heading = 180 * math.atan2(MAGy,MAGx)/M_PI
            if heading < 0:
                heading += 360
            if not IMU_UPSIDE_DOWN:        
                accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
                accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
            else:
                accXnorm = -ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
                accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
            pitch = math.asin(accXnorm)
            roll = -math.asin(accYnorm/math.cos(pitch))
            magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
            if(IMU.LSM9DS0):
                magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)
            else:
                magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)+MAGz*math.sin(roll)*math.cos(pitch) 
            tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/M_PI
            if tiltCompensatedHeading < 0:
                tiltCompensatedHeading += 360
            new_x=kalmanX
            new_y=kalmanY
            Angles=[new_x,new_y]
            return Angles
        else:
            Angles=[0,0]
            return Angles
class Light():
    def Light_Sensor(Activated, PIN):
        if Activated==True:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                GPIO.setup(PIN, GPIO.OUT)
                GPIO.output(PIN,GPIO.HIGH)
                i2c = busio.I2C(board.SCL,board.SDA)
                sensor = adafruit_vl6180x.VL6180X(i2c)
                lux = sensor.read_lux(adafruit_vl6180x.ALS_GAIN_1)
                time.sleep(.1)
                GPIO.output(PIN,GPIO.LOW)
                time.sleep(.1)
                return lux
            except ValueError:
                return 100
    def Front_Lights(Activated):
        lights_Activation=False
        if Activated==True:
            if Light.Light_Sensor(True,6)<25:
                lights_Activation=True
        return lights_Activation
    def Rear_Lights(Activated):
        lights_Activation=False
        if Activated==True:
            if Light.Light_Sensor(True,5)<25:
                lights_Activation=True
        return lights_Activation
    def Front_LEDS(switch, Activated):
        if Activated==True:
            if switch==1:
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                GPIO.setup(22,GPIO.OUT)
                GPIO.output(22,GPIO.HIGH)
            if switch==0:
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                GPIO.setup(22,GPIO.OUT)
                GPIO.output(22,GPIO.LOW)
    def Rear_LEDS(switch, Activated):
        if Activated==True:
            if switch==1:
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                GPIO.setup(23,GPIO.OUT)
                GPIO.output(23,GPIO.HIGH)
            if switch==0:
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                GPIO.setup(23,GPIO.OUT)
                GPIO.output(23,GPIO.LOW)
    def BLINKER(switch,activated):
        if activated==True:
            if switch==1:
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                GPIO.setup(23,GPIO.OUT)
                GPIO.output(23,GPIO.HIGH)
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                GPIO.setup(22,GPIO.OUT)
                GPIO.output(22,GPIO.HIGH)
                time.sleep(.05)
                GPIO.output(23,GPIO.LOW)
                GPIO.output(22,GPIO.LOW)
class Steering():
    def Front_Turn(angle,activated):
        if activated==True:
            print("Steering Front Turn Angle: "+str(angle))
    def Rear_Turn(angle,activated):
        if activated==True:
            print("Steering Rear Turn Angle: "+str(angle))
    def Front_Cast(angle,activated):
        if activated==True:
            print("Steering Front Cast Angle: "+str(angle))
    def Rear_Cast(angle,activated):
        if activated==True:
            print("Steering Rear Cast Angle: "+str(angle))
    def Steering_Wheel(Activated):
        if Activated==True:
            Position=120
            return Position
class Action():
    def Trigger(Activated):
        if Activated==True:
            Amount=50
            return Amount
    def Motor(output, Activated):
        if Activated==True:
            print("Motor Output: "+str(output))
    def Current_Motor_Output(Activated):
        if Activated==True:
            output = 130
            return output
class Obstical_Detection():
    def Front_Right_Sensor(Activated):
        if Activated==True:
            mm = 74.3
            return mm
    def Front_Left_Sensor(Activated):
        if Activated==True:
            mm = 24.5
            return mm
    def Rear_Left_Sensor(Activated):
        if Activated==True:
            mm = 100.3
            return mm
    def Rear_Right_Sensor(Activated):
        if Activated==True:
            mm = 130.2
            return mm
    def Safety_Protocol(Front_Left,Front_Right,Rear_Left,Rear_Right,Activated):
        if Activated==True:
            vehical_stop=False
            if (Front_Left==1) or (Front_Right==1):
                if Action.Current_Motor_Output(False)>0:
                    vehical_stop=True
            if (Rear_Left==1) or (Rear_Right==1):
                if Action.Current_Motor_Output(False)<0:
                    vehical_stop=True
            Action.Motor(0,False)
            print("Safety Protocol Activated: "+str(bool(vehical_stop)))
class Suspension():
    def Front_Left_Sensor(Activated):
        if Activated==True:
            mm = 120
            return mm
    def Front_Right_Sensor(Activated):
        if Activated==True:
            mm = 80
            return mm
    def Rear_Left_Sensor(Activated):
        if Activated==True:
            mm = 50
            return mm
    def Rear_Right_Sensor(Activated):
        if Activated==True:
            mm = 115
            return mm
    def Front_Left_Servo(output,Activated):
        if Activated==True:
            print("Front Left Suspension Servo: "+str(output))
    def Front_Right_Servo(output,Activated):
        if Activated==True:
            print("Front Right Suspension Servo: "+str(output))
    def Rear_Left_Servo(output,Activated):
        if Activated==True:
            print("Rear Left Suspension Servo: "+str(output))
    def Rear_Right_Servo(output,Activated):
        if Activated==True:
            print("Rear Right Suspension Servo: "+str(output))
class Car():
    def Run_System():
        '''                                  Activates Functions                                    '''
        """ Controls 2 wheel or 4 wheel steering with optional wheel cast system                    """
        Car.Steering_System(False)           #Activation Params
        """ Stops car from moving forward or backward if about to hit a cone or fall off the course """
        Car.Obstical_Detection_System(False) #Activation Params
        """ Makes the car move                                                                      """
        Car.Speed_System(False)              #Activation Params
        """ Adjustable suspension system based on distance from ground                              """
        Car.Suspension_System(False)         #Activation Params
    def Steering_System(activated):
        if activated==True:
            Steering.Front_Turn(Steering.Steering_Wheel()/180/70,True)
            Steering.Rear_Turn(-(Steering.Steering_Wheel()/180/70),False)
            Steering.Front_Cast(Steering.Steering_Wheel()/180/30,False)
            Steering.Rear_Cast(-(Steering.Steering_Wheel()/180/30),False)
    def Obstical_Detection_System(activated):
        if activated==True:
            Front_Left=0
            Front_Right=0
            Rear_Left=0
            Rear_Right=0
            if (Obstical_Detection.Front_Right_Sensor(False)<25.4)or(Obstical_Detection.Front_Right_Sensor(False)>25.4*12):
                Front_Right=1
            if (Obstical_Detection.Front_Left_Sensor(False)<25.4)or(Obstical_Detection.Front_Left_Sensor(False)>25.4*12):
                Front_Left=1
            if (Obstical_Detection.Rear_Right_Sensor(False)<25.4)or(Obstical_Detection.Rear_Right_Sensor(False)>25.4*12):
                Rear_Right=1
            if (Obstical_Detection.Rear_Left_Sensor(False)<25.4)or(Obstical_Detection.Rear_Left_Sensor(False)>25.4*12):
                Rear_Left=1
            Obstical_Detection.Safety_Protocol(Front_Left,Front_Right,Rear_Left,Rear_Right,False)
    def Speed_System(activated):
        if activated==True:
            Action.Motor(200*(Action.Trigger()*.01),False)
    def Suspension_System(activated):
        if activated==True:
            Suspension.Front_Left_Servo(Suspension.Front_Left_Sensor(False)*180/152.399918,False)
            Suspension.Front_Right_Servo(Suspension.Front_Right_Sensor(False)*180/152.399918,False)
            Suspension.Rear_Left_Servo(Suspension.Rear_Left_Sensor(False)*180/152.399918,False)
            Suspension.Rear_Right_Servo(Suspension.Rear_Right_Sensor(False)*180/152.399918,False)

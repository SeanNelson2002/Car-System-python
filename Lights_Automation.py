import Car_Automation as car
def Lights_Activation(activated):
        roll=car.Gyroscope.GYRO(True)[0]
        if activated==True:
            if ((roll>25)or(roll<-25)):
                car.Light.BLINKER(1,True)
            else:
                car.Light.BLINKER(0,True)
            if (car.Light.Front_Lights(True)==False):
                car.Light.Front_LEDS(0,True)
            else:
                if car.Light.Front_Lights(True)==True:
                    car.Light.Front_LEDS(1,True)
            if (car.Light.Rear_Lights(True)==False):
                car.Light.Rear_LEDS(0,True)
            else:
                if car.Light.Rear_Lights(True)==True:
                    car.Light.Rear_LEDS(1,True)
while True:
    Lights_Activation(True)

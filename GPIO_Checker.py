import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
INPUT_PINS = [22,23,]
[GPIO.setup(pin, GPIO.IN) for pin in INPUT_PINS]
while True:
    for pin in INPUT_PINS:
        print("pin: "+str(pin)+str(bool(GPIO.input(pin))))
    print("\n")
    time.sleep(.25)

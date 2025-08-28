import time
import shelve
from gpiozero import PWMOutputDevice, DigitalOutputDevice

# ——— PINS & MOTION PARAMS ———
PWM_PIN = 7
DIR_PIN = 1

pwm = PWMOutputDevice(PWM_PIN, frequency=1000)
direction = DigitalOutputDevice(DIR_PIN)
value = 0

def set_pwm(val: float):
    global value
    value = val
    
    
def start():
    direction.off()
    pwm.value = value
    print("Spin")

def stop():
    pwm.value = 0

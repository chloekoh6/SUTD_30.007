import time
import shelve
from gpiozero import PWMOutputDevice, DigitalOutputDevice

# ——— PINS & MOTION PARAMS ———
PWM_PIN = 25
DIR_PIN = 8

pwm = PWMOutputDevice(PWM_PIN, frequency=1000)
direction = DigitalOutputDevice(DIR_PIN)

DB_FILE = '/home/pi/actuator.db'  # path for persistent storage

step = 2
max_pos = 50/step

with shelve.open(DB_FILE) as db:
    db['position'] = 0
    print("Initialized DB with position = 0")
    
def getPosition():
    with shelve.open(DB_FILE) as db:
        return db.get('pos', 0)

def setPosition(pos):
    with shelve.open(DB_FILE, writeback=True) as db:
        db['pos'] = pos

def extend():
    pos = getPosition()
    if pos < max_pos:
        direction.off()
        pwm.value = 1.0
        time.sleep(step)
        pwm.value = 0
        setPosition(pos + step)
        print(f"Current position [saved]: {pos + step}")
    else:
        print("Max position met")

def retract():
    pos = getPosition()
    if pos >= 0:
        direction.on()
        pwm.value = 1.0
        time.sleep(step)
        pwm.value = 0
        setPosition(pos - step)
        print(f"Current position [saved]: {pos + step}")
    else:
        print("Actuator at home")
        
def retract1():
    direction.on()
    pwm.value = 1.0
    time.sleep(step)
    pwm.value = 0

def goToZero():
    pos = getPosition()
    if pos <= 0:
        print("Actuator at home")
        return
    else:
        move = pos*step
        direction.on()       # Set direction
        pwm.value = 1.0      # Full speed
        time.sleep(move)
        pwm.value = 0
        setPosition(0)
    print("Reached physical zero, ")
    print(f"Current position [saved]: 0")        
    
def setZero():
    setPosition(0)
    print("Manual zero set to 0")
    return "Zero set to 0", 200


#!/usr/bin/env python3
import time
import shelve
from gpiozero import PWMOutputDevice, DigitalOutputDevice

# ——— PINS & MOTION PARAMS ———
PUL             = 21    # step pulse
DIR             = 20    # direction
ENA             = 16    # enable (LOW = driver enabled)

MIN_STEP        = 0
MAX_STEP        = 128000

MOVE_INCREMENT  = 3000
PULSE_HIGH_US   = 10
PULSE_PERIOD_US = 50

DB_FILE = '/home/pi/stepper.db'  # path for persistent storage

currentPosition = 0

def init_motor():
    global pul, dir_dev, ena, currentPosition
    # PWMOutputDevice.on() → HIGH, .off() → LOW
    # We'll manually toggle for each microsecond‐precision pulse
    pul = DigitalOutputDevice(PUL, active_high=True, initial_value=False)
    # DigitalOutputDevice.on() → HIGH, .off() → LOW
    dir_dev = DigitalOutputDevice(DIR, active_high=True, initial_value=False)
    # ENA is active‐LOW: so we invert polarity so on() drives LOW
    ena = DigitalOutputDevice(ENA, active_high=False, initial_value=True)
    try:
        loadPosition()
    except:
        currentPosition = 0        
                
def loadPosition():
    global currentPosition
    with shelve.open(DB_FILE) as db:
        currentPosition = db.get('pos', 0)
    print(f"Loaded saved position: {currentPosition}")

def savePosition():
    with shelve.open(DB_FILE, writeback=True) as db:
        db['pos'] = currentPosition

def stepPulse():
    pul.on()
    time.sleep(PULSE_HIGH_US / 10000000.0)
    pul.off()
    time.sleep((PULSE_PERIOD_US - PULSE_HIGH_US) / 10000000.0)

def moveSteps(cw: bool, steps: int):
    global currentPosition
    # cw == True means DIR = LOW, else HIGH
    if cw:
        dir_dev.off()
    else:
        dir_dev.on()
    # tiny settle
    time.sleep(5e-6)

    for _ in range(steps):
        nxt = currentPosition + (1 if cw else -1)
        if not (MIN_STEP <= nxt <= MAX_STEP):
            print("Software limit reached")
            break
        stepPulse()
        currentPosition = nxt

    savePosition()
    print(f"Moved → Position = {currentPosition} (saved)")

def goToZero():
    global currentPosition
    if currentPosition <= 0:
        return
    # CCW toward zero means DIR = HIGH
    dir_dev.on()
    time.sleep(5e-6)
    while currentPosition > 0:
        stepPulse()
        currentPosition -= 1
    savePosition()
    print("Reached physical zero, saved position = 0")

def setZero():
    global currentPosition
    currentPosition = 0
    savePosition()
    print("Manual zero set to 0")
    return "Zero set to 0", 200

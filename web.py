import cv2
import time
import shelve
import threading
from picamera2 import Picamera2
from flask import Flask, Response, render_template, request, jsonify
from gpiozero import PWMOutputDevice, DigitalOutputDevice
import paho.mqtt.client as mqtt
from ultralytics import YOLO

import motor_control
import linear_control
import brush_control

# Start web
app = Flask(__name__)

# Communication setup
MQTT_BROKER = "broker.hivemq.com"
PORT = 1883
TOPIC_COMMAND = "esp32/topic"
TOPIC_STATUS = "rpi/topic"

# Communication(MQTT) Status
last_heartbeat = time.time()
ack_status = None

# Camera setup
ai_enable = False
camera = Picamera2()
camera.configure(camera.create_preview_configuration(main={"format": 'RGB888', "size": (640, 480)}))
#camera.configure(camera.create_preview_configuration(main={"format": "RGB888", "size": (320, 240)}))
camera.start()

model = YOLO('278_best_ncnn_model')

# Motor control for arm
MOVE_INCREMENT  = 3000
motor_control.init_motor()

#Brush Thread set up
DB_FILE = '/home/pi/actuator.db'
brush_s = 0
brush_on = False
slider_value = 0

ai_result = "No Detection"


# Camera frame
def generate_frames():
	global ai_enable
	global brush_on
	global brush_s
	global ai_result
	while True:
		frame = camera.capture_array()
		frame = cv2.rotate(frame, cv2.ROTATE_180)
		
		if ai_enable:
			# Run YOLOv8 inference on the current frame
			results = model.predict(frame, conf=0.15)[0]
			# Get annotated frame
			annotated_frame = results.plot()
			if (get_priority_label(results) == "Heavy"):
				print(get_priority_label(results) + "   brush at 0.75")
				brush_control.set_pwm(0.75)
				brush_s = 75
				brush_on = True
				ai_result = "Hard"
			elif (get_priority_label(results) == "medium"):
				print(get_priority_label(results) + "   brush at 0.5")
				brush_control.set_pwm(0.5)
				brush_s = 50
				brush_on = True
				ai_result = "Medium"
			elif (get_priority_label(results) == "light"):
				print(get_priority_label(results) + "   brush at 0.25")
				brush_control.set_pwm(0.25)
				brush_s = 25
				brush_on = True
				ai_result = "Light"
			else:
				print(get_priority_label(results))
				brush_control.set_pwm(0.00)
				brush_s = 0
				brush_on = False
				ai_result = "No Detection"
			brush_control.set_pwm(brush_s)
			ret, buffer = cv2.imencode('.jpg', annotated_frame)
		else:
			ret, buffer = cv2.imencode('.jpg', frame)
		if not ret:
			continue
		frame_bytes = buffer.tobytes()
		yield (b'--frame\r\n'
			b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
		time.sleep(0.05)
            
def get_priority_label(result):
    priority_order = ["Heavy", "Medium", "Light"]

    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return "No detection"

    cls_ids = boxes.cls.cpu().numpy().astype(int)
    names = result.names  # e.g., {0: 'Heavy', 1: 'Medium', 2: 'Light'}

    # Get detected class names
    detected_labels = {names[int(cls_id)] for cls_id in cls_ids}

    # Return the first one found by priority
    for label in priority_order:
        if label in detected_labels:
            return label

    return "No detection"
          
# MQTT callbacks 
def on_connect(client, userdata, flags, rc):
	print("[MQTT] Connected with rc: ", rc)
	client.subscribe(TOPIC_STATUS)
	
def on_message(client, userdata, msg):
	global last_heartbeat, ack_status
	payload = msg.payload.decode()
	print(f"[MQTT] Recieved on {msg.topic}: {payload}")
	
	if payload == "heartbeat":
		last_heartbeat = time.time()
	elif payload.startswith("ack:"):
		ack_status = payload
		print("ack: " + ack_status)
	
def send_mqtt_command(command):
	client = mqtt.Client("RPIclient")
	client.connect(MQTT_BROKER, PORT, 60)
	client.publish(TOPIC_COMMAND, command)
	client.disconnect()
	
#MQTT thread
def start_mqtt():
	client = mqtt.Client("Rpi")
	client.on_connect = on_connect
	client.on_message = on_message
	client.connect(MQTT_BROKER, PORT, 60)
	client.loop_start()
	return client
	
mqtt_client = start_mqtt()

@app.route('/')
def index():
    return render_template('index.html')  # loads HTML from templates/index.html

@app.route('/video_feed')
def video_feed():
	return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/toggle_ai')
def toggle_ai():
	global ai_enable
	ai_enable = not ai_enable
	print("work")
	return ai_enable, 200
	
@app.route('/up') 
def up():
	global ack_status
	ack_status = None
	try:
		result = mqtt_client.publish(TOPIC_COMMAND, "up")
		print(result.rc)	
		time.sleep(0.1)
	except Exception as e:
		print("error ", e)
	return "Sent", 200
	
@app.route('/down')
def down():
	global ack_status
	ack_status = None
	try:
		result = mqtt_client.publish(TOPIC_COMMAND, "down")
		print(result.rc)	
		time.sleep(0.1)
	except Exception as e:
		print("error ", e)
	return "Sent", 200
	
@app.route('/stop')
def stop():
	global ack_status
	ack_status = None
	try:
		result = mqtt_client.publish(TOPIC_COMMAND, "stop")
		print(result.rc)	
		time.sleep(0.1)
	except Exception as e:
		print("error ", e)
	return "Sent", 200
	
@app.route('/homeROBOT')
def home():
	global ack_status
	ack_status = None
	try:
		result = mqtt_client.publish(TOPIC_COMMAND, "home")
		print(result.rc)	
		time.sleep(0.1)
	except Exception as e:
		print("error ", e)
	return "Sent", 200
	
@app.route('/zero')
def zero():
	global ack_status
	ack_status = None
	try:
		result = mqtt_client.publish(TOPIC_COMMAND, "zero")
		print(result.rc)	
		time.sleep(0.1)
	except Exception as e:
		print("error ", e)
	return "Sent", 200
	
@app.route('/holdup1')
def holdup1():
	global ack_status
	ack_status = None
	try:
		result = mqtt_client.publish(TOPIC_COMMAND, "holdup1")
		print(result.rc)	
		time.sleep(0.1)
	except Exception as e:
		print("error ", e)
	return "Sent", 200
	
@app.route('/holddown1')
def holddown1():
	global ack_status
	ack_status = None
	try:
		result = mqtt_client.publish(TOPIC_COMMAND, "holddown1")
		print(result.rc)	
		time.sleep(0.1)
	except Exception as e:
		print("error ", e)
	return "Sent", 200
	
@app.route('/holdup2')
def holdup2():
	global ack_status
	ack_status = None
	try:
		result = mqtt_client.publish(TOPIC_COMMAND, "holdup2")
		print(result.rc)	
		time.sleep(0.1)
	except Exception as e:
		print("error ", e)
	return "Sent", 200
	
@app.route('/holddown2')
def holddown2():
	global ack_status
	ack_status = None
	try:
		result = mqtt_client.publish(TOPIC_COMMAND, "holddown2")
		print(result.rc)	
		time.sleep(0.1)
	except Exception as e:
		print("error ", e)
	return "Sent", 200

# stepper
@app.route('/arm_up') 
def arm_up():
	motor_control.moveSteps(True, MOVE_INCREMENT)
	print("work")
	return "Sent", 200
	
@app.route('/arm_down')
def arm_down():
	motor_control.moveSteps(False, MOVE_INCREMENT)
	print("work")
	return "Sent", 200
	
@app.route('/homeARM')
def homeARM():
    motor_control_c.goToZero()
    return "Homed", 200

@app.route('/setZero')
def set_zero():
	motor_control.setZero()
	return "Zero set", 200
	
# actuator controls	
@app.route('/setAZero')
def set_a_zero():
	linear_control.setZero()
	return "Zero set", 200
	
@app.route('/extend')
def extend():
	linear_control.extend()
	print("work")
	return "Extended", 200

@app.route('/retract')
def retract():
	linear_control.retract()
	print("work")
	return "Retracted", 200
	
@app.route('/homeACT')
def homeACT():
	linear_control.goToZero()
	print("work")
	return "Home", 200
	
@app.route('/homeACTandARM')
def homeActArm():
	threading.Thread(target=linear_control.goToZero, daemon=True).start() 
	threading.Thread(target=motor_control.goToZero, daemon=True).start() 
	print("work")
	return "Home", 200
	
@app.route('/homeALL')
def homeALL():
	result = mqtt_client.publish(TOPIC_COMMAND, "home")
	print(result.rc)	
	time.sleep(0.1)
	threading.Thread(target=linear_control.goToZero, daemon=True).start() 
	threading.Thread(target=motor_control.goToZero, daemon=True).start() 
	print("work")
	return "Home", 200

@app.route('/stopALL') 
def stopALL():
	linear_control.pwm.value(0)
	brush_control.stop()
	return "Sent", 200
	
@app.route('/get_slider', methods=['POST'])
def get_slider():
    global slider_value
    global brush_on
    global brush_s
    value = request.args.get('value', type=int)

    if value == 25:
        brush_control.set_pwm(0.25)
        brush_on = True
        brush_s = 25
    elif value == 50:
        brush_control.set_pwm(0.50)
        brush_on = True
        brush_s = 50
    elif value == 75:
        brush_control.set_pwm(0.75)
        brush_on = True
        brush_s = 75
    elif value == 100:
        brush_control.set_pwm(1.0)
        brush_on = True
        brush_s = 100
    else:
        brush_control.set_pwm(0.00)
        brush_on = False
        brush_s = 0
           
    brush_control.set_pwm(brush_s) 
    if value is not None:
        slider_value = value
        print("Slider set to:", slider_value)
        return f"Slider value received: {slider_value}", 200

    return "Invalid value", 400
    
@app.route("/set_slider", methods=["GET", 'POST'])
def set_slider():
    # This returns the current slider value to the frontend
    global slider_value
    value = request.args.get('value', type=int)
    print(value)
    slider_value = 0
    return jsonify(value=slider_value)
	
@app.route('/status')
def status():
	global ack_status
	global brush_on
	global ai_result
	global brush_s
	now = time.time()
	hb_age = now-last_heartbeat
	heartbeat_ok = hb_age < 7
	brush_state = brush_on
	brush_speed = brush_s
	actuator_position = linear_control.getPosition()
	servo_position = motor_control.currentPosition
	return {"heartbeat_ok": heartbeat_ok, "heartbeat_age_sec": round(hb_age, 2), "last_ack": ack_status, "brush_on": brush_state,"brush_speed": brush_speed,"actuator_position": actuator_position,"servo_position": servo_position,"ai_result": ai_result}
	
# background watchdog
def heartbeat_monitor():
	while True:
		now = time.time()
		if now - last_heartbeat > 10:
			print("[WARNING] No hearbeat for >10 seconds")
		time.sleep(1) 

#background brush control
def brush_control_based_on_position():
	global brush_s
	brush_on = False
	while True:
		try:
			with shelve.open(DB_FILE) as db:
				pos = db.get('pos', 0)
				if pos != 0 and not brush_on:
					if brush_s == 0.00:
						brush_s = 0.5
						brush_control.set_pwm(brush_s) 
					brush_control.start()
					brush_on = True
					print("[BRUSH] Turned ON - actuator position:", pos)
				elif pos == 0 and brush_on:
					brush_control.stop()
					brush_on = False
					print("[BRUSH] Turned OFF - actuator position is zero")
		except Exception as e:
			print("[BRUSH MONITOR ERROR]:", e)
		time.sleep(0.5)

			
# start flask
if __name__ == '__main__':
	threading.Thread(target=heartbeat_monitor, daemon=True).start() 
	threading.Thread(target=brush_control_based_on_position, daemon=True).start()
	app.run(host='0.0.0.0', port=5000)

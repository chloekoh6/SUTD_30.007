#include <WiFi.h>
#include <WebServer.h>
#include <FastAccelStepper.h>
#include <PubSubClient.h>

// ——— WIFI ———
const char* SSID = "we paid sch fees for this";
const char* PASS = "urmomgreen";

// ——— HiveMQ public broker ———
const char* mqtt_server = "broker.hivemq.com";
const char* topic_command = "esp32/topic";
const char* topic_status = "rpi/topic";
const int mqtt_port = 1883;

// ——— MQTT and WiFi clients ———
WiFiClient espClient;
PubSubClient client(espClient);

// ——— Heartbeat ———
unsigned long lastHeartbeat = 0;

// ——— PINS ———
const int STEP_PIN_1 = 26, DIR_PIN_1 = 27, ENA_PIN_1 = 12;
const int STEP_PIN_2 = 32, DIR_PIN_2 = 33, ENA_PIN_2 = 25;

// —— No limit switches ——
const int LIMIT1_PIN = -1;
const int LIMIT2_PIN = -1;
const bool LIMIT_ACTIVE_LOW = true;  // unused now

// ——— MOTION PARAMETERS ———
const uint32_t MAX_SPEED_HZ      = 2000;
const uint32_t ACCEL_STEPS_SEC2  = 2000;
const uint32_t DECEL             = 2000;

// ——— Jog/Nudge ———
const uint32_t JOG_SPEED_HZ = 1200;
const uint32_t JOG_ACCEL    = 800;
const int32_t  NUDGE_STEPS  = 50;     // per-click nudge = 50

// ——— SOFT LIMITS (in steps) ———
// Set these to your working range after zeroing.
// Example: 0..18000 steps tall — change SOFT_MAX to your machine height in steps.
const long SOFT_MIN = -10000;
const long SOFT_MAX = 3000;

bool hold = false;

FastAccelStepperEngine engine;
FastAccelStepper *motor1 = nullptr, *motor2 = nullptr;

WebServer server(80);

// States for manual spinning
bool m1_spinning = false, m1_cw = true;
bool m2_spinning = false, m2_cw = true;

// Track target moves (move/moveTo) so loop won't stop them prematurely
bool m1_move_active = false;
bool m2_move_active = false;

// Track when motions are "linked" (sync up/down or sync hold) so both stop at limit
bool sync_linked = false;

// ——— MQTT ———
void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) msg += (char)payload[i];

  Serial.print("Received: ");
  Serial.println(msg);

  String ack = "ack:" + msg;
  client.publish(topic_status, ack.c_str());

  if (msg == "up") {                    // sync up, fetch: sync_cw, function: handleSyncCW
    handleSyncCW();
    String state = "ack:" + msg + " complete";
    Serial.print(state);
    client.publish(topic_status, state.c_str());
  } else if (msg == "down") {           // sync down, fetch: sync_ccw, function: handleSyncCW
    handleSyncCCW();
    String state = "ack:" + msg + " complete";
    Serial.print(state);
    client.publish(topic_status, state.c_str());
  } else if (msg == "stop") {           // sync stop, fetch: sync_stop, function: handleSyncStop
    handleSyncStop();
    String state = "ack:" + msg + " complete";
    Serial.print(state);
    client.publish(topic_status, state.c_str());
  } else if (msg == "home") {           // home
    handleGoHome();
    String state = "ack:" + msg + " complete";
    Serial.print(state);
    client.publish(topic_status, state.c_str());
  } else if (msg == "zero") {           // zero
    handleSetZero();
    String state = "ack:" + msg + " complete";
    Serial.print(state);
    client.publish(topic_status, state.c_str());
  } else if (msg == "holdup1") {           // hold and press up motor1
    handleJogStart(1, true);
    String state = "ack:" + msg + " complete";
    Serial.print(state);
    client.publish(topic_status, state.c_str());
  } else if (msg == "holddown1") {           // hold and press down motor1
    handleJogStart(1, false);
    String state = "ack:" + msg + " complete";
    Serial.print(state);
    client.publish(topic_status, state.c_str());
  } else if (msg == "holdup2") {           // hold and press up motor2
    handleJogStart(2, true);
    String state = "ack:" + msg + " complete";
    Serial.print(state);
    client.publish(topic_status, state.c_str());
  } else if (msg == "holddown2") {           // hold and press down motor2
    handleJogStart(2, false);
    String state = "ack:" + msg + " complete";
    Serial.print(state);
    client.publish(topic_status, state.c_str());
  } else if (msg == "holdall") {           // hold and press all
    handleJogStart(1, true);
    String state = "ack:" + msg + " complete";
    Serial.print(state);
    client.publish(topic_status, state.c_str());
  } else if (msg == "holdall") {           // hold and press all
    handleSyncJogStart();
    String state = "ack:" + msg + " complete";
    Serial.print(state);
    client.publish(topic_status, state.c_str());
  } else if (msg == "hold_stop") {           // sync , fetch: sync_stop, function: handleSyncStop
    handleSyncStop();
    String state = "ack:" + msg + " complete";
    Serial.print(state);
    client.publish(topic_status, state.c_str());
  }
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32Client1")) {
      Serial.println("connected!");
      client.subscribe(topic_command);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" trying again in 5 seconds...");
    }
  }
}

// —— helpers ——
void printPositions(const char* tag) {
  long p1 = motor1 ? motor1->getCurrentPosition() : 0;
  long p2 = motor2 ? motor2->getCurrentPosition() : 0;
  Serial.printf("[%s] pos1=%ld pos2=%ld\n", tag, p1, p2);
  std::string state = tag + std::string(" pos1 = ") + std::to_string(p1) + " pos2: " + std::to_string(p2);
  client.publish(topic_status, state.c_str());
}

long clampTarget(long t) {
  if (t < SOFT_MIN) return SOFT_MIN;
  if (t > SOFT_MAX) return SOFT_MAX;
  return t;
}

bool atLimit(long pos, bool dir_cw) {
  // Define CW as "increasing steps". If your wiring is reversed, limits still work on position.
  if (dir_cw)  return pos >= SOFT_MAX;
  else         return pos <= SOFT_MIN;
}

void stopBothIfLinked(const char* reason) {
  if (!sync_linked) return;
  m1_spinning = m2_spinning = false;
  if (motor1) { motor1->setAcceleration(DECEL); motor1->stopMove(); }
  if (motor2) { motor2->setAcceleration(DECEL); motor2->stopMove(); }
  m1_move_active = m2_move_active = false;
  Serial.printf("[LIMIT] %s — stopping BOTH (linked)\n", reason);
  printPositions("LIMIT_STOP_BOTH");
}

// —— SYNC handlers ——
void handleSyncCW() {
  m1_cw = true;
  m2_cw = false;  // mirrored for “up”
  // cancel any target moves
  m1_move_active = m2_move_active = false;
  if (motor1) motor1->stopMove();
  if (motor2) motor2->stopMove();

  m1_spinning = m2_spinning = true;
  sync_linked = true; // linked so one limit stops both
  Serial.println("[CMD] SYNC UP (CW/CCW)");
  printPositions("SYNC_UP");
  server.send(200, "text/plain", "SYNC CW");
}

void handleSyncCCW() {
  m1_cw = false;
  m2_cw = true;
  m1_move_active = m2_move_active = false;
  if (motor1) motor1->stopMove();
  if (motor2) motor2->stopMove();

  m1_spinning = m2_spinning = true;
  sync_linked = true;
  Serial.println("[CMD] SYNC DOWN (CCW/CW)");
  printPositions("SYNC_DOWN");
  server.send(200, "text/plain", "SYNC CCW");
}

void handleSyncStop() {
  m1_spinning = m2_spinning = false;
  m1_move_active = m2_move_active = false;
  if (motor1) { motor1->setAcceleration(DECEL); motor1->stopMove(); }
  if (motor2) { motor2->setAcceleration(DECEL); motor2->stopMove(); }
  sync_linked = false;
  Serial.println("[CMD] SYNC STOP");
  printPositions("SYNC_STOP");
  server.send(200, "text/plain", "SYNC STOP");
}

// —— Motor 1 ——
void handleM1CW()  { 
  sync_linked = false;
  m1_move_active = false; if (motor1) motor1->stopMove();
  m1_cw = true;  m1_spinning = true;  
  Serial.println("[CMD] M1 CW");  printPositions("M1_CW");  
  server.send(200, "text/plain", "M1 CW"); 
}
void handleM1CCW() { 
  sync_linked = false;
  m1_move_active = false; if (motor1) motor1->stopMove();
  m1_cw = false; m1_spinning = true;  
  Serial.println("[CMD] M1 CCW"); printPositions("M1_CCW"); 
  server.send(200, "text/plain", "M1 CCW"); 
}
void handleM1Stop(){ 
  m1_spinning = false; m1_move_active = false; 
  if (motor1) { motor1->setAcceleration(DECEL); motor1->stopMove(); }
  Serial.println("[CMD] M1 STOP"); printPositions("M1_STOP"); 
  server.send(200, "text/plain", "M1 STOP"); 
}

// —— Motor 2 ——
void handleM2CW()  { 
  sync_linked = false;
  m2_move_active = false; if (motor2) motor2->stopMove();
  m2_cw = true;  m2_spinning = true;  
  Serial.println("[CMD] M2 CW");  printPositions("M2_CW");  
  server.send(200, "text/plain", "M2 CW"); 
}
void handleM2CCW() { 
  sync_linked = false;
  m2_move_active = false; if (motor2) motor2->stopMove();
  m2_cw = false; m2_spinning = true;  
  Serial.println("[CMD] M2 CCW"); printPositions("M2_CCW"); 
  server.send(200, "text/plain", "M2 CCW"); 
}
void handleM2Stop(){ 
  m2_spinning = false; m2_move_active = false; 
  if (motor2) { motor2->setAcceleration(DECEL); motor2->stopMove(); }
  Serial.println("[CMD] M2 STOP"); printPositions("M2_STOP"); 
  server.send(200, "text/plain", "M2 STOP"); 
}

// —— Set Zero (home reference = pos 0) ——
void handleSetZero() {
  if (motor1) motor1->setCurrentPosition(clampTarget(-10000));
  if (motor2) motor2->setCurrentPosition(clampTarget(10000));
  Serial.println("[CMD] SET ZERO (both set to 0)");
  printPositions("SET_ZERO");
  server.send(200, "text/plain", "Positions set to ZERO");
}

// —— Home (return to absolute 0) ——
void handleGoHome() {
  // stop manual jog, start target moves
  m1_spinning = m2_spinning = false;
  sync_linked = false; // home moves are independent; both targets are safe (0 within bounds)

  if (motor1) {
    motor1->setSpeedInHz(MAX_SPEED_HZ);
    motor1->setAcceleration(ACCEL_STEPS_SEC2);
    motor1->moveTo(clampTarget(-10000));
    m1_move_active = true;
  }
  if (motor2) {
    motor2->setSpeedInHz(MAX_SPEED_HZ);
    motor2->setAcceleration(ACCEL_STEPS_SEC2);
    motor2->moveTo(clampTarget(10000));
    m2_move_active = true;
  }
  Serial.println("[CMD] HOME to pos=0 (moveTo(0) both)");
  printPositions("HOME_START");
  server.send(200, "text/plain", "Going to 0");
}

// —— Jog continuous: /jog_start?m=1&dir=cw  and /jog_stop?m=1 ——
void handleJogStart(int m, bool dir) {
  //int m = server.hasArg("m") ? server.arg("m").toInt() : 1;
  //String dir = server.hasArg("dir") ? server.arg("dir") : "cw";
  FastAccelStepper* mot = (m == 2 ? motor2 : motor1);

  if (!mot) { server.send(400, "text/plain", "Bad motor"); return; }
  // cancel any target move for this motor
  if (m == 1) { m1_move_active = false; motor1->stopMove(); }
  else        { m2_move_active = false; motor2->stopMove(); }

  mot->setSpeedInHz(JOG_SPEED_HZ);
  mot->setAcceleration(JOG_ACCEL);

  if (m == 1) { m1_spinning = true;  m1_cw = dir; }
  else        { m2_spinning = true;  m2_cw = true; }
  sync_linked = false; // manual single-motor jog not linked

  Serial.printf("[CMD] JOG START m=1 and dir=", dir);
  printPositions("JOG_START");
  server.send(200, "text/plain", "JOG START");
}

// —— Synced continuous jog (mirrored) ——
void handleSyncJogStart() {
  String dir = server.hasArg("dir") ? server.arg("dir") : "up"; // "up" or "down"
  // cancel any target moves
  m1_move_active = m2_move_active = false;
  if (motor1) motor1->stopMove();
  if (motor2) motor2->stopMove();

  m1_spinning = m2_spinning = true;
  m1_cw = (dir == "up");
  m2_cw = !(dir == "up");
  sync_linked = true; // stop both at limit

  if (motor1) { motor1->setSpeedInHz(JOG_SPEED_HZ); motor1->setAcceleration(JOG_ACCEL); }
  if (motor2) { motor2->setSpeedInHz(JOG_SPEED_HZ); motor2->setAcceleration(JOG_ACCEL); }

  Serial.printf("[CMD] SYNC JOG START dir=%s\n", dir.c_str());
  printPositions("SYNC_JOG_START");
  server.send(200, "text/plain", "SYNC JOG START");
}

void setup() {
  Serial.begin(115200);

  // WiFi
  WiFi.begin(SSID, PASS);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nIP: " + WiFi.localIP().toString());

  server.begin();

  // Init motor engine
  engine.init();

  motor1 = engine.stepperConnectToPin(STEP_PIN_1);
  if (motor1) {
    motor1->setDirectionPin(DIR_PIN_1);
    motor1->setEnablePin(ENA_PIN_1, true);   // active-LOW enable (common)
    motor1->setAutoEnable(true);
    motor1->setSpeedInHz(MAX_SPEED_HZ);
    motor1->setAcceleration(ACCEL_STEPS_SEC2);
  }

  motor2 = engine.stepperConnectToPin(STEP_PIN_2);
  if (motor2) {
    motor2->setDirectionPin(DIR_PIN_2);
    motor2->setEnablePin(ENA_PIN_2, true);   // active-LOW enable (common)
    motor2->setAutoEnable(true);
    motor2->setSpeedInHz(MAX_SPEED_HZ);
    motor2->setAcceleration(ACCEL_STEPS_SEC2);
  }

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  Serial.printf("[BOOT] Ready. Soft limits: [%ld .. %ld]\n", SOFT_MIN, SOFT_MAX);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  if (millis() - lastHeartbeat > 5000) {
    client.publish(topic_status, "heartbeat");
    lastHeartbeat = millis();
  }

  // MOTOR 1
  if (motor1) {
    long p = motor1->getCurrentPosition();

    if (m1_spinning) {
      // Stop at limits
      if (atLimit(p, m1_cw)) {
        m1_spinning = false;
        motor1->setAcceleration(DECEL);
        motor1->stopMove();
        Serial.printf("[LIMIT] M1 hit %s bound at pos=%ld\n", m1_cw ? "MAX" : "MIN", p);
        printPositions("M1_LIMIT");
        stopBothIfLinked("M1 reached bound");
      } else {
        if (m1_cw) motor1->runForward();
        else       motor1->runBackward();
      }
    } else if (m1_move_active) {
      // target move in progress — do not stop
      if (!motor1->isRunning()) {
        m1_move_active = false;
        Serial.println("[DONE] M1 target move complete");
        printPositions("M1_DONE");
      }
    } else {
      // idle
    }
  }

  // MOTOR 2
  if (motor2) {
    long p = motor2->getCurrentPosition();

    if (m2_spinning) {
      if (atLimit(p, m2_cw)) {
        m2_spinning = false;
        motor2->setAcceleration(DECEL);
        motor2->stopMove();
        Serial.printf("[LIMIT] M2 hit %s bound at pos=%ld\n", m2_cw ? "MAX" : "MIN", p);
        printPositions("M2_LIMIT");
        stopBothIfLinked("M2 reached bound");
      } else {
        if (m2_cw) motor2->runForward();
        else       motor2->runBackward();
      }
    } else if (m2_move_active) {
      if (!motor2->isRunning()) {
        m2_move_active = false;
        Serial.println("[DONE] M2 target move complete");
        printPositions("M2_DONE");
      }
    } else {
      // idle
    }
  }
}

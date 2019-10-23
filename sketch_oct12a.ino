#include <Keypad.h>
#include <Servo.h>

enum state {
  task1,
  task2,
  task3,
  unlocked
};


int ECHO_PIN = 6;
int TRIG_PIN = 7;
int SOUND_PIN = 21;
int SERVO_PIN = 3;
int BUZZER_PIN = 2;
int TASK_1_LED = 51;
int TASK_2_LED = 41;
int TASK_3_LED = 47;
int ERR_LED = 45;

char combo[4] = {'4', '3', '1', '2'};
int curr_combo_inx = 0;
char curr_combo[4] = {'#', '#', '#', '#'};

int task_1_claps = 0;
unsigned long last_sound = millis();
unsigned long last_ultrasonic_trigger = millis();
bool should_play = false;

state curr_state = task1;

const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

byte colPins[ROWS] = {28, 30, 8, 24}; //connect to the row pinouts of the keypad
byte rowPins[COLS] = {38, 36, 34, 32}; //connect to the column pinouts of the keypad

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );
Servo servo;

void setup() {
  // put your setup code here, to run once:
  pinMode(SOUND_PIN, INPUT); 
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TASK_1_LED, OUTPUT);
  pinMode(ERR_LED, OUTPUT);
  servo.attach(SERVO_PIN);
  resetState();


  attachInterrupt(digitalPinToInterrupt(SOUND_PIN), soundActive, HIGH);

  
  Serial.begin(9600); 
  
}

void setState(state st) {
  if (st == curr_state + 1) {
    if (curr_state == task1) {
      digitalWrite(TASK_1_LED, HIGH);  
    }

    if (curr_state == task2) {
      digitalWrite(TASK_2_LED, HIGH);    
    }

    if (curr_state == task3) {
      digitalWrite(TASK_3_LED, HIGH);  
      servo.write(90);
      delay(1500);
    }
    
    Serial.println(st);
    Serial.println(curr_state);
    curr_state = st;
  } else {
    curr_state = task1;
    digitalWrite(ERR_LED, HIGH); 
    digitalWrite(TASK_1_LED, LOW); 
    digitalWrite(TASK_2_LED, LOW); 
    digitalWrite(TASK_3_LED, LOW); 
    delay(1000);
    resetState();
  }
}

void resetState() {
  curr_state = task1;
  digitalWrite(ERR_LED, LOW);
  digitalWrite(TASK_1_LED, LOW);
  digitalWrite(TASK_2_LED, LOW);
  digitalWrite(TASK_3_LED, LOW);
  servo.write(350);
  delay(1500);
}

float getDistance() {
  float duration, distance; 
  digitalWrite(TRIG_PIN, LOW); 
  delayMicroseconds(2); 
  digitalWrite(TRIG_PIN, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(TRIG_PIN, LOW); 
  duration = pulseIn(ECHO_PIN, HIGH); 
  distance = (duration/2) / 29.1;
  return distance;
}

void playBuzzer() {
    tone(BUZZER_PIN, 440); 
    delay(1000);
    tone(BUZZER_PIN, 530); 
    delay(1000);
    noTone(BUZZER_PIN);
}

void soundActive() {
  unsigned long mil = millis();
  if (mil - last_sound > 2000 && task_1_claps < 3) {
    should_play = true;
    task_1_claps += 1;
    last_sound = millis();
  }
}

bool comboValid() {
  Serial.println("eeage");
  for (int i = 0; i < 4; i++) {
    if (curr_combo[i] != combo[i]) {
      return false;  
    }
  }

  return true;
}

void resetCombo() {
  curr_combo_inx = 0;
  for (int i = 0; i < 4; i++) {
    curr_combo[0] = '#';
  }
}

void loop() {
  // put your main code here, to run repeatedly:

//  bool sound_activated = digitalRead(SOUND_PIN);
//  
//  if (sound_activated) {
//
//  }

  if (should_play) {
    playBuzzer();
    should_play = false;
  }

  if (task_1_claps == 3) {
    task_1_claps = 0;
    setState(task2);
  }
  
  float distance = getDistance();
  if (abs(distance - 10.0) < 2.0 && millis() - last_ultrasonic_trigger > 5000) {
    Serial.println(last_ultrasonic_trigger);
    last_ultrasonic_trigger = millis();
    setState(task3);
  }

  char key = keypad.getKey();
  if (key) {
    if (key == '#') {
      resetCombo();
    } else {
      curr_combo[curr_combo_inx] = key;
      curr_combo_inx += 1;

      if (curr_combo_inx == 4) {
        Serial.println("eee");
        if (comboValid()) {
          setState(unlocked);
          resetCombo();
        } else {
          resetCombo();
        }
      }
    }
  }
}

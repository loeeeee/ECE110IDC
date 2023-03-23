#include <Servo.h>                           // Include servo library
#include <SoftwareSerial.h>

Servo servoLeft;                             // Declare left servo signal
Servo servoRight;                            // Declare right servo signal

#define LCDpin 14

SoftwareSerial LCDSerial = SoftwareSerial(255, LCDpin);

#define rightServoPin 11
#define leftServoPin 12
//Pins for QTI connections on board
#define lineSensor1 51 // Right
#define lineSensor2 49 // Middle
#define lineSensor3 47 // Left
// RGB pin
#define RGBred 45
#define RGBgreen 46
#define RGBblue 44
// LED pins
#define yellowPin 8
#define bluePin 9


const int leftSpeedDis = 0;
const int rightSpeedDis = 0;

int ledCycle;
bool noRepeats = false;

// length of RFID code
int len = 14;
char val = 0;

int detectedPosition = 0;
int part_1_result[5];

bool ERROR = false;

void setup()                                 // Built in initialization block
{
  // Serial initialization
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  LCDSerial.begin(9600);

  servoLeft.attach(leftServoPin);                      // Attach left signal to pin 13
  servoRight.attach(rightServoPin);                     // Attach right signal to pin 12

  // Attach built-in RGB LED
  pinMode(RGBred, OUTPUT);
  pinMode(RGBgreen, OUTPUT);
  pinMode(RGBblue, OUTPUT);

  // Pin mode for LEDs
  pinMode(yellowPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Initalizaiton flashes
  on_off_RGB(127, 0, 64, 50, false);

  ledCycle = 0;

  LCDSerial.write(12); // clear
  delay(10);
  LCDSerial.write(22); // no cursor no blink
  delay(10);
  LCDSerial.write(17); // backlight
  delay(10);
  //play_song();
}

void loop() {
  movement();
  //delay(100);
}

//Defines funtion 'rcTime' to read value from QTI sensor
long qti_read(int pin)
{
  pinMode(pin, OUTPUT);    // Sets pin as OUTPUT
  digitalWrite(pin, HIGH); // Pin HIGH
  delay(1);                // Waits for 1 millisecond
  pinMode(pin, INPUT);     // Sets pin as INPUT
  digitalWrite(pin, LOW);  // Pin LOW
  long time = micros();    // Tracks starting time
  while(digitalRead(pin)); // Loops while voltage is high
  time = micros() - time;  // Calculate decay time
  return time;  // Return decay time
}

int qti_state(){
  // takes nothing, return qti state
  return 4 * (qti_read(lineSensor1) > 250) + 2 * (qti_read(lineSensor2) > 250) + (qti_read(lineSensor3) > 250);
}

void sensing() {
  char rfidData[len+1] = {};
  int get_more = 1;
  int i = 0;

  while(get_more == 1){
    if(Serial1.available() > 0) {
    val = Serial1.read();

      // Handle unprintable characters
      switch(val) {
        case 0x2: break;               // start of transmission - do not save
        case 0x3: get_more = 0; break; // end of transmission - done with code
        case 0xA: break;               // line feed - do not save
        case 0xD: break;               // carriage return - do not save
        default:  rfidData[i]=val; i+=1;  break; // actual character
      }
    }
  }
  Serial.println(rfidData[9]);
  if (rfidData[9] == 'D') {
    detectedPosition = ledCycle;
    flashYellow();
  } else {
    flashBlue();
  }
}

void movement(){
  // takes qti_state, move the robort
  int temp_qti_state = qti_state();

  if (temp_qti_state != 7) {
    noRepeats = false;
  }

  switch (temp_qti_state) {
    case 0:
      // All white
      // Make a circle
      neural_steer(2, true);
      break;
    case 1:
      // Right is black
      // Turn Right
      turn_left(5);
      break;
    case 2:
      // Middle is black
      // Go
      move_forward(1);
      break;
    case 3:
      // Right and Middle are black
      // Turn right
      turn_right(5);
      break;
    case 4:
      // Left is Black
      // Turn left
      turn_right(5);
      break;
    case 5:
      // Left and Right is black
      // Panic
      on_off_RGB(64, 0, 127, 500, false);
      delay(10);
      on_off_RGB(127, 0, 64, 500, false);
      break;
    case 6:
      // Left Middle are black
      // Turn left
      turn_left(5);
      break;
    case 7:
      // All black
      // Stop for a second
      if(!noRepeats) {
        stop_move(1000);
        cycleLED();
        sensing();
        noRepeats = true;
      } else {
        move_forward(1);
      }

      break;
    default:
      Serial.println("WTF!");
      on_off_RGB(127, 0, 64, 500, false);
      delay(10);
      on_off_RGB(64, 0, 127, 500, false);
      break;
  }
  return;
}

void cycleLED() {
  int mod = ledCycle % 6;
  switch (mod) {
    case 0:
      on_off_RGB(255, 0, 0);
      break;
    case 1:
      on_off_RGB(255, 255, 0);
      break;
    case 2:
      on_off_RGB(0, 255, 0);
      break;
    case 3:
      on_off_RGB(0, 0, 255);
      break;
    case 4:
      on_off_RGB(255, 0, 255);
      LCDSerial.write(12); // clear
      LCDSerial.print(detectedPosition);   
      break;
    default:
      on_off_RGB(255, 0, 0);
      // Shout to all
      shout_and_listen();
      on_off_RGB(255, 255, 255, 0, true);
      stop_move(100000000);
      break;
  }
  ledCycle++;
  return;
}

void shout_and_listen(){
  int flag = 0; // this keep track of how much of the array is filled.
  bool isFinish = false;
  int countdown = 100;
  if(detectedPosition > 4){
    ERROR = true;
  }
  while(!isFinish){
    if (countdown%10 == 0){
      broadcast(106 + detectedPosition);
    }
    if(Serial2.available() > 0){
      char c = Serial2.read();
      if(c>='f' && c<='j'){
        part_1_result[1] = (int) c-101;
      }
      if(c>='k' && c<='o'){
        part_1_result[2] = (int) c-106;
      }
      if(c>='p' && c<='t'){
        part_1_result[3] = (int) c-111;
      }
      if(c>='u' && c<='y'){
        part_1_result[4] = (int) c-116;
      }
      on_off_RGB(0,255,0);
    }
    countdown--;
    delay(10);
    for(int i = 0;i<5;i++){
      LCDSerial.write(13);
      LCDSerial.print(part_1_result[i]);
      if(part_1_result[i]==0){
        isFinish=true;
      }
    }
  }
  if (part_1_result[2] != detectedPosition){
    ERROR = true;
  }
  if(ERROR){
    
    //do_it_again();
    return;
  }
}

void do_it_again(){
  neural_steer(120, true);
  int hash_cnt = 0;
  while(hash_cnt <= 4){
    int temp_qti_state = qti_state();
    switch (temp_qti_state) {
      case 0:
        // All white
        // Make a circle
        break;
      case 1:
        // Right is black
        // Turn Right
        turn_left(5);
        break;
      case 2:
        // Middle is black
        // Go
        move_forward(1);
        break;
      case 3:
        // Right and Middle are black
        // Turn right
        turn_right(5);
        break;
      case 4:
        // Left is Black
        // Turn left
        turn_right(5);
        break;
      case 5:
        // Left and Right is black
        // Panic
        on_off_RGB(64, 0, 127, 50, false);
        delay(10);
        on_off_RGB(127, 0, 64, 50, false);
        break;
      case 6:
        // Left Middle are black
        // Turn left
        turn_left(5);
        break;
      case 7:
        // All black
        // Stop for a second
        hash_cnt++;
        move_forward(500);
        break;
      default:
        Serial.println("WTF!");
        on_off_RGB(127, 0, 64, 50, false);
        delay(10);
        on_off_RGB(64, 0, 127, 50, false);
        break;
    }
  }
  neural_steer(120, true);
  ledCycle = 0;
  movement();
  return;
}

// Helper
void shout_and_listen_backup(char code){
  int position[5] = {0,0,0,0,0};
  while(true){
    if(Serial2.available()){
      char c = Serial2.read();
      Serial2.print(code);
      if(c>='f' && c<='j'){
        position[1] = (int) c-101;
      }
      if(c>='k' && c<='o'){
        position[2] = (int) c-106;
      }
      if(c>='p' && c<='t'){
        position[3] = (int) c-111;
      }
      if(c>='u' && c<='y'){
        position[4] = (int) c-116;
      }
      bool communicationSuccessful = true;
      for(int i = 0;i<5;i++){
        //Serial.println(position[i]);
        if(position[i]==0){
          communicationSuccessful=false;
        }
      }
      if(communicationSuccessful){
        break;
      }
    }
  }

}

void on_off_RGB(int R, int G, int B){
  analogWrite(RGBred, 255 + -1 * R);
  analogWrite(RGBgreen, 255 + -1 * G);
  analogWrite(RGBblue, 255 + -1 * B);

  delay(500);

  analogWrite(RGBred, 255);
  analogWrite(RGBgreen, 255);
  analogWrite(RGBblue, 255);

  return;
}

void on_off_RGB(int R, int G, int B, int time, bool isPersistant){
  analogWrite(RGBred, 255 + -1 * R);
  analogWrite(RGBgreen, 255 + -1 * G);
  analogWrite(RGBblue, 255 + -1 * B);
  delay(500);

  if (!isPersistant) {
    analogWrite(RGBred, 255);
    analogWrite(RGBgreen, 255);
    analogWrite(RGBblue, 255);
  }
  return;
}

void flashBlue() {
  digitalWrite(bluePin, HIGH);
  delay(500);
  digitalWrite(bluePin, LOW);
}

void flashYellow() {
  digitalWrite(yellowPin, HIGH);
  delay(500);
  digitalWrite(yellowPin, LOW);
}

void broadcast(char input){
  Serial2.print((char)(input));
  on_off_RGB(255, 0, 0);
  return;
}

int speed(bool isRight, int percentage){
  // return the speed
  int default_num;
  if (isRight) {
    default_num = 1500 + 200 * percentage / 100 + rightSpeedDis;
  }else{
    default_num = 1500 - 200 * percentage / 100 + leftSpeedDis;
  }
  return default_num;
}

void move_forward(int distance){
  servoLeft.writeMicroseconds(speed(false, 100));
  servoRight.writeMicroseconds(speed(true, 100));
  delay(distance);
}

void move_backward(int distance){
  servoLeft.writeMicroseconds(speed(false, -100));
  servoRight.writeMicroseconds(speed(true, -100));
  delay(distance);
}

void turn_left(int degree){
  servoLeft.writeMicroseconds(speed(false, 100));
  servoRight.writeMicroseconds(speed(true, 20));
  delay(degree * 4);
}

void turn_right(int degree){
  servoLeft.writeMicroseconds(speed(false, 20));
  servoRight.writeMicroseconds(speed(true, 100));
  delay(degree * 4);
}

void neural_steer(int degree, bool isRight){
  servoLeft.writeMicroseconds(speed(!isRight, -100));
  servoRight.writeMicroseconds(speed(isRight, 100));
  delay(degree * 5);
}

void stop_move(int time){
  servoLeft.writeMicroseconds(speed(false, 0));
  servoRight.writeMicroseconds(speed(true, 0));
  delay(time);
}

void play_song() {
  int durs[9]  = {211, 212, 212, 211, 212, 
                212, 212, 212, 212
  };
  int octs[9]  = {216, 216, 216, 216, 216, 
                  216, 216, 215, 216
  };
  int notes[9] = {227, 227, 227, 223, 227,
                230, 232, 230, 232
  };
 for(long k=0; k<9; k++){
    LCDSerial.write(durs[k]); LCDSerial.write(octs[k]); LCDSerial.write(notes[k]);
    int len = 214 - durs[k];
    float del = 2000 / pow(2, len);
    delay(int(del*1.1));
  }
}
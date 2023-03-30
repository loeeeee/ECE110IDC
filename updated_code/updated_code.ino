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

// for servo calibration
const int leftSpeedDis = 0;
const int rightSpeedDis = 0;

bool no_repeats = false;

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

  attach_servos();

  // Attach built-in RGB LED
  pinMode(RGBred, OUTPUT);
  pinMode(RGBgreen, OUTPUT);
  pinMode(RGBblue, OUTPUT);

  // Pin mode for LEDs
  pinMode(yellowPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Initalizaiton flashes
  flash_RGB(127, 0, 64, 50, false);

  LCDSerial.write(12); // clear
  delay(10);
  LCDSerial.write(22); // no cursor no blink
  delay(10);
  LCDSerial.write(17); // backlight
  delay(10);
  //play_song();
}

void loop() {
  detecting_phase();

  detach_servos();

  shout_and_listen();
   
  flash_RGB(255, 255, 255);
  attach_servos();
  reverse_phase();

  wait_to_go();

  end_question_mark();
  /*
  */

  how_do_we_get_here();
  detach_servos();  
}

// phase during which the bot moves to each hash and collects data
void detecting_phase() {

  for (int hash_count = 0; hash_count < 5; hash_count++) {
    no_repeats = true;
    while (move_to_hash()); // move to next hash (false if on hash)
    stop_move(1000);        // pause for 1 second
    cycle_LED(hash_count);
    sensing(hash_count);    // read RFID at location (won't happen at last line)
  }
  no_repeats = true;
  while (move_to_hash()); // move to next hash (false if on hash)
}

void shout_and_listen(){

  int countdown = 0;

  while(true){
    if (countdown % 10 == 0){
      broadcast(106 + detectedPosition);
      flash_RGB(255,0,0);
    }
    if(Serial2.available() > 0){
      char c = Serial2.read();
      if (c == 'z') {
        LCDSerial.write(13);
        LCDSerial.println("Detected Z!");
        return;
      }
      flash_RGB(0,255,0);
      LCDSerial.write(13);
      LCDSerial.print(c);
    }
    countdown++;
    delay(2);
  }
}

void reverse_phase() {
  no_repeats = true;
  move_backward(550); // oves forward
  return;
}

void wait_to_go() {
  while(true) {
    flash_RGB(255, 0, 0);
    stop_move(1);

    if(Serial2.available() > 0){
      char c = Serial2.read();
      if (c == (char)detectedPosition) {
        LCDSerial.write(13);
        LCDSerial.println("Moving to final position!");
        return;
      }
    }
  }
}

void end_question_mark() {
  no_repeats = true;

  for (int hash_count = 0; hash_count < 3; hash_count++) { // move forwards three hashes
    no_repeats = true;
    while (move_to_hash()); // move to next hash (false if on hash)
    stop_move(50);
    flashYellow();
  }
  move_forward(140);
  tank_turn(600, true);
}

void how_do_we_get_here(){
  no_repeats = true;

  for (int hash_count = 0; hash_count < 3; hash_count++) { // move forwards three hashes
    no_repeats = true;
    while (move_curve_to_hash()); // move to next hash (false if on hash)
    stop_move(50);
    flashYellow();
  }
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

void sensing(int hash_count) {
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
    detectedPosition = hash_count;
    flashYellow();
  } else {
    flashBlue();
  }
}



bool move_to_hash(){
  // takes qti_state, move the robort
  int temp_qti_state = qti_state();

  if (temp_qti_state != 7) {
    no_repeats = false;
  }

  switch (temp_qti_state) {
    case 0:
      // All white
      // Make a circle
      move_forward(15);
      break;
    case 1:
      // Right is black
      // Turn Right
      forward_left(5);
      break;
    case 2:
      // Middle is black
      // Go
      move_forward(1);
      break;
    case 3:
      // Right and Middle are black
      // Turn right
      forward_right(5);
      break;
    case 4:
      // Left is Black
      // Turn left
      forward_right(5);
      break;
    case 5:
      // Left and Right is black
      // PANIC!!!
      break;
    case 6:
      // Left Middle are black
      // Turn left
      forward_left(5);
      break;
    case 7:
      // All black
      // Stop for a second
      if(!no_repeats) {
        return false;
      } else {
        move_forward(1);
      }
      break;

    default:
      // PANIC!!!
      break;
  }
  return true;
}

bool move_curve_to_hash(){
  // takes qti_state, move the robort
  int temp_qti_state = qti_state();

  if (temp_qti_state != 7) {
    no_repeats = false;
  }

  switch (temp_qti_state) {
    case 0:
      // All white
      // Make a circle
      move_forward(15);
      break;
    case 1:
      // Right is black
      // Turn Right
      turn_left(20);
      LCDSerial.write(13);
      LCDSerial.print("Right is black.");
      break;
    case 2:
      // Middle is black
      // Go
      move_forward(1);
      break;
    case 3:
      // Right and Middle are black
      // Turn right
      turn_right(20);
      break;
    case 4:
      // Left is Black
      // Turn left
      turn_right(20);
      LCDSerial.write(13);
      LCDSerial.print("Right is black.");
      break;
    case 5:
      // Left and Right is black
      // PANIC!!!
      break;
    case 6:
      // Left Middle are black
      // Turn left
      turn_left(60);
      break;
    case 7:
      // All black
      // Stop for a second
      if(!no_repeats) {
        return false;
      } else {
        move_forward(1);
      }
      break;

    default:
      // PANIC!!!
      break;
  }
  return true;
}

bool move_back_to_hash() {
  int temp_qti_state = qti_state();

  if (temp_qti_state != 7) {
    no_repeats = false;
  }

  switch (temp_qti_state) {
    case 0:
      // All white
      // Make a circle
      tank_turn(2, true);
      break;
    case 1:
      // right is black
      // Turn Right
      backward_right(10);
      break;
    case 2:
      // Middle is black
      // Go
      move_backward(1);
      break;
    case 3:
      // Right and Middle are black
      // Turn left
      backward_left(10);
      break;
    case 4:
      // Left is Black
      // Turn left
      backward_left(10);
      break;
    case 5:
      // Left and Right is black
      // PANIC!!!
      break;
    case 6:
      // Left Middle are black
      // Turn right
      backward_right(10);
      break;
    case 7:
      // All black
      // Stop for a second
      if (!no_repeats) {
        return false;
      } else {
        move_backward(1);
      }
      break;

    default:
      // PANIC!!!
      break;
  }

  return true;
}

void cycle_LED(int hash_count) {
  switch (hash_count) {
    case 0:
      flash_RGB(255, 0, 0);
      break;
    case 1:
      flash_RGB(255, 255, 0);
      break;
    case 2:
      flash_RGB(0, 255, 0);
      break;
    case 3:
      flash_RGB(0, 0, 255);
      break;
    case 4:
      flash_RGB(255, 0, 255);
      LCDSerial.write(12); // clear
      LCDSerial.print(detectedPosition);   
      break;
    default:
      flash_RGB(255, 255, 255);
      break;
  }
}

void flash_RGB(int R, int G, int B){
  analogWrite(RGBred, 255 + -1 * R);
  analogWrite(RGBgreen, 255 + -1 * G);
  analogWrite(RGBblue, 255 + -1 * B);

  delay(500);

  analogWrite(RGBred, 255);
  analogWrite(RGBgreen, 255);
  analogWrite(RGBblue, 255);

  return;
}

void fast_RGB(int R, int G, int B) {
  analogWrite(RGBred, 255 + -1 * R);
  analogWrite(RGBgreen, 255 + -1 * G);
  analogWrite(RGBblue, 255 + -1 * B);

  delay(200);

  analogWrite(RGBred, 255);
  analogWrite(RGBgreen, 255);
  analogWrite(RGBblue, 255);

}


void flash_RGB(int R, int G, int B, int time, bool isPersistant){
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
  flash_RGB(255, 0, 0);
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
  servoRight.writeMicroseconds(speed(true, 0));
  delay(degree);
}

void turn_right(int degree){
  servoLeft.writeMicroseconds(speed(false, 0));
  servoRight.writeMicroseconds(speed(true, 100));
  delay(degree);
}

void forward_left(int degree){
  servoLeft.writeMicroseconds(speed(false, 100));
  servoRight.writeMicroseconds(speed(true, 20));
  delay(degree);
}

void forward_right(int degree){
  servoLeft.writeMicroseconds(speed(false, 20));
  servoRight.writeMicroseconds(speed(true, 100));
  delay(degree);
}

void backward_left(int degree){
  servoLeft.writeMicroseconds(speed(false, -100));
  servoRight.writeMicroseconds(speed(true, -20));
  delay(degree);
}

void backward_right(int degree){
  servoLeft.writeMicroseconds(speed(false, -20));
  servoRight.writeMicroseconds(speed(true, -100));
  delay(degree);
}

void tank_turn(int degree, bool isRight){
  servoLeft.writeMicroseconds(speed(!isRight, -100));
  servoRight.writeMicroseconds(speed(isRight, 100));
  delay(degree);
}

void stop_move(int time){
  servoLeft.writeMicroseconds(speed(false, 0));
  servoRight.writeMicroseconds(speed(true, 0));
  delay(time);
}

void attach_servos() {
  servoLeft.attach(leftServoPin);                      // Attach left signal to pin 13
  servoRight.attach(rightServoPin);                     // Attach right signal to pin 12
}

void detach_servos() {
  servoLeft.detach();                      // Attach left signal to pin 13
  servoRight.detach();                     // Attach right signal to pin 12
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
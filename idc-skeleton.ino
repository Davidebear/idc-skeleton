// The basic model here is a function line_follow that contains 
// Everything needed to traverse the line
// The main part to add is a sense funciton for each bot
// and the after line follow stuff

// –––––––––––––––––––––––––––––––––––––––––––
// 0. LIBRARIES, MACROS, INIT
// –––––––––––––––––––––––––––––––––––––––––––

// libraries
#include <Wire.h> //Allows us to work with I2C
#include "Adafruit_TCS34725.h" //Color sensor
#include <Adafruit_TiCoServo.h> //Servos
#include <SoftwareSerial.h> //LCD

//TASK #1 : Thermal
#include <SparkFunMLX90614.h> //Click here to get the library: http://librarymanager/All#Qwiic_IR_Thermometer by SparkFun 
IRTherm therm; // Create an IRTherm object to interact with throughout
int anomaly_nr = 0; //TODO: ask thermal team what this is about.

//TASK #2 : Hall & Dino Stuff
//TODO: determineCondtiion()?
const int Hall_In = 0; // input pin on L3, analog zero
const float VCC = 5.0; // power voltage
const float Hall_sensitivity = 0.005; // 5 mV/G 
// bool detect = false; // bool for detection
int dinoScore = 0; // where is the magnet?
bool escaped = false; // did the dino escape?
bool everyone = false;
int botstates[] = {0,0,0,0};

//TASK #3: Color
// Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

//TASK #4: RFID

//TASK #5 : Ping
const int pingPin = 50;

// macros
#define LCDpin 14 //Used to communicate with LCD
#define Xrec 17 //Receive pin for XBee
#define Xsend 16 //Send pin for XBee
#define togglePin 3 //CHANGE: set toggle pin

#define redpin 45
#define bluepin 44
#define greenpin 46

#define commonAnode true
byte gammatable[256]; //Array used to calculate gamma corrections to turn RGB measurements into human-observable colors.

// Serial init
SoftwareSerial mySerial = SoftwareSerial(255, LCDpin);

// Servo init
Adafruit_TiCoServo servoLeft;
Adafruit_TiCoServo servoRight;

// QTI controls
int pins[] = {47, 51, 52};
bool count = true; // set to false if you need to identify hashes!

int values[3]; //Stores whether each QTI is on the line or not
int durations[3]; //Array which stores decay times of capacitors in QTI sensors (proxy for darkness of a surface)
int threshold = 100; //threshold durations are compared to to determine if the corresponding entry of values is 0 or 1
int hash_count = 0;

// Task selection
int taskOptions[5] = {1, 2, 3, 4, 5}; // represents task challenge
int task = taskOptions[3]; //CHANGE: Change based on task

// score handling
int score = 0;
int finalScores[5] = {0, 0, 0, 0, 0};

// communication handling 
bool isTxRx = false;

//declare
void initLCD();
void initBuiltInLED();
void init_therm();

// –––––––––––––––––––––––––––––––––––––––––––
// 1. SETUP FUNC
// –––––––––––––––––––––––––––––––––––––––––––
void setup() {
  Serial.begin(9600); 
  Serial1.begin(9600);
  Serial2.begin(9600); 

  Serial.println("serial setup.");

  initLCD(); // init LCD
  initBuiltInLED(); // init LCD

  Serial.println("LCD, LED");
  
  servoLeft.attach(11); // mount Servos
  servoRight.attach(12); //

  Serial.println("Serial mounted");

  Wire.begin(); // join I2C bus

  pinMode(togglePin, INPUT);// togglePin

  switch (task) {
    case 1:
      init_therm();
      break;
    case 2:
      // hall
      break;
    case 3:
      break;
    case 4:
      break;
    case 5:
      break;
  // Alternatively, TEMP_F can be replaced with TEMP_C for Celsius or TEMP_K for Kelvin.
  }

  // Serial.println(taskToString(task));
}

// –––––––––––––––––––––––––––––––––––––––––––
// 1.2 SETUP HELPER FUNCTIONS
// –––––––––––––––––––––––––––––––––––––––––––

void init_therm() {
  if (therm.begin() == false){ // Initialize thermal IR sensor
        Serial.println("Qwiic IR thermometer did not acknowledge! Freezing!");
        while(1);
      }
      Serial.println("Qwiic IR Thermometer did acknowledge.");
      therm.setUnit(TEMP_F); // Set the library's units to Farenheit (sic)
      delay(10);
}

void initBuiltInLED() {
  // Set pin modes
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);

  // start with light off
  analogWrite(redpin, 255);
  analogWrite(greenpin, 255);
  analogWrite(bluepin, 255);
}

void initLCD() {
  mySerial.begin(9600);
  delay(100);
  mySerial.write(12); // clear
  delay(10);
  mySerial.write(22); // no cursor no blink
  delay(10);
  mySerial.write(17); // backlight
  delay(10);
}

// –––––––––––––––––––––––––––––––––––––––––––
// 2. LOOP FUNC
// –––––––––––––––––––––––––––––––––––––––––––
void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("in the loop");
  while(hash_count<5){
    line_follow(); 
  }

  servoLeft.detach();
  servoRight.detach();

  show_bot_score(); //display individual score

  isTxRx = true;
  tx_rx(); // TODO: written, needs to be tested.

  show_group_score();

  sync_diagnoses();  //TODO: facilitated by Michael
}

// –––––––––––––––––––––––––––––––––––––––––––
// 3. USER-DEFINED FUNCS
// –––––––––––––––––––––––––––––––––––––––––––
void check_qti_sensors() { // updates value array
  for (int i = 0; i < 3; i++) {
    long duration = 0;                      //
    pinMode(pins[i], OUTPUT);               // code that actually
    digitalWrite(pins[i], HIGH);            // reads from the sensor
    delay(1);                               //
    pinMode(pins[i], INPUT);                // low values from the sensor
    digitalWrite(pins[i], LOW);             // are dark objects, high
    while (digitalRead(pins[i])) {          // values are light objects
      duration++;                           //
    }

    durations[i] = duration;                // store values in arrays
    values[i] = (duration > threshold);
  }
}
void line_follow() {
  check_qti_sensors(); // determine movement

  Serial.println(values[0]);
  Serial.println(values[1]);
  Serial.println(values[2]);

  if (values[0] && values[2]) { // HASH FOUND
    servoLeft.writeMicroseconds(1500);
    servoRight.writeMicroseconds(1500);
    delay(500);
    hash_count++; // hash count inc.
    
    // sense();

    //Move past hash    
    servoLeft.writeMicroseconds(1700);
    servoRight.writeMicroseconds(1300);
    delay(200);
  }
  else if (values[0]) {
    servoLeft.writeMicroseconds(1300);
    servoRight.writeMicroseconds(1300);
  }
  else if (values[2]) {
    servoLeft.writeMicroseconds(1700);
    servoRight.writeMicroseconds(1700);
  }
  else {
    //drive forward
    servoLeft.writeMicroseconds(1700);
    servoRight.writeMicroseconds(1300);
  }

}
void sense() { //TODO: populate with sensor func handling and update score if found
  //options : 1) RFID, 2) IR, 3) Ping, 4) Hall Effect, 5) Color
  // count = false;
  bool positive_signal = false;
  
  switch (task) {
    case 1:
      sense_thermal();
      break;
    case 2:
      sense_hall();
      break;
    case 3:
      sense_color();
      break;
    case 4:
      sense_rfid();
      break;
    case 5:
      sense_ping();
      break;
    default:
      Serial.println("Incorrect task assigned");
      break;
  }

  if (positive_signal) {
    score = hash_count;
    finalScores[task-1] = score;
  }

}

bool sense_thermal(){
 // Call therm.read() to read object and ambient temperatures from the sensor.
 if (therm.read()){
  set_RGBi(255, 0, 0); 
  Serial.print("Object: " + String(therm.object(), 2));
  Serial.println("F");
  Serial.print("Ambient: " + String(therm.ambient(), 2));
  Serial.println("F");
  Serial.println();
  delay(200);
  set_RGBi(0,0,0);
  
  if (therm.object() < 50){
    set_RGBi(0, 255, 0);
    anomaly_nr = hash_count;
    Serial.print("Cold object at: ");
    Serial.println(hash_count); 
    delay(200);
    set_RGBi(0, 0, 0); 
    return true; 
    }

  else{
    return false;
  }
  
  }
}
bool sense_hall() { // internal 
  bool detect = false;
  float Hall_Reading = analogRead(Hall_In); // takes reading
  float Hall_Voltage = Hall_Reading * 5.0 / 1023.0; // converts
  float Hall_Gauss = (Hall_Voltage - (VCC/2)) / 0.005; // computes flux density
  
  if (Hall_Gauss > 75 || Hall_Gauss < -75) {
    // turn on the red light
    Serial.println("North pole");
    set_RGBi(0, 255,0);
    delay(100);
    set_RGBi(0, 0,0);
    detect = true;
  }
  else {
    Serial.println("No pole");
    set_RGBi(255, 0,0);
    delay(100);
    set_RGBi(0, 0,0);
    detect = false;
  }

  if (detect) {
    dinoScore = hash_count;
  }
  return detect;
}
bool sense_color() {
  bool yes = false;
  char value;
  if(Serial1.available() > 0 && Serial1.read() == 0x2) {
    yes = true;
    set_RGBi(255, 0, 0);
    while (Serial1.read() != 0x3) {
      value = Serial1.read();
      Serial.print(value);
      Serial.print("\t");
    }
    delay(250);
    set_RGBi(0, 0, 0);
  }else{
    set_RGBi(0,255,0);
    delay(250);
    set_RGBi(0,0,0);
  }
  return yes;
}
bool sense_rfid() {
  char val = 0; // variable to store the data from the serial port
  bool dataRead = false;

  while (Serial1.available() > 0) {
    val = Serial1.read(); // Read the data but do nothing with it
    dataRead = true;      // Indicate that data has been read
  }

  // Return true if any data was read, false otherwise
  return dataRead;
}
//ping stuff
long microsecondsToInches(long microseconds) {
  return microseconds / 74 / 2;
}
long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}
bool sense_ping(){
  long duration, inches, cm;

  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // Read pin
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  // convert the time into a distance
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);

  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();

  if (cm < 8 ){
    return true;
  } else {
    return false;
  }

  delay(200);
}

//displays
void show_bot_score() { //TODO: confirm LCD print is correct
  mySerial.write(128);
  mySerial.print(score);
}

// communications
void tx_rx() { // loop where send every mod 10, read other 9 times
  int txrx_count = 0;

  char encoded_val = ((4+task)*10) + (score-1); //check math

  while (isTxRx) {
    
    if ((txrx_count % 10 == 0) & (txrx_count < 100)) {
      send_my_score(encoded_val);
      txrx_count++;
    }
    else {
      recieve();
      txrx_count++;
    }

    if (check_full_recieve()) {
      isTxRx = false;
    }
  }
  
}

void send_my_score(char encoded_val) {
  Serial2.print(encoded_val);
}

void recieve() {
  if (Serial2.available()) {
    char score_read = Serial2.read();
    int to_read = (int)score_read;
    decode_score(to_read);
  }
}

bool check_full_recieve() {
  bool ret = true;
  for (int i = 0; i++; i<5) {
    if (finalScores[i] == 0) {
      ret = false;
      break;
    }
  }
  return ret;
}

void decode_score(int encoded_score) {
  int id = int((encoded_score-50)/10);
  int pos = (encoded_score % 10);

  finalScores[id] = (pos+1);

}

void show_group_score() {
  for (int i = 1; i < 6; i++) {
    score = finalScores[i-1];
    switch (i) {
      case 1:
        mySerial.write(128);
        mySerial.print("1=");
        mySerial.print(score);
        break;

      case 2:
        mySerial.write(132);
        mySerial.print("2=");
        mySerial.print(score);
        break;

      case 3:
        mySerial.write(136);
        mySerial.print("3=");
        mySerial.print(score);
        break;

      case 4:
        mySerial.write(140);
        mySerial.print("4=");
        mySerial.print(score);
        break;

      case 5:
        mySerial.write(148);
        mySerial.print("5=");
        mySerial.print(score);
        break;
    }
  }
}

// color code
void set_RGB(int r, int g, int b){
  // Set RGB LED pins based on low=bright (default)
  analogWrite(redpin, r);
  analogWrite(greenpin, g);
  analogWrite(bluepin, b);
}

void set_RGBi(int r, int g, int b){
  // Set RGB LED pins based on high=bright 
  set_RGB(255-r, 255-g, 255-b);
}

// diagnosis sync. funcs
void sync_diagnoses() {
  bool waitForSignal = true;

  // if you are bot 5, don't wait to send signal
  if (score == 5) {
    waitForSignal = false;
  }
  // any other bot should wait to send their signal
  while(waitForSignal) {
    if(Serial2.available()) {
      char data = Serial2.read();
      Serial.print("Data received: ");
      Serial.println(data);
      if(data = (char)(score + 96)) {
        waitForSignal = false;
      }
    }
    delay(50);
  }
  // see which diagnose action should be taken
  bool switchFlipped = digitalRead(togglePin);

  delay(500);

  if (switchFlipped) {
    // play two second note
    mySerial.write(214); mySerial.write(216); mySerial.write(227);
    delay(2000);
  } else {
    // Make bright red
    analogWrite(redpin, 0);
    analogWrite(greenpin, 255);
    analogWrite(bluepin, 255);

    delay(2000);

    // Turn all off
    analogWrite(redpin, 255);
    analogWrite(greenpin, 255);
    analogWrite(bluepin, 255);
  }

  delay(1000);

  // broadcast to next bot
  Serial.print("Broadcasting next char: ");
  Serial.print((char)(score + 95));
  Serial2.write((char)(score + 95));
}

void dance(){
  servoLeft.attach(11);
  servoRight.attach(12);
  servoLeft.writeMicroseconds(1300);         // Left wheel counterclockwise
  servoRight.writeMicroseconds(1550);
  delay(500);
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
  delay(100);
  servoLeft.writeMicroseconds(1450);         // Left wheel counterclockwise
  servoRight.writeMicroseconds(1700);
  delay(500);
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
  delay(100);
  servoLeft.writeMicroseconds(1500);         // Left wheel counterclockwise
  servoRight.writeMicroseconds(1700);
  delay(100);
  servoLeft.writeMicroseconds(1500);         // Left wheel counterclockwise
  servoRight.writeMicroseconds(1300);
  delay(100);
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
  delay(50);
  servoLeft.writeMicroseconds(1300);         // Left wheel counterclockwise
  servoRight.writeMicroseconds(1500);
  delay(100);
  servoLeft.writeMicroseconds(1700);
  servoRight.writeMicroseconds(1500);
  delay(100);
  servoLeft.writeMicroseconds(1500);         // Left wheel counterclockwise
  servoRight.writeMicroseconds(1500);
  delay(50);

  servoLeft.writeMicroseconds(1500);         // Left wheel counterclockwise
  servoRight.writeMicroseconds(1700);
  delay(2000);
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
  delay(50);
  servoLeft.writeMicroseconds(1500);         // Left wheel counterclockwise
  servoRight.writeMicroseconds(1300);
  delay(2000);
  servoLeft.detach();                        // Stop sending servo signals
  servoRight.detach(); 
}

void anthem() {
 
  #define num 26
 
  int durs[num] = {211, 210, 212, 212, 212,
  213, 211, 210, 212, 212,
  212, 213, 211, 211, 212, 211,
  211, 212, 213, 211, 210,
  212, 212, 212, 212, 212
  };
  int octs[num] = {216, 216, 216, 216, 216,
  217, 217, 217, 217, 216,
  216, 216, 216, 216, 217, 217,
  217, 217, 217, 217, 216,
  217, 217, 216, 216, 216
  };
  int notes[num] = {230, 227, 223, 227, 230,
  223, 227, 225, 223, 227,
  229, 230, 230, 230, 227, 232,
  225, 223, 222, 220, 222,
  223, 223, 230, 227, 223
  };
 
  for(long k=0; k<num; k++){
    mySerial.write(durs[k]); mySerial.write(octs[k]); mySerial.write(notes[k]);
    int len = 214 - durs[k];
    float del = 2000 / pow(2, len);
    delay(int(del*1.1));
  }
}

void lightShow(){
    mySerial.write(217);
    mySerial.write(212);
  for (int i=0; i<8; i++) {
    set_RGBi(255, 0,0);
    mySerial.write(229);
    delay(500);
    set_RGBi(0, 0,255);
    mySerial.write(223);
    delay(500);
  }
  
 }

// Code to print task selected
const char* taskToString(int task) {
    switch (task) {
        case 1: 
          return "IR";
        case 2: 
          return "HALL";
        case 3: 
          return "COLOR";
        case 4: 
          return "RFID";
        case 5: 
          return "PING";
        default: 
          return "UNKNOWN TASK";
    }
}

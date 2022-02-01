#include <Arduino.h>
//#include <LiquidCrystal_I2C.h>
#include <LiquidCrystal.h>

//Parameters for timers in seconds
unsigned long GameTime = 600;
unsigned long OnePointTime = 60;
unsigned long CaptureTime = 20;
unsigned long ResetTime = 20;

// constants won't change. They're used here to set pin numbers:
const int blueButtonPin = 8; //the number of the pushbutton pin
const int redButtonPin = 9;  
const int redLedPin =  7; //the number of the LED pin 
const int blueLedPin =  6;
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
//LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
char charToScreen = 1;
//HardwareSerial Serial;

//Main conter comparator
unsigned long millisNow = 0;
//Debounced signal from buttons  
const unsigned int debounceDelay = 20;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
bool blueButtonON, redButtonON = false; // variable for reading the pushbutton status
bool lastButtonState, blueBtReadings, redBtReadings = LOW; 
//Total timer variables
const unsigned long totalTimer = GameTime * 1001;
unsigned long totalTimer_previousMillis = 0;
bool totalTimerON = false;
//How fast points will add to each team
const unsigned long redBlueCounter_TimeInterval = OnePointTime * 1000;
unsigned long redBlueCounter_PreviousMillis = 0;
bool redBlueCounterON, redCounterON, blueCounterON = false; //, blueLedBlinkON, ledRedBlinkON = false;
//Capture and reset timer variables 
const unsigned long captureInterval = CaptureTime * 1000;
const unsigned long resetInterval = ResetTime * 1000;
const unsigned long captureProgress_StepInterval = captureInterval/16;
const unsigned long resetProgress_StepInterval = resetInterval/16;
unsigned long captureProgressMillis, resetProgressMillis = 0;
unsigned long captureProgress_Step, resetProgress_Step = 0;
//Parameter for counter transition 
unsigned int captureProgressCounter, resetProgressCounter = 0;
unsigned int redCounter, blueCounter = 0;
//Waite interval to southe transition 
const unsigned long transitionWaite = 300; 
unsigned long transitionWaiteMillis = 0;
//Wait interval to wait in transition between point Reset and capture begin
const unsigned long screenInfoCounterWaite = 2000; 
unsigned long screenInfoCounterWaiteMillis = 0;
//timer to rest points (red blue blinking)
const unsigned long resetPointsWaiteTimer = 3000;
unsigned long previousMillisResetPoints = 0;
//Timer to get into hidden menu
const unsigned long waiteButtonPressTimer = 6000;
unsigned long previousMillisWaiteButtonPress = 0; 
//Led blink controll variables
unsigned long previousMillisLED, previousMillisLEDBlink = 0; 
unsigned int ledBlinkInterval, ledBlinkSpeed = 0; 
bool ledState = false;
//Testing variables
const unsigned long ledUpdateIntervalOnSerial = 1000;

enum State {Initialize, BlueButtonPressed, RedButtonPressed, ResetBlueCounterState, ResetRedCounterState, 
WaiteStateCounterON, ResetRedBluePoints, RedBlueHiddenMenu};
int lastState, state = Initialize;
bool updateScreenON = false;

void stateMachine ();
void CaptureProgressUpdate();
void ResetProgressUpdate();
void BlueCounter_totalTimerON();
void RedCounter_totalTimerON();
void StopTimer();

//---------------Costom Charecter-----------------
unsigned char customSquare[] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};
unsigned char Heart[] = {
  0b00000,
  0b01010,
  0b11111,
  0b11111,
  0b01110,
  0b00100,
  0b00000,
  0b00000
};
void setup() {
  // initialize the LED pin as an output:
  //debug_init();
  pinMode(blueLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(blueButtonPin, INPUT_PULLUP);
  pinMode(redButtonPin, INPUT_PULLUP);
  //lcd.init();
  //Serial.begin(9600); // open the serial port at 9600 bps:
  lcd.begin(16, 2);                      // initialize the lcd 
  // Print a message to the LCD.
  //lcd.backlight();
  lcd.setCursor(1,0);
  lcd.print("Blue");
  lcd.setCursor(12,0);
  lcd.print("Red");
  lcd.setCursor(2,1);
  lcd.print(blueCounter);
  lcd.setCursor(13,1);
  lcd.print(redCounter);
  lcd.createChar(0, customSquare);
  lcd.createChar(1, Heart);
}
// ------------ LED Update Blink Speed ---------------------------------
void updateLEDSpeed() {
  if (0 == ledBlinkSpeed) { ledBlinkInterval = 0; }
  else if (5 >= ledBlinkSpeed) { ledBlinkInterval = 500; }
  else if (10 >= ledBlinkSpeed) { ledBlinkInterval = 300; }
  else if (14 >= ledBlinkSpeed) { ledBlinkInterval = 150; }
  else if (16 >= ledBlinkSpeed) { ledBlinkInterval = 50; }
  else ledBlinkInterval = 0;
  //updateLEDState();
}
//---------------  Screen update  ----------------------
void UpdateScreen() {
  if (redCounterON){
    digitalWrite(redLedPin, HIGH);
    lcd.setCursor(11,0);
    lcd.write(1);
    lcd.setCursor(15,0);
    lcd.write(1);
  }
   if (blueCounterON){
    digitalWrite(blueLedPin, HIGH);
    lcd.setCursor(0,0);
    lcd.write(1);
    lcd.setCursor(5,0);
    lcd.write(1);
  }
  if (!blueCounterON){
    digitalWrite(blueLedPin, LOW);
    lcd.setCursor(0,0);
    lcd.print(" ");
    lcd.setCursor(5,0);
    lcd.print(" ");
  }
  if (!redCounterON){
    digitalWrite(redLedPin, LOW);
    lcd.setCursor(11,0);
    lcd.print(" ");
    lcd.setCursor(15,0);
    lcd.print(" ");
  }
  lcd.setCursor(0,1);
  lcd.print("                ");
  lcd.setCursor(2,1);
  lcd.print(blueCounter);
  lcd.setCursor(13,1);
  lcd.print(redCounter);
  }

void loop() {
  millisNow = millis();

  //------------ Debouncer -----------
  blueBtReadings = !digitalRead(blueButtonPin);
  redBtReadings = !digitalRead(redButtonPin);
  if (blueBtReadings != lastButtonState && redBtReadings != lastButtonState) lastDebounceTime = millisNow;
  if ((millisNow - lastDebounceTime) > debounceDelay) {
    if (blueBtReadings != blueButtonON) blueButtonON = blueBtReadings;
    if (redBtReadings != redButtonON) redButtonON = redBtReadings;
  }
  lastButtonState = redBtReadings;
  lastButtonState = blueBtReadings;
    //Serial.print("state: ");
    //Serial.println(ledON);
    //Serial.println(state);
  //LED update
  /*if (millisNow - previousMillisLED >= ledUpdateIntervalOnSerial) {
    previousMillisLED = millisNow;
    //updateLEDState();
    Serial.print("state: ");
    //Serial.println(ledON);
    Serial.println(state);
    Serial.print("bluebutton: ");
    Serial.println(blueButtonON);
    Serial.print("captureProgress_Step: ");
    Serial.println(captureProgress_Step );
    Serial.print("captureProgressMillis: ");
    Serial.println(captureProgressMillis);
    Serial.print("millisNow: ");
    Serial.println(millisNow);
    Serial.print("ledBlinkSpeed: ");
    Serial.println(ledBlinkSpeed);
    Serial.print("resetProgressCounter: ");
    Serial.println(resetProgressCounter);
    Serial.print("captureProgressCounter: ");
    Serial.println(captureProgressCounter);
  }*/
  //----TotalTimer if totalTimerON if statemnt will be true when totalTimer expires------
  if (totalTimerON && millisNow - totalTimer_previousMillis >= totalTimer) {
    redBlueCounterON = false;
    redCounterON = false;
    blueCounterON = false;
    ledBlinkSpeed = 0;
    digitalWrite(redLedPin, LOW);
    digitalWrite(blueLedPin, LOW);
    totalTimerON = false;
    state = Initialize;
    UpdateScreen();
  }
  //----------Red or Blue counter on -----------
  if (redBlueCounterON && (millisNow - redBlueCounter_PreviousMillis >= redBlueCounter_TimeInterval)) {
    redBlueCounter_PreviousMillis = millisNow;
    if (redCounterON) {
      redCounter ++;
    }
    if (blueCounterON) {
      blueCounter ++;
    }
    UpdateScreen();
  }
   if (redButtonON && blueButtonON && !(state == RedBlueHiddenMenu || state == ResetRedBluePoints)){
      lastState = state;
      state = RedBlueHiddenMenu;
      previousMillisWaiteButtonPress = millisNow;
      ledBlinkSpeed = 1;
      updateLEDSpeed();
      UpdateScreen();
      resetProgressCounter = 0;
      captureProgressCounter = 0;
      digitalWrite(redLedPin, LOW);
      digitalWrite(blueLedPin, LOW);
      updateScreenON = true;
    }
  // LED Blink Controll
   /*
  if (blueLedBlinkON){
    if (millisNow - previousMillisLEDBlink >= ledBlinkInterval) {
      previousMillisLEDBlink = millisNow;
      if (ledState == LOW)ledState = HIGH;
      else ledState = LOW;
      digitalWrite(blueLedPin, ledState);
    }
  }
  if (ledRedBlinkON){
    if (millisNow - previousMillisLEDBlink >= ledBlinkInterval) {
      previousMillisLEDBlink = millisNow;
      if (ledState == LOW)ledState = HIGH;
      else ledState = LOW;
      digitalWrite(redLedPin, ledState);
    }
  }
  */
 stateMachine ();
}

void stateMachine (){
  switch(state){
    case Initialize: 

      if (blueButtonON && !redButtonON){
        state = BlueButtonPressed;
        captureProgressMillis = millisNow;
        break;
      }
      if (redButtonON && !blueButtonON){
        state = RedButtonPressed;
        captureProgressMillis = millisNow;
        break;
      }
    break;
    case RedBlueHiddenMenu:
      if (millisNow - previousMillisWaiteButtonPress >= waiteButtonPressTimer) {
        previousMillisWaiteButtonPress = millisNow;
        if (redButtonON && blueButtonON)
          previousMillisResetPoints = millisNow;
          state = ResetRedBluePoints;
      }
      if (!redButtonON && !blueButtonON){
       state = lastState;
     }
    break;
    case ResetRedBluePoints:
    if (millisNow - previousMillisLEDBlink >= ledBlinkInterval) {
        previousMillisLEDBlink = millisNow;
        if (ledState == LOW)ledState = HIGH;
        else ledState = LOW;
        digitalWrite(blueLedPin, ledState);
        digitalWrite(redLedPin, !ledState);
      }
     if (millisNow - previousMillisResetPoints >= resetPointsWaiteTimer) {
        redBlueCounterON = false;
        redCounterON = false;
        blueCounterON = false;
        digitalWrite(redLedPin, LOW);
        digitalWrite(blueLedPin, LOW);
        blueCounter = 0;
        redCounter = 0;
        UpdateScreen();
        ledBlinkSpeed = 0;
        updateLEDSpeed();
        state = Initialize;
     }
      if (!redButtonON && !blueButtonON){
       state = lastState;
     }
    break;
    case BlueButtonPressed:
      //-------Led Blincking controll------
      if (millisNow - previousMillisLEDBlink >= ledBlinkInterval && !blueCounterON) {
        previousMillisLEDBlink = millisNow;
        if (ledState == LOW)ledState = HIGH;
        else ledState = LOW;
        digitalWrite(blueLedPin, ledState);
      }
      //Update a bar on screen
      if (captureProgressCounter < 17){ CaptureProgressUpdate(); }
      //Start totalTimer and bluecounter
      if (captureProgressCounter == 17 && (millisNow - transitionWaiteMillis >= transitionWaite)){ BlueCounter_totalTimerON();}
      //when blue button low transition to bluecounter state 
      if (captureProgressCounter == 18 && ((millisNow -  screenInfoCounterWaiteMillis >= screenInfoCounterWaite) || !blueButtonON)){
        state = WaiteStateCounterON;
        captureProgress_Step = 0;
        ledBlinkSpeed = 0;
        captureProgressCounter = 0;
        UpdateScreen();
      }
      //Reset to initialize if blue button low before time
      if (!blueButtonON && captureProgressCounter < 17 && !blueCounterON) {
        state = Initialize;
        captureProgress_Step = 0;
        captureProgressCounter = 0;
        ledBlinkSpeed = 0;
        updateLEDSpeed();
        UpdateScreen();
        digitalWrite(blueLedPin, LOW);
      }
    break;
    case RedButtonPressed:
      //-------Led Blincking controll------
      if (millisNow - previousMillisLEDBlink >= ledBlinkInterval && !redCounterON) {
        previousMillisLEDBlink = millisNow;
        if (ledState == LOW)ledState = HIGH;
        else ledState = LOW;
        digitalWrite(redLedPin, ledState);
      }
      //Update a bar on screen
      if (captureProgressCounter < 17){ CaptureProgressUpdate(); }
      //Start totalTimer and bluecounter
      if (captureProgressCounter == 17 && (millisNow - transitionWaiteMillis >= transitionWaite)){ RedCounter_totalTimerON();}
      //when red button low transition to bluecounter state 
      if (captureProgressCounter == 18 && ((millisNow - screenInfoCounterWaiteMillis >= screenInfoCounterWaite) || !redButtonON)){
        state = WaiteStateCounterON;
        captureProgress_Step = 0;
        ledBlinkSpeed = 0;
        captureProgressCounter = 0;
        UpdateScreen();
      }
      //Reset to initialize if blue button low before time
      if (!redButtonON && captureProgressCounter < 17 && !redCounterON) {
        state = Initialize;
        captureProgress_Step = 0;
        captureProgressCounter = 0;
        ledBlinkSpeed = 0;
        updateLEDSpeed();
        UpdateScreen();
        digitalWrite(redLedPin, LOW);
      }
    break;
    case WaiteStateCounterON:
      if (updateScreenON){ UpdateScreen(); updateScreenON = false;}
      if(redButtonON && !redCounterON){
        ledBlinkSpeed = 0;
        resetProgressCounter = 0;
        resetProgress_Step = 0;
        resetProgressMillis = millisNow;
        state = ResetRedCounterState;
        break;
      }
      if(blueButtonON && !blueCounterON){
        ledBlinkSpeed = 0;
        resetProgressCounter = 0;
        resetProgress_Step = 0;
        resetProgressMillis = millisNow;
        state = ResetBlueCounterState;
        break;
      }
    break;
    case ResetBlueCounterState:
      if (resetProgressCounter == 0 &&  !(resetProgressCounter > 1)){ 
        lcd.setCursor(0,1);
        lcd.print("  Red reset ON ");
        //transitionWaiteMillis = millisNow;
      }
      if (resetProgressCounter < 17){ ResetProgressUpdate ();}
      if (resetProgressCounter == 17 && (millisNow - transitionWaiteMillis >= transitionWaite)) {
        StopTimer();
        ledBlinkSpeed = 0;
        resetProgressCounter = 18;
        resetProgress_Step = 0;
        screenInfoCounterWaiteMillis = millisNow;
      }
      if (resetProgressCounter == 18 && ((millisNow - screenInfoCounterWaiteMillis >= screenInfoCounterWaite) || !blueButtonON)) {
        state = Initialize;
        resetProgressCounter = 0;
        UpdateScreen();
      }
      if (!blueButtonON && resetProgressCounter < 17 && state == ResetBlueCounterState ){
        state = WaiteStateCounterON;
        resetProgressCounter = 0;
        UpdateScreen();
      }
    break;
    case ResetRedCounterState:
      if (resetProgressCounter == 0 &&  !(resetProgressCounter > 1)){ 
        lcd.setCursor(0,1);
        lcd.print("  Blue reset ON ");
        //transitionWaiteMillis = millisNow;
      }
      if ( resetProgressCounter < 17){ 
        ResetProgressUpdate ();
        }
      if (resetProgressCounter == 17 && (millisNow - transitionWaiteMillis >= transitionWaite)) {
        StopTimer();
        screenInfoCounterWaiteMillis = millisNow;
        ledBlinkSpeed = 0;
        resetProgressCounter = 18;
        resetProgress_Step = 0;
      }
      if (resetProgressCounter == 18 && ((millisNow - screenInfoCounterWaiteMillis >= screenInfoCounterWaite) || !redButtonON)) {
        state = Initialize;
        resetProgressCounter = 0;
        UpdateScreen();
      }
      if (!redButtonON && resetProgressCounter < 17  && state == ResetRedCounterState){
        state = WaiteStateCounterON;
        resetProgressCounter = 0;
        UpdateScreen();
      }
    break;
  }
}
/*  _____            _                    _____                                     _    _           _       _       
  / ____|          | |                  |  __ \                                   | |  | |         | |     | |      
 | |     __ _ _ __ | |_ _   _ _ __ ___  | |__) | __ ___   __ _ _ __ ___  ___ ___  | |  | |_ __   __| | __ _| |_ ___ 
 | |    / _` | '_ \| __| | | | '__/ _ \ |  ___/ '__/ _ \ / _` | '__/ _ \/ __/ __| | |  | | '_ \ / _` |/ _` | __/ _ \
 | |___| (_| | |_) | |_| |_| | | |  __/ | |   | | | (_) | (_| | | |  __/\__ \__ \ | |__| | |_) | (_| | (_| | ||  __/
  \_____\__,_| .__/ \__|\__,_|_|  \___| |_|   |_|  \___/ \__, |_|  \___||___/___/  \____/| .__/ \__,_|\__,_|\__\___|
             | |                                          __/ |                          | |                        
             |_|                                         |___/                           |_|                       */
//Progress bar controll loop for updating bar on the screen
void CaptureProgressUpdate() {
  if (millisNow - captureProgressMillis >= captureProgress_Step) {
    lcd.setCursor(captureProgressCounter,1);
    lcd.write(1);
    //lcd.print(‭65535‬, DEC);
    captureProgress_Step = captureProgress_StepInterval + captureProgress_Step;
    captureProgressCounter ++;
    ledBlinkSpeed ++;
    transitionWaiteMillis = millisNow;
    updateLEDSpeed();
  }
}

void ResetProgressUpdate () {
  if (millisNow - resetProgressMillis >= resetProgress_Step){
    if (1 == resetProgressCounter){
      lcd.setCursor(0,1);
      lcd.write(1);
      //lcd.print(1111, HEX);
      lcd.setCursor(1,1);
      lcd.write(1);
      //lcd.print(0);
      //lcd.setCursor(2,1);
      //lcd.write(0);
    }
    if (1 < resetProgressCounter) {
      lcd.setCursor(resetProgressCounter,1);
      lcd.write(1);
      //lcd.print(0);
    }
    //lcd.print(0);
    resetProgress_Step = resetProgress_StepInterval + resetProgress_Step;
    resetProgressCounter ++;
    ledBlinkSpeed ++;
    transitionWaiteMillis = millisNow;
    updateLEDSpeed();
  }
}
//Update the screen when blue is captured start totalTimer and blueCounter turn on blueLed 
void BlueCounter_totalTimerON(){
  totalTimer_previousMillis = millisNow;
  totalTimerON = true;
  captureProgressCounter = 18;
  lcd.setCursor(0,1);
  lcd.print("  Blue captured  ");
  digitalWrite(blueLedPin, HIGH);
  digitalWrite(redLedPin, LOW);
  redBlueCounter_PreviousMillis = millisNow;
  screenInfoCounterWaiteMillis = millisNow;
  redBlueCounterON = true;
  blueCounterON = true;
}
void RedCounter_totalTimerON(){
  totalTimer_previousMillis = millisNow;
  totalTimerON = true;
  captureProgressCounter = 18;
  lcd.setCursor(0,1);
  lcd.print("  Red captured  ");
  digitalWrite(blueLedPin, LOW);
  digitalWrite(redLedPin, HIGH);
  redBlueCounter_PreviousMillis = millisNow;
  screenInfoCounterWaiteMillis = millisNow;
  redBlueCounterON = true;
  redCounterON = true;
}
 //---------------STOP Timer--------------------
void StopTimer(){
    lcd.setCursor(0,1);
    lcd.print("PointNeutralized");
    redBlueCounterON = false;
    redCounterON = false;
    blueCounterON = false;
    ledBlinkSpeed = 0;
    digitalWrite(redLedPin, LOW);
    digitalWrite(blueLedPin, LOW);
}
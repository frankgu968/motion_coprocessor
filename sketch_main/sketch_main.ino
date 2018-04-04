#define TIMEOUT 100 //time out value for serial reads in ms
#define NUM_CMDS 7  //used for the commandList -- allows for new commands to be added in the future (comm protocol may need expansion)

#define DEBUG 0      //enables trace statments
#define ZERO_CALIBRATE 0 //for manually calibrating the 0 position
#define DO_BOUNDS_CHECK 1

#define RETURN_TO_ZERO 1 
#define USING_MODE_SELECT 1
#define AUTOMATIC_MODE 1
#define MANUAL_MODE 0
#define FORCE_TEST 1
#define THRESH_CHECK 1

#if DEBUG 
  #define TRACE(x) Serial.print(x);  
  #define TRACEN(x) Serial.println(x);          
#else
  #define TRACE(x) {};
  #define TRACEN(x) {}; 
#endif

#include <EEPROM.h>
#include <TimerOne.h>   // http://www.arduino.cc/playground/Code/Timer1
#include <Q2HX711.h>

#define STEP_PWM_PIN 10

#define X_DIR 7
#define X_ENA 4 
#define Y_DIR 6 
#define Y_ENA 3 
#define Z_DIR 5 
#define Z_ENA 8 

#define greenLED A0
#define yellowLED 13
#define redLED 12
#define whiteLED 9

#define STEP_PIN A2
#define RST_PIN 2
#define MODE_PIN A1

#define X_ADDR 0
#define Y_ADDR 4
#define Z_ADDR 8

#define HX711_CLK_PIN   A5
#define  HX711_DATA_PIN A4

byte mode = AUTOMATIC_MODE;
boolean stopTriggered = false;
boolean engageEnabled = false;

class commandFormat{
  public:
    String command;             //3 character name of the command (e.g. MOV, STP, RST, ENG, DNG) 
    byte numArgs;               //currently support 255 data arguments
    boolean allowedDataFormat;  //specifies if the data arguments should be populated through the use of bools (true) or empty (false)
};

commandFormat commandList[NUM_CMDS];

void setUpCommandList();
void fillCommandStruct(int index, String commandName, byte numDataArgs, boolean fillData);
void EEPROMWritelong(int address, long value);
long EEPROMReadlong(long address);
void cycleLEDs();
void turnOffLEDs();
void disableAllMotors();
void idleLED();
void pendingLED();
void errLED();
void saveCurPos();
void returnToZero();


Q2HX711 hx711(HX711_DATA_PIN, HX711_CLK_PIN);

void setup() {
  Serial.begin(9600);
 
  setUpCommandList();  

  pinMode (STEP_PWM_PIN,OUTPUT);
  pinMode (X_DIR, OUTPUT);  
  pinMode (X_ENA, OUTPUT);
  pinMode (Y_DIR, OUTPUT);  
  pinMode (Y_ENA, OUTPUT);  
  pinMode (Z_DIR, OUTPUT);  
  pinMode (Z_ENA, OUTPUT);

  pinMode(greenLED,OUTPUT);
  pinMode(yellowLED,OUTPUT);
  pinMode(redLED,OUTPUT); 
  pinMode(whiteLED,OUTPUT);
 
  disableAllMotors();    
  
  #if ZERO_CALIBRATE  
  
    TRACEN("Calibrating EEPROM for this position to be the origin");
    TRACEN("WARNING: CHANGE THE X, Y, AND Z BOUNDS IF THIS IS A DIFFERENT ORIGIN THAN BEFORE!");
    EEPROMWritelong(X_ADDR,0);
    EEPROMWritelong(Y_ADDR,0);
    EEPROMWritelong(Z_ADDR,0);
    
  #endif
  
  engageEnabled = false;
  digitalWrite(whiteLED,LOW);
  turnOffLEDs();
  cycleLEDs();
    
  #if USING_MODE_SELECT

    digitalWrite(greenLED,HIGH);
    attachInterrupt(digitalPinToInterrupt(RST_PIN),STOP_ISR,FALLING);
    pinMode(STEP_PIN,INPUT);
    pinMode(RST_PIN,INPUT);
    boolean modeSelPressed = false; 
    
    while(!modeSelPressed){
      delay(10);     
      toggleLEDs();
      if(digitalRead(STEP_PIN) == LOW){
        modeSelPressed = true;      
        if(digitalRead(MODE_PIN) == LOW){
          mode = AUTOMATIC_MODE;
          turnOffLEDs();
          delay(250);
          digitalWrite(yellowLED,HIGH);
          delay(250);
          digitalWrite(yellowLED,LOW); 
          delay(250);
          digitalWrite(yellowLED,HIGH);
          delay(250);
          digitalWrite(yellowLED,LOW);   
          delay(250);
          digitalWrite(yellowLED,HIGH);
          delay(250);
          digitalWrite(yellowLED,LOW);     
          idleLED();    
          TRACEN("Mode selected to be automatic");
        } else {
          mode = MANUAL_MODE;
          turnOffLEDs();
          delay(250);
          digitalWrite(greenLED,HIGH);
          delay(250);
          digitalWrite(greenLED,LOW); 
          delay(250);
          digitalWrite(greenLED,HIGH);
          delay(250);
          digitalWrite(greenLED,LOW); 
          delay(250);
          digitalWrite(greenLED,HIGH);
          
          idleLED();           
          TRACEN("Mode selected to be manual");
        }

        //init is in here rather than setup because you only want the PWM occuring after you have selected a mode
        //this will prevent random motor movement if the USB cable is removed when the system is in an idle state
        Timer1.initialize(240); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
        Timer1.attachInterrupt( timerIsr ); // attach the service routine here
        Timer1.pwm(STEP_PWM_PIN,512,240);  
      }      
    }
    
  #endif
}

void loop() {  
  parseEvent();
}

void setUpCommandList(){
  //to modify the command data field length, change the function calls to fill command struct
  //to add new commands, simply add a new call to fillCommandStruct() with the appropriate params (see comment section below)
  //IMPORTANT: to add new commands, you will also need to increment the NUM_CMD define at the top of this file. 
  
  fillCommandStruct(0,"MOV",3,true);
  fillCommandStruct(1,"STP",0,false);
  fillCommandStruct(2,"RST",0,false);
  fillCommandStruct(3,"ENG",0,false);
  fillCommandStruct(4,"DNG",0,false);
  fillCommandStruct(5,"LON",0,false);
  fillCommandStruct(6,"LOF",0,false);
  
  //user to add commands here (format provided): 
  /*fillCommandStruct(7,[STING NAME OF YOUR COMMAND],[NUM ARGS OF YOUR COMMAND],[ARE YOUR ARG FIELDS POPULATED WITH FLOATS]); */ 
}

void fillCommandStruct(int index, String commandName, byte numDataArgs, boolean fillData){
  commandList[index].command = commandName;  
  commandList[index].numArgs = numDataArgs;
  commandList[index].allowedDataFormat = fillData;
}

void STOP_ISR(){
  TRACEN("STOP ISR triggered");
  idleLED();
  if(digitalRead(RST_PIN) == LOW){  
    if(!stopTriggered){
      stopTriggered = true;
      if(RETURN_TO_ZERO){
        returnToZero();
      }      
    }
  }
}

void turnOffLEDs(){
  digitalWrite(redLED,LOW);
  digitalWrite(yellowLED,LOW);
  digitalWrite(greenLED,LOW);  
  delay(10);
}

void turnOnLEDs(){
  digitalWrite(redLED,HIGH);
  digitalWrite(yellowLED,HIGH);
  digitalWrite(greenLED,HIGH);  
  delay(10);
}

void toggleLEDs(){
  turnOnLEDs();
  delay(200);
  turnOffLEDs();
  delay(200);  
}

void cycleLEDs(){
  delay(250);
  digitalWrite(greenLED,HIGH);
  delay(250);
  digitalWrite(greenLED,LOW);  
  
  digitalWrite(yellowLED,HIGH);
  delay(250);
  digitalWrite(yellowLED,LOW);
  
  digitalWrite(redLED,HIGH);
  delay(250);
  digitalWrite(redLED,LOW);
  
  digitalWrite(yellowLED,HIGH);
  delay(250);
  digitalWrite(yellowLED,LOW);
  
  digitalWrite(greenLED,HIGH);
  delay(250);
  digitalWrite(greenLED,LOW);     
}

void errLED(){
  delay(10);
  digitalWrite(greenLED,LOW);
  digitalWrite(yellowLED,LOW);
  digitalWrite(redLED,HIGH); 
}

void pendingLED(){
  delay(10);
  digitalWrite(greenLED,LOW);
  digitalWrite(yellowLED,HIGH);
  digitalWrite(redLED,LOW); 
}

void idleLED(){
  delay(10);
  digitalWrite(greenLED,HIGH);
  digitalWrite(yellowLED,LOW);
  digitalWrite(redLED,LOW); 
}




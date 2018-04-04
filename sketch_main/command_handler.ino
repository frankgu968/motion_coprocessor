#if DEBUG 
#define TRACE(x) Serial.print(x);  
#define TRACEN(x) Serial.println(x);              
#else
#define TRACE(x) {};
#define TRACEN(x) {}; 
#endif

#if FORCE_TEST
  #define TRACEF(x) Serial.print(x);
  #define TRACEFN(x) Serial.println(x);
#else
  #define TRACEF(x) {};
  #define TRACEFN(x) {};
#endif

#define X_BOUND_MAX 140  * STEPS_PER_MM //- 10 - 10 (5cm per side buffer)
#define X_BOUND_MIN -140 * STEPS_PER_MM //- 10 - 10 (5cm per side buffer)

#define Y_BOUND_MAX 380  * STEPS_PER_MM //53.5 - 10 - 10cm (5cm per side buffer)
#define Y_BOUND_MIN 0                   //53.5 - 10 - 10cm (5cm per side buffer)

#define Z_BOUND_MAX 35   * STEPS_PER_MM //14 - 6 - 2cm
#define Z_BOUND_MIN -35  * STEPS_PER_MM 

#define FORCE_THRESH1     90000
#define FORCE_THRESH2     100000
#define FORCE_THRESH3     110000
#define FORCE_THRESH_MAX  120000

#define FORCE_STEADY        83886
#define FORCE_ERROR_THRESH  0.01
#define UPPER_ERROR (1+FORCE_ERROR_THRESH)
#define LOWER_ERROR (1-FORCE_ERROR_THRESH)
#define FORCE_STEADY_UPPER  FORCE_STEADY * UPPER_ERROR
#define FORCE_STEADY_LOWER  FORCE_STEADY * LOWER_ERROR
#define PROFILE_SIZE 100

#define EEPROM_WRITE_CYCLES 5000
#define ENGAGE_FORCE_CHECK_DIST 0.5

#define STEPS_PER_MM 318.5

byte commandHandler(String cmd, float dat[], int datSize);
char handleMOV(float dat[], int datSize);
void moveMotor(char dir, float dist);
boolean boundsCheck(float x, float y, float z);
char handleSTP();
char handleRST();
char handleENG();
byte engUntilThreshold(long threshold, long threshold_max);
long read_average_force(byte times) ;
char handleDNG();

boolean enabled_X = false;
boolean enabled_Y = false;
boolean enabled_Z = false;

long x_pos = 0;
long y_pos = 0;
long z_pos = 0;

long x_dirVec = 0;
long y_dirVec = 0;
long z_dirVec = 0;

long x_thresh = 0;
long y_thresh = 0;
long z_thresh = 0;

long x_counter = 0;
long y_counter = 0;
long z_counter = 0;

boolean x_done = false;
boolean y_done = false;
boolean z_done = false;

boolean move_x_Done = false;
boolean move_y_Done = false;
boolean move_z_Done = false;

boolean x_started = false;
boolean y_started = false;
boolean z_started = false;

int profileIndex = 0;


const long forceProfile [PROFILE_SIZE] = {85864,85864,85864,85864,85864,85864,85864,85864,85864,85864,
                                          85864,85864,85864,85864,85864,85864,85864,85864,85864,85864,
                                          85864,85864,85864,85864,85864,85864,85864,85864,85864,85864,
                                          85864,85864,85864,85864,85864,85864,85864,85864,85864,85864,
                                          85864,85864,85864,85864,85864,85864,85864,85864,85864,85864,
                                          85864,85864,85864,85864,85864,85864,85864,85864,85864,85864,
                                          85864,85864,85864,85864,85864,85864,85864,85864,85864,85864,
                                          85864,85864,85864,85864,85864,85864,85864,85864,85864,85864,
                                          85864,85864,85864,85864,85864,85864,85864,85864,85864,85864,
                                          85864,85864,85864,85864,85864,85864,85864,85864,85864,85864};


byte commandHandler(String cmd, float dat[], int datSize){
  int eventCode = 3;
  idleLED(); 
  if (cmd == "MOV"){
    eventCode = handleMOV(dat, datSize);
  }
  else if (cmd == "STP"){
    eventCode = handleSTP();
  }    
  else if (cmd == "RST"){
    eventCode = handleRST();
  }
  else if (cmd == "ENG"){
    eventCode = handleENG();
  } 
  else if (cmd == "DNG"){
    eventCode = handleDNG();
  } 
  else if (cmd == "LON"){
    TRACEN("turning white LED on");
    digitalWrite(whiteLED,HIGH);
    eventCode = 150;
  } 
  else if (cmd == "LOF"){
     TRACEN("turning white LED off");
    digitalWrite(whiteLED,LOW);
    eventCode = 160;
  }
  else {
    TRACE("command is ");
    TRACEN(cmd);   
    eventCode = 3;
  }   
  return eventCode;
}

char handleMOV(float dat[], int datSize){  
  int eventResult = 100;
  float x_dist = dat[0];
  float y_dist = dat[1];
  float z_dist = dat[2];
  
  if(boundsCheck(x_dist,y_dist,z_dist) == true){
    if(x_dist != 0){
      moveMotor('X',x_dist);
    }
    if(y_dist != 0){
      moveMotor('Y',y_dist);
    }
    if(z_dist != 0){
      moveMotor('Z',z_dist);
    }    
  }
  else{
    eventResult = (unsigned int)-1;  
  }
  return eventResult;
}

boolean boundsCheck(float x, float y, float z){
  
  boolean retStat = true;
  if(x == 0 && y == 0 && z == 0){
    retStat = false;
    TRACEN("error: need to specify a non-zero distance vector");
  }  
  else{
  #if DO_BOUNDS_CHECK
  
    long cur_x_pos = EEPROMReadlong(X_ADDR);
    delay(10);
    long cur_y_pos = EEPROMReadlong(Y_ADDR);
    delay(10);
    long cur_z_pos = EEPROMReadlong(Z_ADDR);
    delay(10);
  
    long proposedPosX = cur_x_pos + (x*STEPS_PER_MM);
    long proposedPosY = cur_y_pos + (y*STEPS_PER_MM);
    long proposedPosZ = cur_z_pos + (z*STEPS_PER_MM);  
      
    if(proposedPosX < X_BOUND_MIN || proposedPosX > X_BOUND_MAX){
      retStat = false;
      TRACE("input value is over the X bound! proposed: ");
      TRACE(proposedPosX);
      TRACE(" cur: ");   
      TRACEN(cur_x_pos);   
    }
    if(proposedPosY < Y_BOUND_MIN || proposedPosY > Y_BOUND_MAX){      
      retStat = false;
      TRACE("input value is over the Y bound! proposed: ");      
      TRACE(proposedPosY);
      TRACE(" cur: ");  
      TRACEN(cur_y_pos);  
    }
    if(proposedPosZ < Z_BOUND_MIN || proposedPosZ > Z_BOUND_MAX){      
      retStat = false;
      TRACE("input value is over the Z bound! proposed: ");
      TRACE(proposedPosZ);
      TRACE(" cur: ");
      TRACEN(cur_z_pos);  
    }
    
  #endif
  }
  return retStat;  
}

void moveMotor(char dir, float dist){
  int DIR_PIN, ENA_PIN, ADDR;   
  
  if(dir == 'X'){
     DIR_PIN = X_DIR;
     ENA_PIN = X_ENA;
     ADDR = X_ADDR;
     TRACEN("X dir specified");
     x_pos = EEPROMReadlong(X_ADDR);
     x_dirVec = (dist > 0) ? 1 : -1;
     x_thresh = (long)(dist * STEPS_PER_MM * x_dirVec);
     enabled_X = true;
     x_started = true;      
  }
  else if (dir == 'Y'){    
    DIR_PIN = Y_DIR;
    ENA_PIN = Y_ENA;
    ADDR = Y_ADDR;
    TRACEN("Y dir specified");
    y_pos = EEPROMReadlong(Y_ADDR);
    y_dirVec = (dist > 0) ? 1 : -1;
    y_thresh = (long)(dist * STEPS_PER_MM * y_dirVec); 
    enabled_Y = true;
    y_started = true;   
  }
  else if (dir == 'Z'){
    DIR_PIN = Z_DIR;
    ENA_PIN = Z_ENA;
    ADDR = Z_ADDR;
    TRACEN("Z dir specified");
    z_pos = EEPROMReadlong(Z_ADDR);
    z_dirVec = (dist > 0) ? 1 : -1;
    z_thresh = (long)(dist * STEPS_PER_MM * z_dirVec);
    enabled_Z = true;
    z_started = true;
    
  }
  else{
    TRACE("moveMotors error: invalid direction specified");
  }

  //determine magnitude and direction
  int dirVal = LOW;
    
  if(dist < 0){
    if(dir == 'Y'){
      dirVal = LOW;
    }
    else{
      dirVal = HIGH;
    }    
  }
  else if (dist > 0){
    if(dir == 'Y'){
      dirVal = HIGH;      
    }
    else{
      dirVal = LOW;
    }
  }
  else{
    TRACE("moveMotors error: 0 distance specified");
    return;
  }
  
  digitalWrite(DIR_PIN,dirVal);
  digitalWrite(ENA_PIN,LOW);
  TRACE(dirVal);
  TRACE(" ");
  TRACEN(dist);

}

char handleSTP(){
  TRACEN("handle STP");  
  profileIndex = 0;
  Timer1.detachInterrupt();
  disableAllMotors();   
  saveCurPos();    
  Timer1.attachInterrupt( timerIsr );
  int eventResult = 110;
  return eventResult;
}

char handleRST(){
  TRACEN("handle RST");
  if(!stopTriggered){
    stopTriggered = true;
    if(RETURN_TO_ZERO){
      returnToZero();
    }      
  }    
  int eventResult = 120;
  return eventResult;
}

char handleENG(){  
  
  int eventResult = 130;
  byte retVal = 0;  
  
  disableAllMotors(); 
  engageEnabled = true;
  
  delay(5);
  y_pos = EEPROMReadlong(Y_ADDR);     
  delay(5);
  
  TRACEN("engaging until threshold");
  retVal = engUntilThreshold();  
  if(retVal == 100){
    return retVal;
  }
  else{
    TRACEN("threshold1 engage max dist reached");
    return 255;
  }    
}

byte engUntilThreshold(){
  
  boolean threshReached = false;
  long forceVal = 0;
  byte retVal = 0;
  
  moveMotor('Y', (Y_BOUND_MAX - y_pos)/STEPS_PER_MM);
      
  while(!threshReached && !stopTriggered){
      forceVal = read_average_force(5);
      TRACEF(millis());
      TRACEF(",");
      TRACEFN(forceVal);
      
      #if THRESH_CHECK
        if(forceVal < FORCE_STEADY_UPPER && forceVal > FORCE_STEADY_LOWER){
          
        } else if (forceVal < forceProfile[1]*UPPER_ERROR &&
                   forceVal > forceProfile[1]*LOWER_ERROR) {
//          profileIndex++;
//          if(profileIndex == PROFILE_SIZE){
//            //we have reached the end of the profile and are done engagement. return true
//            handleSTP();
//            saveCurPos();
//            threshReached = true;
//            retVal = 100;      
//          }
        } else {
          //break or error - there is no force profile
          TRACEN("force outside of profile");          
          handleSTP();
          //saveCurPos();
          threshReached = true;
          Serial.print("{CPT}");
          Serial.print("\n");
          retVal = 100;        
        } 
      #endif
    if(y_pos + y_counter >= Y_BOUND_MAX){
      handleSTP();
      threshReached = true;
      retVal = 0;
    } 
  }      
   
  if(stopTriggered){
    retVal = 100;    
  }
  return retVal;  
}

long read_average_force(byte times) {
  long sum = 0;
  for (byte i = 0; i < times; i++) { 
    sum += hx711.read();    
  }
  return (sum / times)/100.0;
}

char handleDNG(){
  int eventResult = 140;
  return eventResult;
}

void returnToZero(){
  long curXpos, curYpos, curZpos; 
  boolean wasEngageEnabled = engageEnabled;
  disableAllMotors(); 
  saveCurPos();

  delay(5);  
  curXpos = EEPROMReadlong(X_ADDR);
  delay(5);  
  curYpos = EEPROMReadlong(Y_ADDR);  
  delay(5);
  curZpos = EEPROMReadlong(Z_ADDR);

  if(curYpos != 0){
    //moveMotor('Y', (curYpos * -1.0)/STEPS_PER_MM);
    
    TRACE("moving Y back by ");
    TRACE(curYpos);
    TRACEN("steps");
    
    Timer1.disablePwm(STEP_PWM_PIN); 
    Timer1.detachInterrupt();
    digitalWrite(Y_DIR,LOW);
    digitalWrite(Y_ENA,LOW);
    delay(5);
    for(long i = 0; i < curYpos; i++){
      digitalWrite(STEP_PWM_PIN,HIGH);
      delayMicroseconds(120);
      digitalWrite(STEP_PWM_PIN,LOW);
      delayMicroseconds(120);
    }
    digitalWrite(Y_ENA,HIGH);
    EEPROMWritelong(Y_ADDR,0);
    delay(5);
    Timer1.initialize(240); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
    Timer1.attachInterrupt( timerIsr ); // attach the service routine here
    Timer1.pwm(STEP_PWM_PIN,512,240);
    if(curXpos == 0 && curZpos == 0 && (!wasEngageEnabled || stopTriggered)){
      TRACEN("ret to zero, only Y is moving");
      if(stopTriggered == true){
        stopTriggered = false;
      }       
      Serial.print("{CPT}");
      Serial.print("\n");            
    }
    
  }  
  if(curXpos != 0){
    moveMotor('X', (curXpos * -1.0)/STEPS_PER_MM); 
  }  
  if(curZpos != 0){
    moveMotor('Z', (curZpos * -1.0)/STEPS_PER_MM);   
  }
  if(curZpos == 0 && curYpos == 0 && curZpos == 0){
    Serial.print("{CPT}");
    Serial.print("\n"); 
  }
}

void saveCurPos(){
  if(x_counter != 0){
    EEPROMWritelong(X_ADDR,x_pos + (x_counter*x_dirVec));
    delay(5);
  }
  if(y_counter != 0 || engageEnabled){
    EEPROMWritelong(Y_ADDR,y_pos + (y_counter*y_dirVec));
    delay(5);
  }
  if(z_counter != 0){
    EEPROMWritelong(Z_ADDR,z_pos + (z_counter*z_dirVec));
    delay(5);
  }
  
  x_counter = 0;
  y_counter = 0;
  z_counter = 0;
  engageEnabled = false;
}

void disableAllMotors(){   
    
  digitalWrite(X_ENA,HIGH);
  digitalWrite(Y_ENA,HIGH);
  digitalWrite(Z_ENA,HIGH);  
  enabled_X = false;
  enabled_Y = false;
  enabled_Z = false;
  TRACEN("disable all motors complete");  
}

void timerIsr()
{

  if(enabled_Y == true){
    y_counter++;
    if(y_counter >= y_thresh){
      
      TRACEN("y move done");
      TRACE(" y thresh: ");
      TRACE(y_thresh); 
      TRACE(" y counter: ");
      TRACE(y_counter);
      TRACE(" y pos: ");
      TRACE(y_pos); 
      TRACE(" y dirVec: ");
      TRACEN(y_dirVec);     
      TRACE("y pos after move is: ");
      TRACEN(y_pos + (y_counter*y_dirVec));
      
      enabled_Y = false;    
      digitalWrite(Y_ENA,HIGH);  
      y_done = true;             
      y_thresh = 0;       
    }
  }

  if(enabled_X == true){
    x_counter++;
    if(x_counter >= x_thresh){    

      TRACEN("x move done");
      TRACE(" x thresh: ");
      TRACE(x_thresh); 
      TRACE(" x counter: ");
      TRACE(x_counter);
      TRACE(" x pos: ");
      TRACE(x_pos); 
      TRACE(" x dirVec: ");
      TRACEN(x_dirVec);                
      TRACE("x pos after move is: ");
      TRACEN(x_pos + (x_counter*x_dirVec));      
      
      enabled_X = false;
      digitalWrite(X_ENA,HIGH);
      x_done = true;      
      x_thresh = 0;      
    }
  }
   
  if(enabled_Z == true){
    z_counter++;
    if(z_counter >= z_thresh){
      
      TRACEN("z move done");
      TRACE(" z thresh: ");
      TRACE(z_thresh); 
      TRACE(" z counter: ");
      TRACE(z_counter);
      TRACE(" z pos: ");
      TRACE(z_pos); 
      TRACE(" z dirVec: ");
      TRACEN(z_dirVec);      
      TRACE("z pos after move is: ");
      TRACEN(z_pos + (z_counter*z_dirVec));      
      
      enabled_Z = false;
      digitalWrite(Z_ENA,HIGH);
      z_done = true;          
      z_thresh = 0;           
    }
  } 

  if(x_started == true){
    if(x_done){
      move_x_Done = true;
    }else{
      move_x_Done = false;
    }
  } else {
    move_x_Done = true;
  }

  if(y_started == true){
    if(y_done){
      move_y_Done = true;      
    } else {
      move_y_Done = false;
    }
  } else {
    move_y_Done = true;
  }

  if(z_started == true){
    if(z_done){
      move_z_Done = true;      
    } else{
      move_z_Done = false;
    }
  } else {
    move_z_Done = true;
  }

  //if all motors are done, then save state and reset enables
  if(move_z_Done && move_y_Done && move_x_Done && (x_started || y_started || z_started)){
    
    if(x_dirVec !=0 && x_started){
      delay(5);
      EEPROMWritelong(X_ADDR,x_pos + (x_counter*x_dirVec));
      x_counter = 0;      
    }
    if(y_dirVec != 0 && y_started && !stopTriggered){
      delay(5);
      EEPROMWritelong(Y_ADDR,y_pos + (y_counter*y_dirVec));
      y_counter = 0;
    }
    if(z_dirVec != 0 && z_started){
      delay(5);
      EEPROMWritelong(Z_ADDR,z_pos + (z_counter*z_dirVec));
      z_counter = 0;
    }   
    
    x_started = false;
    y_started = false;
    z_started = false;   

    move_x_Done = false; 
    move_z_Done = false;
    move_y_Done = false;
          
    x_done = false;   
    y_done = false; 
    z_done = false;

    x_dirVec = 0;
    y_dirVec = 0;
    z_dirVec = 0;    

    if(!engageEnabled){
      Serial.print("{CPT}");
      Serial.print("\n");
    }

    if(stopTriggered == true){
      stopTriggered = false;
    }
    
  }
}



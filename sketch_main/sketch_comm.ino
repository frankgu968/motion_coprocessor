#if DEBUG 
#define TRACE(x) Serial.print(x);  
#define TRACEN(x) Serial.println(x);              
#else
#define TRACE(x) {};
#define TRACEN(x) {}; 
#endif

boolean isValidFloat(String in);
boolean missingEndBraceCheck(String data[], int numDataArgs, String commandString);
int findCommandIndex(String cmd);
void eventErrorHandle(int eventCode);

char command[4] = {NULL,NULL,NULL,NULL};
String cmd = "";
String endStr = "";

void parseEvent() {   
  if(Serial.available()){
    
    char inChar = (char)Serial.read();
    int eventCode = (unsigned int)-1;       //follows communication protocol standard
    boolean validInput = true;
    
    //if the first character denotes the start of a packet, then proceed with reading
    //else drop the frame (read until newline and return)       
    
    if(inChar == '{'){
        delay(1);           
        Serial.read();
        delay(1);
        command[0] = Serial.read();
        delay(1);
        command[1] = Serial.read();
        delay(1);
        command[2] = Serial.read();
        delay(1);        
        cmd = String(command); 
        int cmdIndex = findCommandIndex(cmd);         
        char endChar = Serial.read();
        
        if(cmdIndex == -1){
          validInput = false;
          eventCode = 3;
          Serial.readStringUntil('\n');
          TRACEN("invalid command index");
          errLED(); 
          Serial.print("{ERR," + String(eventCode) + "}\n");           
        }        
        else{                                          
          int numArguments = commandList[cmdIndex].numArgs;                           

          //depending on the command, check if the next char is valid (should be end of frame or comma)
          if((numArguments == 0 && endChar != '}') || (numArguments > 0 && endChar != ',')){
            validInput = false;
            eventCode = 3;       
            TRACEN("invalid char");     
          }

          int nargs = (numArguments > 0) ? numArguments:2;
          String data[nargs] = {""};
          float dat[nargs] = {0};
          
          if(numArguments > 0 && validInput){ 
                         
            //take in arguments               
            for (int i = 0; i < numArguments; i++){
              delay(1);          
              if(i == numArguments - 1){
                data[i] = Serial.readStringUntil('}');
                
              }
              else{
                data[i] = Serial.readStringUntil(',');
              }
            } 
                                
            delay(1);                        
            endStr = Serial.readStringUntil('\n');    
                   
            //do checks on data - this is dependent on the command
            //if the command is allowed to have floating point data format, analyze it 
            if (commandList[cmdIndex].allowedDataFormat){
                for(int k = 0; (k < numArguments) && validInput; k++){
                  if(data[k].length() != 0){
                    if(isValidFloat(data[k])){
                        dat[k] = data[k].toFloat();                  
                    }
                    else{
                      validInput = false;
                      eventCode = 4;
                      TRACEN("invalid float format");
                    }
                  }
                  else{
                    validInput = false;
                    eventCode = 5;
                    TRACEN("data fileds are empty!");            
                  }
                }          
              }                         

            //check for end of frame if all else is valid
            if (validInput){     
              validInput = missingEndBraceCheck(data,numArguments,cmd);        
            }
          }

          //if it is not something like a MOV command, then check if the end of string is immediately after frame close
          if (numArguments == 0){            
            endStr = Serial.readStringUntil('\n');
          }

          //handle the command if there is valid frame, otherwise throw error
          if(endStr.length() == 0 && validInput){
            if(mode == MANUAL_MODE && cmd != "STP"){
              boolean modeSelPressed = false;   
              pendingLED(); 
              TRACEN("manual mode, command queued. waiting...");
              while(!modeSelPressed){
                delay(10);                   
                if(digitalRead(STEP_PIN) == LOW){                  
                  modeSelPressed = true;      
                  eventCode = commandHandler(cmd, dat, numArguments);  
                }
              }                             
            } else{
              eventCode = commandHandler(cmd, dat, numArguments);  
            }         
            eventErrorHandle(eventCode);
          }     
          else{            
            if(eventCode == (unsigned int)-1){
              //invalid sequence (no bracket or new line)
              eventCode = 7;
              TRACE("invalid end sequence");
            }            
            errLED(); 
            Serial.print("{ERR," + String(eventCode) + "}\n");            
          }
        }       
      }  
      else{
        //frame does not start with a { so throw error
        Serial.readStringUntil('\n');
        errLED(); 
        Serial.print("{ERR,1}\n");         
      }  
    }       
}

boolean isValidFloat(String str){
  boolean validFormat = true;
  boolean decPointSeen = false;
  boolean negativePointSeen = false;
  int i = 0;
  
  if(str.charAt(0) == '-'){
    negativePointSeen = true;
    i++;
  }
  
  for(;i<str.length() && validFormat;i++){
    if(isDigit(str.charAt(i))){
      //continue through
    }      
    else if(!decPointSeen && str.charAt(i) == '.'){
      decPointSeen = true;
    }
    else if (!negativePointSeen && (str.charAt(i) == '-' && i > 0)){
      validFormat = false;
    }        
    else{
      validFormat = false;
    }
  }

  return validFormat;
}

boolean missingEndBraceCheck(String data[], int numDataArgs, String commandString){
  boolean validInput = true;
  
  for (int i = 0; (i < numDataArgs) && validInput; i++){
    if(data[i].lastIndexOf('\n') != -1){
      validInput = false;
      TRACE("missing end brace");
    }
  }
  
  if(validInput && (commandString.lastIndexOf('\n') != -1)){
    validInput = false;
    TRACE("missing end brace");
  }
    
  return validInput;  
}

int findCommandIndex(String cmd){  
  int cmdIndex = -1;
  for(int i = 0; (i < NUM_CMDS) && (cmdIndex == -1); i++){    
    if (commandList[i].command == cmd){      
      cmdIndex = i;      
    }
  }
  return cmdIndex;
}

void eventErrorHandle(int eventCode){
  if(eventCode != 0 && eventCode != 255){
    TRACEN(eventCode);
    if(eventCode != 100 && eventCode != 120){
      Serial.print("{CPT}");
      Serial.print("\n");
    }
  } 
  else if (eventCode == 255){
    TRACE("invalid command");
    errLED(); 
    Serial.print("{ERR," + String(eventCode)+"}");
    Serial.print("\n");
  }          
  else{    
    TRACE("something went wrong ");       
    errLED();                   
    Serial.print("{ERR," + String(eventCode)+"}");
    Serial.print("\n");
  }
  
  return;
}




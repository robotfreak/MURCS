/*
  robot commanded by serial input
 
 Looks for a set of ASCII characters in the signal to send
 commands to drive a small robot. 
 
 The minimum circuit:
 * Serial input connected to RX pin 0
 * Serial output connected to TX pin 1

 * optional Bluetooth Module connected to Serial2
 
 
 Note: If you don't yet have a serial device to connect with, you can use the 
 built in Serial Monitor in the Arduino software when connect via USB for testing.
 Also, be sure to disconect RX & TX pins from other devices when trying to program
 the Arduino over USB.
 
 created 2010
 by Tim Heath, Ryan Hickman, and Glen Arrowsmith
 Visit http://www.cellbots.com for more information
 adapted for MURCS robot by RobotFreak 2013
 */


#define BUFFERSIZE 20
#define DEFAULT_speedMultiplier 5 
#define MAX_SPEED_LIMIT   150
#define USE_BLUETOOTH_SERIAL2    // we use Bluetooth over Serial2

// ** GENERAL SETTINGS ** - General preference settings
boolean DEBUGGING = true; // Whether debugging output over serial is on by defauly (can be flipped with 'h' command)

// ** Robot Motor Controller Pin Settings
int m1SpeedControl = 13; //M1 Speed Control
int m2SpeedControl = 11; //M2 Speed Control
int m1DirectionControl = 12; //M1 Direction Control
int m2DirectionControl = 10; //M2 Direction Control

int speedMultiplier = DEFAULT_speedMultiplier; // Default speed setting. Uses a range from 1-10 
const long maxRunTime = 2000; // Maximum run time for motors without additional command. * Should use a command to set this. * 

// No config required for these parameters
unsigned long stopTime=millis(); // used for calculating the run time for motors
char incomingByte; // Holds incoming serial values
char msg[8]; // For passing back serial messages
char inBytes[BUFFERSIZE]; //Buffer for serial in messages
int serialIndex = 0; 
int serialAvail = 0;

void setup() {
  pinMode(m1SpeedControl,OUTPUT);
  pinMode(m2SpeedControl,OUTPUT);
  pinMode(m1DirectionControl,OUTPUT);
  pinMode(m2DirectionControl,OUTPUT);

  Serial.begin(9600);
#ifdef USE_BLUETOOTH_SERIAL2
  Serial2.begin(115200);
#endif
  if (DEBUGGING) {
      Serial.println("MURCS CellBot V1.1");
  }
 
} 


// Drive DC motors to move the robot using values in range -100 to 100 for left and right
unsigned long driveWheels(int valueLeft, int valueRight) {
    // Set left motor pins to turn in the desired direction
    if (valueLeft < 0){
      digitalWrite(m1DirectionControl,HIGH);
    }
    else {
      digitalWrite(m1DirectionControl,LOW);
    }
    // Set right motor pins to turn in the desired direction
    if (valueRight < 0){
      digitalWrite(m2DirectionControl,HIGH);
    }
    else {
      digitalWrite(m2DirectionControl,LOW);
    }
    // Maps "w" values to the wider range that the motor responds to
    valueLeft = map(abs(valueLeft), 0, 100, 0, MAX_SPEED_LIMIT);
    valueRight = map(abs(valueRight), 0, 100, 0, MAX_SPEED_LIMIT);
    analogWrite(m1SpeedControl,valueLeft);
    analogWrite(m2SpeedControl,valueRight);
  }

  stopTime=millis() + maxRunTime; // Set time to stop running based on allowable running time
  return stopTime;
}

// Stop the bot
void stopBot() {
  driveWheels(0,0);
  if (DEBUGGING) { Serial.println("Stopping both wheels"); }
  serialReply("i", "st"); // Tell the phone that the robot stopped
}


// Replies out over serial and handles pausing and flushing the data to deal with Android serial comms
void serialReply(char* sensorname, char* tmpmsg) {
#ifdef USE_BLUETOOTH_SERIAL2
  Serial2.print(sensorname);
  Serial2.print(":");
  Serial2.println(tmpmsg); // Send the message back out the serial line
  //Wait for the serial debugger to shut up
  delay(200); //this is a magic number
  Serial2.flush(); //clears all incoming data
#else
  Serial.print(sensorname);
  Serial.print(":");
  Serial.println(tmpmsg); // Send the message back out the serial line
  //Wait for the serial debugger to shut up
  delay(200); //this is a magic number
  Serial.flush(); //clears all incoming data
#endif
}

// Check if enough time has elapsed to stop the bot and if it is safe to proceed
void checkIfStopBot() {
  if (stopTime < millis()) {
    stopBot();
  }
}

// Send command to attached Bluetooth device to initiate pairing
void pairBluetooth() {
#ifdef USE_BLUETOOTH_SERIAL2
//  Serial2.print("\r\n+INQ=1\r\n"); // This is for Seeedstudio master/slave unit (change as needed for your model)
#else
  Serial.print("\r\n+INQ=1\r\n"); // This is for Seeedstudio master/slave unit (change as needed for your model)
#endif
}

// Reads serial input if available and parses command when full command has been sent. 
void readSerialInput() {
#ifdef USE_BLUETOOTH_SERIAL2
  while(Serial2.available() && serialIndex < BUFFERSIZE) {
#else
  while(Serial.available() && serialIndex < BUFFERSIZE) {
#endif
    //Store into buffer.
    inBytes[serialIndex] = Serial2.read();

    //Check for command end.    
    if (inBytes[serialIndex] == '\n' || inBytes[serialIndex] == ';' || inBytes[serialIndex] == '>') { //Use ; when using Serial Monitor
       inBytes[serialIndex] = '\0'; //end of string char
       parseCommand(inBytes); 
       serialIndex = 0;
    }
    else{
      serialIndex++;
    }
  }
  
  if(serialIndex >= BUFFERSIZE){
    //buffer overflow, reset the buffer and do nothing
    //TODO: perhaps some sort of feedback to the user?
    for(int j=0; j < BUFFERSIZE; j++){
      inBytes[j] = 0;
      serialIndex = 0;
    }
  }
}

// Cleans and parses the command
void parseCommand(char* com) {
  if (com[0] == '\0') { return; } //bit of error checking
  int start = 0;
  //get start of command
  while (com[start] != '<'){
    start++; 
    if (com[start] == '\0') {
      //its not there. Must be old version
      start = -1;
      break;
    }
  }
  start++;
  performCommand(com);
}

void performCommand(char* com) {  
  if (strcmp(com, "f") == 0) { // Forward
    stopTime = driveWheels(speedMultiplier * 10, speedMultiplier * 10);
  } else if (strcmp(com, "r") == 0) { // Right
    stopTime = driveWheels(speedMultiplier * 10, speedMultiplier * -10);
  } else if (strcmp(com, "l") == 0) { // Left
    stopTime = driveWheels(speedMultiplier * -10, speedMultiplier * 10);
  } else if (strcmp(com, "b") == 0) { // Backward
    stopTime = driveWheels(speedMultiplier * -10, speedMultiplier * -10);
  } else if (strcmp(com, "s") == 0) { // Stop
    stopBot();
  } else if (strcmp(com, "h") == 0) { // Help mode - debugging toggle
    // Print out some basic instructions when first turning on debugging
    if (not DEBUGGING) {
      Serial.println("Ready to listen to commands! Try ome of these:");
      Serial.println("F (forward), B (backward), L (left), R (right), S (stop), D (demo).");
      Serial.println("Also use numbers 1-9 to adjust speed (0=slow, 9=fast).");
    }
    DEBUGGING = !DEBUGGING;
  } else if (strcmp(com, "1") == 0 || strcmp(com, "2") == 0 || strcmp(com, "3") == 0 || strcmp(com, "4") == 0 || strcmp(com, "5") == 0 || strcmp(com, "6") == 0 || strcmp(com, "7") == 0 || strcmp(com, "8") == 0 || strcmp(com, "9") == 0 || strcmp(com, "0") == 0) {
    //I know the preceeding condition is dodgy but it will change soon 
    if (DEBUGGING) { Serial.print("Changing speed to "); }
    int i = com[0];
    speedMultiplier = i - 48; // Set the speed multiplier to a range 1-10 from ASCII inputs 0-9
    if (DEBUGGING) { Serial.println(speedMultiplier); }
  } else if (com[0] == 'w') { // Handle "wheel" command and translate into PWM values ex: "w -100 100" [range is from -100 to 100]
    int valueLeft=90, valueRight=90;
    sscanf (com,"w %d %d",&valueLeft, &valueRight); // Parse the input into multiple values
    stopTime = driveWheels(valueLeft, valueRight);
  } else if (com[0] == 'p') { // Initiates Bluetooth pairing so another device can connect
    pairBluetooth();
  } else { 
    serialReply("e", com);// Echo unknown command back
    if (DEBUGGING) {
      Serial.print("Unknown command: ");
      Serial.println(com);
    }
  }
}

// Main loop running at all times
void loop() 
{
  readSerialInput();
  checkIfStopBot();
}


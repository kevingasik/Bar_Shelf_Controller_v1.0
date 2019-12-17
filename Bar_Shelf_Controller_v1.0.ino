
/* Declares libray .h files to include for the controller */
#include "FrenicVFD.h" // Allows Serial Communication for VFD AC motor Controllers
#include "Bounce2.h" // Allows for DeBounces of signals
#include "EEPROM.h" // Allows for ??? 
#include <array>

/*Declares Objects to used in setup and loop functions of the controller */
FrenicVFD motor = FrenicVFD(1);//Create a VFD motor Object
//Declare a Bounce Object with 6 positions
Bounce inputSwitches[6];

/*Declares Constants to be assigned for the setup and loop functions */
uint16_t accelTime_ms = 1, decelTime_ms = 1; //Set Aceel and Decel times
elapsedMillis decelTimer = 0;

/*Declares Array and Enumeration constants to be assigned for the setup and loop functions */
// pin 23 -- SWITCH_UP
// pin 27 -- SWITCH_DN
// pin 28 -- SWITCH_DOT
// pin 16 -- SWITCH_ULS
// pin 17 -- SWITCH_DLS
// pin 19 -- SWITCH_UOT

//Declares the Solid State Relay pin to Control the brake motor on and off
int solidStateRelay = 11;

//Declare an Enumeration for states for the state machine in the loop function
enum currentState {
  FORWARD = 0,
  IDLEING = 1, // IDLEING = 1
  REVERSE = 2, // Enumerations will do plus one to the next entry so, REVERSE = 2
  BRAKE_ENGAGE = 3, // BRAKE = 3
  MALFUNCTION = 4, // MALFUNCTION = 4
  COAST = 5, // COAST = 5
  BRAKE_DISENGAGE = 6
};

//Declare a constant for the Pendant Switch Pins to be controlled by the operator
//const byte Pins[2] = {23, 27};
const byte Pins[6] = {27, 23, 28, 16, 17, 19};

//Declare an Enumeration for the different limit switches in the electro-mechanical system
enum inputSwitchEnum {
  SWITCH_OT = 0, // Normally Closed(NC) - active HIGH
  SWITCH_ULS, // NC - active HIGH
  SWITCH_DLS, // NC - active HIGH
  SWITCH_UP, // Normally Open(NO) - active low
  SWITCH_DN, // NO = active low
};

//Declare the starting state for the system
int state = IDLEING;
static int switchValues[sizeof(Pins)];

void setup() {
  //Open a serial port with 9600 baud, for the computer to print
  Serial.begin(9600);
  delay(200);
  Serial.println("hello WOrld");
  // start VFD comms and initialize object
  motor.begin(&Serial2, 19200, 13, SERIAL_8N1); // Begin Communication with VFD
  motor.setAccelerationTime(accelTime_ms, false); //Change a setting in the VFD
  motor.setDecelerationTime(decelTime_ms, false); //Change a setting in the VFD
  delay(100); // Delay for communications to take effect through serial port
  motor.setSpeed(33); // The lowest is speed at 3%


  //Initializes the solid State Relay Pin to control the brake motor
  pinMode(solidStateRelay, OUTPUT);

  //Set up Switch Pins to be pullup Resistors. Therefore they will read 1 when
  //a switch has not been triggered and 0 when the switch is triggering. In addition
  //Set up Switch Pins for Debouncing and allow the switch pins to be set in the enumeration Declaration
  for (int i = 0; i < sizeof(Pins); i++) {
    pinMode(Pins[i], INPUT_PULLUP);
    inputSwitches[i] = Bounce();
    inputSwitches[i].attach(Pins[i]);
    inputSwitches[i].interval(50);
  }
}

void loop() {
  //main state machine
  switch (state) {
    case IDLEING:
      //Just WAIT and read inputs to move motor
      brakeLow();
      Serial.println("IDLEING Brake ON/LOW");
      int *outputs;
      outputs = readSwitches();
      idleStateChanger(outputs);

      break;

    case FORWARD:
      goForwardState();
      outputs = readSwitches();
      ForwardStateChanger(outputs);
      Serial.println("FORWARD");
      break;

    case REVERSE:
      goReverseState();
      outputs = readSwitches();
      ReverseStateChanger(outputs);
      Serial.println("REVERSE");
      break;

    case BRAKE_DISENGAGE:
      brakeHigh(); 
      delay(600);
      outputs = readSwitches();
      brakeStateChanger(outputs);
      Serial.println("BRAKE DISENGAGE");
      break;

    case MALFUNCTION:
      malfunctionState();
      brakeLow();
      break;
      

    case COAST:
      coastMotorState();
      Serial.println("COASTING");
      break;

    default:
      Serial.print("Invalid Selection\n");
      break;
  }
}

void brakeLow() {
  digitalWrite(solidStateRelay, LOW);
  Serial.println("BRAKE LOW/ON");
}

void brakeHigh() {
  digitalWrite(solidStateRelay, HIGH);
  Serial.println("BRAKE HIGH/OFF");
}

void goForwardState() {
  motor.goFwd();
  return;
}

void goReverseState() {
  motor.goRev();
  return;
}

void coastMotorState() {
  motor.coastToStop();
  if (motor.isStopped() == HIGH) {
    delay(600);
    state = IDLEING;
    Serial.println("GOING TO IDLEING BRAKING");
  } else {
    state = COAST;
  }
  return;
}

void malfunctionState() {
  motor.controlledStop();
  return;
}

int* readSwitches() {
  for (int i = 0; i < sizeof(Pins); i++) {
    inputSwitches[i].update();
    switchValues[i] = inputSwitches[i].read();
  }
  return switchValues;
}

void brakeStateChanger(int arr[]) {
 //If hitting a lower limit switch and Up button is pressed go FWD
  if ((arr[4] == HIGH) && (arr[0] == LOW)) {
    state = FORWARD;
    Serial.println("Brake A");
    return;
  }
  else if (arr[4] == HIGH) {
    state = IDLEING;
    Serial.println("Brake C");
    return;
  }
  //If hitting a upper limit switch and the down button is pressed go Down
  else if ((arr[3] == HIGH) && (arr[1] == LOW)) {
    state = REVERSE;
    Serial.println("Brake D");
    return;
  }
  else if (arr[3] == HIGH) {
    state = IDLEING;
    Serial.println("Brake E");
    return;
  }
  //If hitting up and the upper limit switch is not pressed go up
  else if ((arr[0] == LOW) && (arr[3] == LOW)) {
    state = FORWARD;
    Serial.println("Brake F");
    return;
  }
  //If hitting down and the down limit switch is not pressed go down
  else if ((arr[1] == LOW) && (arr[4] == LOW)) {
    state = REVERSE;
    Serial.println("Brake G");
    return;
  }
  //if no button is pressed do nothing
  else if ((arr[0] == HIGH) && (arr[1] == HIGH)) {
    state = IDLEING;
    Serial.println("Brake H");
    return;
  }

}

void ForwardStateChanger(int arr[]) {
  //If going Forward you need to listen if you hit a limit switch or an Over travel listen switch
  //If in Forward State
  //If an Over Travel Limit switch is hit STOP
  if (( arr[2] == HIGH) || (arr[5] == HIGH)) {
    state = MALFUNCTION;
    Serial.println("Rev A");
    return;
  }
  //If the upper limit switch is hit COAST
  if (arr[3] == HIGH) {
    state = COAST;
    Serial.println("Rev B");
    return;
  }
  //If the upper limit switch is not hit and the FORWARD button is still pressed
  if ((arr[3] == HIGH) && (arr[0] == LOW)) {
    state = FORWARD;
    Serial.println("Rev C");
    return;
  } 
  //If the reverse button is released while in the reverse state
  else if (arr[0] == HIGH) {
    state = COAST;
    Serial.println("Rev D");
    return;
  }
}

void ReverseStateChanger(int arr[]) {
  //If going Reverse you need to listen if you hit a limit switch or an Over travel listen switch
  //If in Reverse
  //If an Over Travel Limit switch is hit STOP
  if (( arr[2] == HIGH) || (arr[5] == HIGH)) {
    state = MALFUNCTION;
    Serial.println("Rev A");
    return;
  }
  //If the lower limit switch is hit COAST
  if (arr[4] == HIGH) {
    state = COAST;
    Serial.println("Rev B");
    return;
  }
  //If the lower limit switch is not hit and the Reverse button is still pressed
  if ((arr[4] == HIGH) && (arr[1] == LOW)) {
    state = REVERSE;
    Serial.println("Rev C");
    return;
  } 
  //If the reverse button is released while in the reverse state
  else if (arr[1] == HIGH) {
    state = COAST;
    Serial.println("Rev D");
    return;
  }
}

void idleStateChanger(int arr[]) {
  //arr[0] = forward button // activated when LOW
  //arr[1] = reverse button // activated when LOW
  //arr[2] = Upp over travel
  //arr[3] = upper limit switch
  //arr[4] = lower limit switch
  //arr[5] = Down over travel
  //Forward is Up
  //Reverse id Down
  //Idle State
  delay(10);
  //If hitting an Upper Over Travel or Lower Over Travel Limit Switch Stop Everything
  if (( arr[2] == HIGH) || (arr[5] == HIGH)) {
    state = MALFUNCTION;
    Serial.println("A");
    return;
  }
  //If hitting a lower limit switch and Up button is pressed go FWD
  else if ((arr[4] == HIGH) && (arr[0] == LOW)) {
    //state = FORWARD;
    state = BRAKE_DISENGAGE;
    Serial.println("B");
    return;
  }
  else if (arr[4] == HIGH) {
    state = IDLEING;
    Serial.println("C");
    return;
  }
  //If hitting a upper limit switch and the down button is pressed go Down
  else if ((arr[3] == HIGH) && (arr[1] == LOW)) {
    //state = REVERSE;
    state = BRAKE_DISENGAGE;
    Serial.println("D");
    return;
  }
  else if (arr[3] == HIGH) {
    state = IDLEING;
    Serial.println("E");
    return;
  }
  //If hitting up and the upper limit switch is not pressed go up
  else if ((arr[0] == LOW) && (arr[3] == LOW)) {
    //state = FORWARD;
    state = BRAKE_DISENGAGE;
    Serial.println("F");
    return;
  }
  //If hitting down and the down limit switch is not pressed go down
  else if ((arr[1] == LOW) && (arr[4] == LOW)) {
    //state = REVERSE;
    state = BRAKE_DISENGAGE;
    Serial.println("G");
    return;
  }
  //if no button is pressed do nothing
  else if ((arr[0] == HIGH) && (arr[1] == HIGH)) {
    state = IDLEING;
    Serial.println("H");
    return;
  }

  return;
}

#include <AccelStepper.h>
#include <FlexCAN.h>

// Stepper constants
const float GEAR_RATIO = 25.2;
const float STEP_RESOLUTION = 200; // 200 steps per revolution
const float CURRENT = 1.5; // 1.5 amps current limit
const int MICROSTEP = 1;

// Encoder constants
const int COUNTS_PER_REV = 2000;
const int ENCODER_TYPE = 2; // X2 encoding

// CANBus ID and command unpacking constants;
const int8_t CAN_ID = 0x4;
const float P_MIN = -25.0f / 6.0f;
const float P_MAX = 25.0f / 6.0f;
const float V_MIN = -45.0f;
const float V_MAX = 45.0f;
const float KP_MIN = 0.0f;
const float KP_MAX = 500.0f;
const float KD_MIN = 0.0f;
const float KD_MAX = 5.0f;
const float T_MIN = -18.0f;
const float T_MAX = 18.0f;
const float I_MIN = -40.0f;
const float I_MAX = 40.0f;

// Initialize LED pin number
const int ledPin = 13;

// Initialize stepper motor pin numbers and variables
const int motorInterfaceType = 1;
const int M0 = 17;
const int M1 = 16;
const int M2 = 15;
const int dirPin = 8;
const int stepPin = 9;
const int sleepReset = 14;
float radPerSecond = 0.0;
volatile float oldPos = 0.0;
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

// Initialize encoder pin numbers and variables 
bool atStart = false;
const int encoderA = 20;
const int encoderB = 21;
const int encoderI = 22;
volatile int encoderCount = 0;
volatile float actualPosition = 0.0;

// Initialize exampleClass and reception variable, which are used for CANbus reception. Also initialize transmission message
char reception[8];
char stored[8];
static CAN_message_t txmsg;

class ExampleClass : public CANListener {
public:
   void printFrame(CAN_message_t &frame, int mailbox);
   bool frameHandler(CAN_message_t &frame, int mailbox, uint8_t controller); //overrides the parent version so we can actually do something
};

bool ExampleClass::frameHandler(CAN_message_t &frame, int mailbox, uint8_t controller){
//   Serial.println((String)"ID: " + frame.id);
//   Serial.print("Data: ");
   for (int c = 0; c < frame.len; c++){
      reception[c] = frame.buf[c];
//      Serial.print(reception[c], HEX);
   }
//   Serial.write('\n');

   return true;
}

ExampleClass exampleClass;

void setup() {
  pinMode(ledPin, OUTPUT);

  // Pull sleepReset high
  pinMode(sleepReset, OUTPUT);
  digitalWrite(sleepReset, HIGH);

  // Setup stepper settings
  pinMode(dirPin, OUTPUT);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  stepper.setMaxSpeed(10000);

  // Setup encoder settings
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  pinMode(encoderI, INPUT);
  attachInterrupt(encoderA, doEncoderA, CHANGE);

  // Setup CANBus
  CAN_filter_t filter;
  filter.id = CAN_ID;
  filter.flags.extended = 0;
  filter.flags.remote = 0;
  Can0.begin(1000000);
  Can0.attachObj(&exampleClass);
  
  for (uint8_t filterNum=0; filterNum<8; filterNum++){
    Can0.setMask(0x7FF<<18, filterNum);
    Can0.setFilter(filter, filterNum);  
  }
  for (uint8_t filterNum=0; filterNum<8; filterNum++){
    exampleClass.attachMBHandler(filterNum);
  }

  // Setup Serial
  // Serial.begin(9600);
  // while (!Serial);

  // Setup microstep settings
  setSteps();
}

void loop() {
  digitalWrite(ledPin, HIGH);   // set the LED on
  // Serial.println("Waiting for CANBus entry");

  while (strlen(reception) < 8){
    delay(1);
  }
  memcpy(stored, reception, strlen(reception));
  memset(reception, 0, sizeof(reception));
  
  if (stored[0]==0xFF && stored[1]==0xFF && stored[2]==0xFF && stored[3]==0xFF && stored[4]==0xFF && stored[5]==0xFF && stored[6]==0xFF && stored[7]==0xFC){
    // Serial.println("Enabled motor"); 
    atStart = true;
  }
  else if (stored[0]==0xFF && stored[1]==0xFF && stored[2]==0xFF && stored[3]==0xFF && stored[4]==0xFF && stored[5]==0xFF && stored[6]==0xFF && stored[7]==0xFE){
    stepper.setCurrentPosition(0);
    if (atStart == true){
      // Serial.println("Zeroing motor at start");
      atStart = false;
      encoderCount = 0;
    }
    else{
      // Serial.println("Zeroing motor due to skipped steps");
      stepper.setCurrentPosition(calculateSteps(actualPosition));
    }
  }
  else{
    float* new_cmd = unpack_cmd(stored);
    radPerSecond = new_cmd[1];
    runStepper(new_cmd[0]); 

  }
  
}

// FUNCTIONS FOR MOVEMENT AND STEPPER SETTINGS

float calculateSteps(float rad){
  return (rad * (STEP_RESOLUTION/(2*PI)) * GEAR_RATIO * MICROSTEP);
}

float calculateRad(float steps){
  return ((2 * PI * steps) / (STEP_RESOLUTION * GEAR_RATIO * MICROSTEP));
}

void runStepper(float rad){
  float steps = calculateSteps(rad);
  float stepsPerSecond = calculateSteps(radPerSecond);
  if (steps < stepper.currentPosition()){
    stepsPerSecond = -1.0 * stepsPerSecond;
    radPerSecond = -1.0 * radPerSecond;
  }
  
  stepper.setSpeed(stepsPerSecond);

  if (stepper.currentPosition() < steps){
    while (stepper.currentPosition() <= steps){
//      Serial.print((String)"Expected position: ");
//      Serial.println(calculateRad(stepper.currentPosition()), 6);
//      Serial.println(actualPosition, 6);
      stepper.runSpeed();
      delay(1);
    }
  }
  else if (stepper.currentPosition() > steps){
    while (stepper.currentPosition() >= steps){
//      Serial.print((String)"Expected position: ");
//      Serial.println(calculateRad(stepper.currentPosition()), 6);
//      Serial.println(actualPosition, 6);
      stepper.runSpeed();
      delay(1);
    }
  }
  pack_reply(calculateRad(stepper.currentPosition()), radPerSecond);
//  pack_reply(actualPosition, radPerSecond); 
}

void setSteps(){  
  if (MICROSTEP < 16){
    digitalWrite(M2, LOW);
  }
  else{
    digitalWrite(M2, HIGH);
  }

  switch(MICROSTEP){
    case 1:
      digitalWrite(M0, LOW);
      digitalWrite(M1, LOW);
      Serial.println("FULL STEP | M0: LOW | M1: LOW | M2: LOW");
      break;
    case 2:
      digitalWrite(M0, HIGH);
      digitalWrite(M1, LOW);
      Serial.println("HALF STEP | M0: HIGH | M1: LOW | M2: LOW");
      break;
    case 4:
      digitalWrite(M0, LOW);
      digitalWrite(M1, HIGH);
      Serial.println("1/4 STEP | M0: LOW | M1: HIGH | M2: LOW");
      break;
    case 8:
      digitalWrite(M0, HIGH);
      digitalWrite(M1, HIGH);
      Serial.println("1/8 STEP | M0: HIGH | M1: HIGH | M2: LOW");
      break;
    case 16:
      digitalWrite(M0, LOW);
      digitalWrite(M1, LOW);
      Serial.println("1/16 STEP | M0: LOW | M1: LOW | M2: HIGH");
      break;
    case 32:
      digitalWrite(M0, HIGH);
      digitalWrite(M1, HIGH);
      Serial.println("1/32 STEP | M0: HIGH | M1: HIGH | M2: HIGH");
      break;
    default:
      Serial.println("MICROSTEP VALUE NOT VALID, PLEASE CHANGE");
      break;
  }
}

// FUNCTION INTERRUPTS FOR ENCODER 

void doEncoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(encoderA) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderB) == LOW) {
      encoderCount = encoderCount + 1;         // CW
    }
    else {
      encoderCount = encoderCount - 1;         // CCW
    }
  }
  // must be a high-to-low edge on channel A
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderB) == HIGH) {
      encoderCount = encoderCount + 1;          // CW
    }
    else {
      encoderCount = encoderCount - 1;          // CCW
    }
  }
  actualPosition = (encoderCount * 2 * PI) / (COUNTS_PER_REV * GEAR_RATIO * ENCODER_TYPE);
  if ((encoderCount % 250) == 0){
      pack_reply(calculateRad(stepper.currentPosition()), radPerSecond);
      // Serial.println(actualPosition);
//    pack_reply(actualPosition, radPerSecond);
  }
}

// FUNCTIONS FOR CANBUS UNPACKING AND PACKING COMMANDS
float uint_to_float(int x_int, float x_min, float x_max, int bitNum){
  /// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float off_set = x_min;
  return ((float)x_int)*span/((float)((1<<bitNum)-1)) + off_set;
}

int float_to_uint(float x, float x_min, float x_max, int bitNum){
  /// Converts a float to an unsigned int, given range and number of bits ///
  float span = x_max - x_min;
  float off_set = x_min;
  return (int) ((x-off_set)*((float)((1<<bitNum)-1))/span);
}

float* unpack_cmd(char *msg){
  float* cmd = new float[5];
  
  int p_int = (msg[0]<<8)|msg[1];
  int v_int = (msg[2]<<4)|(msg[3]>>4);
  int kp_int = ((msg[3]&0xF)<<8)|msg[4];
  int kd_int = (msg[5]<<4)|(msg[6]>>4);
  int t_int = ((msg[6]&0xF)<<8)|msg[7];

  // Serial.println(v_int);

  cmd[0] = uint_to_float(p_int, P_MIN, P_MAX, 16); // Negative for Gazebo purposes
  cmd[1] = uint_to_float(v_int, V_MIN, V_MAX, 12);
  cmd[2] = uint_to_float(kp_int, KP_MIN, KP_MAX, 12);
  cmd[3] = uint_to_float(kd_int, KD_MIN, KD_MAX, 12);
  cmd[4] = uint_to_float(t_int, T_MIN, T_MAX, 12);
  
  return cmd;
}

void pack_reply(float pos, float vel){
  pos = pos;
  vel = vel; 
  int p_int = float_to_uint(pos, P_MIN, P_MAX, 16);
  int v_int = float_to_uint(vel, V_MIN, V_MAX, 12);
  int t_int = float_to_uint(CURRENT, -T_MAX, T_MAX, 12);
  // Serial.println(p_int);
  // Serial.println(v_int);

  txmsg.id = 0x0;
  txmsg.len = 6;
    
  txmsg.buf[0] = CAN_ID;
  txmsg.buf[1] = p_int>>8;
  txmsg.buf[2] = p_int&0xFF;
  txmsg.buf[3] = v_int>>4;
  txmsg.buf[4] = ((v_int&0xF)<<4) + (t_int>>8);
  txmsg.buf[5] = t_int&0xFF;

  // for (int k=0; k<txmsg.len; k++){
  //  Serial.println(txmsg.buf[k]);
  // }

  Can0.write(txmsg);
}

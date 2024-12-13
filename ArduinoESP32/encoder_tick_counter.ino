// pins for the encoder inputs
#define RH_ENCODER_A 16 
#define RH_ENCODER_B 17
#define LH_ENCODER_A 19
#define LH_ENCODER_B 18
 
// variables to store the number of encoder pulses
// for each motor
volatile unsigned long leftCount = 0;
volatile unsigned long rightCount = 0;
 
void setup() {
  pinMode(LH_ENCODER_A, INPUT_PULLUP);
  pinMode(LH_ENCODER_B, INPUT_PULLUP);
  pinMode(RH_ENCODER_A, INPUT_PULLUP);
  pinMode(RH_ENCODER_B, INPUT_PULLUP);
  
  // initialize hardware interrupts
  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, RISING);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, RISING);
  
  Serial.begin(115200);
}
 
void loop() {
  Serial.print("Right Count: ");
  Serial.print(rightCount);
  Serial.print("  Left Count: ");
  Serial.print(leftCount);
  Serial.println();
  delay(100);
}
 
// encoder event for the interrupt call
void leftEncoderEvent() {
  if (digitalRead(LH_ENCODER_A) > digitalRead(LH_ENCODER_B)) 
    leftCount++;
  else
    leftCount--;
}

 
// encoder event for the interrupt call
void rightEncoderEvent() {
  if (digitalRead(RH_ENCODER_A) > digitalRead(RH_ENCODER_B)) 
    rightCount++;
  else
    rightCount--;
}
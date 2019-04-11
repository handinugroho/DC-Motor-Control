#define encoderResolution 3440
#define IN1 8
#define IN2 9
#define EN1 5
#define pinA 2
#define pinB 3

//PID Constant
#define Kp 20
#define Ki 20
#define Kd 0

#define interval 10 

volatile int pulse = 0;
volatile int previousPulse = 0;
volatile double rotSpeed = 0;

int pinA, pinB;

float setPoint = 1;

bool forward = 1;

float error = 0;
float lastError = 0;
float errorIntegral = 0;
float errorDerivative = 0;

long long int previousMillis,currentMillis;
float power;
float errorPID;

void encoderA() {
  pinA = digitalRead(pinA);
  pinB = digitalRead(pinB);
  
  if (pinA == LOW && pinB == LOW) {
    pulse--; // CCW
  }
  else if (pinA == LOW && pinB == HIGH) {
    pulse++; //CW
  }
  else if (pinA == HIGH && pinB == LOW) {
    pulse++; // CW
  }
  else if (pinA == HIGH && pinB == HIGH) {
    pulse--; // CCW
  }
 
  if (pulse >= encoderResolution){
   pulse = 0;
  }
  if (pulse < 0){
   pulse = encoderResolution;
  }
}

void encoderB(){
  pinA = digitalRead(pinA);
  pinB = digitalRead(pinB);
  
  if (pinA == LOW && pinB == LOW){
    pulse++; // CW
  }
  else if (pinA == LOW && pinB == HIGH){
    pulse--; //CcW
  }
  else if (pinA == HIGH && pinB == LOW){
    pulse--; // CCW
  }
  else if (pinA == HIGH && pinB == HIGH){
    pulse++; // CW
  }
  
  if (pulse >= encoderResolution){
   pulse = 0;
  }
  if (pulse < 0){
   pulse = encoderResolution;
  }
}

ISR(TIMER1_COMPA_vect){
  int deltaSpeed = (pulse - previousPulse);
  if(deltaSpeed < 0) {
  deltaSpeed +=encoderResolution;
  }
  rotSpeed = deltaSpeed/ (encoderResolution * 0.01);
  previousPulse = pulse;
}

float getPosition() {
  return (float) pulse*360/encoderResolution;
}

void init_timer(){
    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    OCR1A = 624;
    TCCR1B |= (1 << WGM12);   // CTC mode
    TCCR1B |= (1 << CS12);    // 256 prescaler  
    TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
    interrupts();             // enable all interrupts
}

void setup(){
  Serial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EN1, OUTPUT);

  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);

  init_timer();
  attachInterrupt(digitalPinToInterrupt(pinA), encoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), encoderB, CHANGE);
  
  previousMillis = millis();
  power = 0;
}

void loop() {

  if(forward) { // CW Mode
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else {    // CCW Mode
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  analogWrite(EN1, power);
  currentMillis = millis();
  if ((currentMillis - previousMillis) > interval){
     error = setPoint - rotSpeed;
     errorIntegral = (setPoint - rotSpeed) * (millis() - previousMillis) / 1000.0;
     errorDerivative = (error - lastError) * 1000.0 /(float)(millis() - previousMillis);
     errorPID = Kp*error + Ki*errorIntegral + Kd*errorDerivative;

     power += errorPID; 
     if(power>255) power=255;
     
     lastError = error;
     previousMillis = millis();
  }
  Serial.print(setPoint);
  Serial.print(" ");
  Serial.println(rotSpeed);

}

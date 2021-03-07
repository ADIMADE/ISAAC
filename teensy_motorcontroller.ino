
/* --- Teensy Pins --- */
#define motorLeftDir 3
#define motorLeftSpeed 2
#define motorLeftEnc 5
#define motorRightDir 1
#define motorRightSpeed 0
#define motorRightEnc 4

/* --- PID Controller --- */
//PID factors
//(Serial Plotter red line)
float kpLeft = 0.6;
float kiLeft = 0.0000000007; //.0000002;
float kdLeft = 0.000008; //0.00035

//(Serial Plotter green line)
float kpRight = 0.6;
float kiRight = 0.000000007; //0.0000000805
float kdRight = 0.000008; //0.00035

// PID Errors
double errorLeft, errorRight, cumErrorLeft, cumErrorRight, prevErrorLeft, prevErrorRight, lastErrorLeft, lastErrorRight;

// motor speed controll
int pwmLeft, pwmRight;
double targetRpm;

/* --- RPM Measurement --- */
#define leftEncoderCountPerRev 735
#define rightEncoderCountPerRev 735

float actualRpmLeft, actualRpmRight;

long previousMillis = 0;
long currentMillis = 0;
long elapsedTime = 0;

volatile long encoderLeftValue = 0;
volatile long encoderRightValue = 0;

int measurementInterval = 100;

/*--------- Drive Forward Function ---------*/
void driveForward(double targetRpm){

  // Calculation of pwm signal based on determined function
  //pwmLeft = (int)round((1.5425 * targetRpm) + 22.9215);
  //pwmRight = (int)round((1.5425 * targetRpm) + 22.9215);

  // Drive loop
  while(true){

    if (targetRpm == 0){
    pwmLeft = 0;
    pwmRight = 0;
    break;
    }
    
    currentMillis = millis();
    elapsedTime = currentMillis - previousMillis;

    // RPM calculation
    if (elapsedTime > measurementInterval){
      previousMillis = currentMillis;

      actualRpmLeft = (float)(encoderLeftValue * 600 / leftEncoderCountPerRev);
      actualRpmRight = (float)(encoderRightValue * 600 / rightEncoderCountPerRev);

      // Display RPM values
      Serial.print(targetRpm);
      Serial.print("   ");
      Serial.print(actualRpmLeft);
      Serial.print("       ");
      Serial.println(actualRpmRight); 

      // Reset encoder counters
      encoderLeftValue = 0;
      encoderRightValue = 0;
    
    
      // PID controll algorithm
      errorLeft = targetRpm - actualRpmLeft;
      errorRight = targetRpm - actualRpmRight;

      cumErrorLeft += errorLeft;
      cumErrorRight += errorRight;

      pwmLeft += (kpLeft * errorLeft) + (kiLeft * cumErrorLeft) + (kdLeft * (errorLeft - prevErrorLeft));
      pwmRight += (kpRight * errorRight) + (kiRight * cumErrorRight) + (kdRight * (errorRight - prevErrorRight));

    }

    if(pwmLeft > 255){
      pwmLeft = 255;
    }
    if(pwmLeft < 30){
      pwmLeft = 30;
    }
    if(pwmRight > 255){
      pwmRight = 255;
    }
    if(pwmRight < 30){
      pwmRight = 30;
    }

    // Start left motor  -> (Forward "LOW" / Backward "HIGH")
    digitalWrite(motorLeftDir, LOW);
    analogWrite(motorLeftSpeed, pwmLeft);

    // Start right motor -> (Forward "HIGH" / Backward "LOW")
    digitalWrite(motorRightDir, HIGH);
    analogWrite(motorRightSpeed, pwmRight);

    prevErrorLeft = errorLeft;
    prevErrorRight = errorRight;

    cumErrorLeft += errorLeft;
    cumErrorRight += errorRight;
  }
}

//-------- Encoder Loops --------//
void callbackEncoderLeft(){
  // Increment value for each pulse from encoder
  encoderLeftValue++;
}

void callbackEncoderRight(){
  // Increment value for each pulse from encoder
  encoderRightValue++;
}

/*-------- Initial Loop --------*/
void setup() {

  // PWM frequency settings
  analogWriteFrequency(motorLeftSpeed, 193); // pwm frequency 200Hz
  analogWriteFrequency(motorRightSpeed, 193);// pwm frequency 200Hz
  
  // Setup GPIO
  pinMode(motorLeftDir, OUTPUT);
  pinMode(motorLeftSpeed, OUTPUT);
  pinMode(motorLeftEnc, INPUT_PULLUP);
  pinMode(motorRightDir, OUTPUT);
  pinMode(motorRightSpeed, OUTPUT);
  pinMode(motorRightEnc, INPUT_PULLUP);

  // Assign interrupts for encoder pins
  attachInterrupt(digitalPinToInterrupt(motorLeftEnc), callbackEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(motorRightEnc), callbackEncoderRight, RISING);

  // Setup timer
  previousMillis = millis();
}

/*--------- Main Loop ---------*/
void loop() {
  Serial.println("Start Motor Controller");
  driveForward(40);
}

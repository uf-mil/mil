// Gokart sensor debugging and data collection script. Data is used to calibrate drive by wire with hall sensors.
// Arduino Mega used to power and read sensors. For data acquisition, used in tandem with python script to save serial data. (Pressure sensor requires 12V external source)
// Author: Dushyanth Ganesan, University of Florida  
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// INITIALIZE VARIABLES

//// time count with millis() or micros() must be declared volatile long 
float steeringPot, brakePressure, brakePed, accPed;

// front driver wheel speed
volatile float          driverSpeed     = 0;
const int               driverSpeedPin  = 4;
volatile unsigned long  driverPrevTime  = 0;
volatile unsigned long  driverDelt      = 50000;

// front passenger wheel speed
volatile float          passSpeed     = 0;
const int               passSpeedPin  = 2;
volatile unsigned long  passPrevTime  = 0;
volatile unsigned long  passDelt      = 50000;

// rear axle speed
volatile float          rearSpeed     = 0;
const int               rearSpeedPin  = 3;
volatile unsigned long  rearPrevTime  = 0;
volatile unsigned long  rearDelt      = 50000;


volatile unsigned long  currentTime   = 0;
unsigned long           loopTime      = 0;

const float             perimeter     = PI*0.2794;  // meters
int                     driverMag     = 1;          // shows hall effect sensor tick for driver wheel speed
int                     passMag       = 1;          // shows hall effect sensor tick for passenger wheel speed
float                   rearMag       = 1;          // shows hall effect sensor tick for rear axle speed

 int interrCount = 0;                             // only when required for testing and debugging interrupt consistency

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);                // Baud Rate: 115200 (Ensure similar setting on serial monitor/plotter or other serial scripts.)
  pinMode(driverSpeedPin, INPUT_PULLUP); // Required: pullup resistor toggled on 
  pinMode(passSpeedPin, INPUT_PULLUP); // Required: pullup resistor toggled on
  pinMode(rearSpeedPin, INPUT_PULLUP); // Required: pullup resistor toggled on
  attachInterrupt(digitalPinToInterrupt(driverSpeedPin),ISR_driverSpeed, FALLING); // FALLING: interrupt triggered when pin reading changes from HIGH to LOW
  attachInterrupt(digitalPinToInterrupt(passSpeedPin),ISR_passSpeed, FALLING); // FALLING: interrupt triggered when pin reading changes from HIGH to LOW
  attachInterrupt(digitalPinToInterrupt(rearSpeedPin),ISR_rearSpeed, FALLING); // ISR: Interrupt Service Routine (Callback function for the interrupt)
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  // put your main code here, to run repeatedly:
  loopTime = micros(); // keeps track of time at each loop iteration (microseconds)

  // read pins
  driverMag     = digitalRead(driverSpeedPin);// driver wheel speed sensor
  passMag       = digitalRead(passSpeedPin); // passenger wheel speed sensor
  rearMag       = digitalRead(rearSpeedPin); // rear axle speed sensor
  steeringPot   = analogRead(A7);            // steering potentiometer
  brakePed      = analogRead(A3);            // brake pedal hall effect sensor
  accPed        = analogRead(A11);           // accelerator hall effect sensor
  brakePressure = analogRead(A15);           // master cylinder pressure sensor

  // Stop case: If no tick in 4 seconds, set speed = 0
  
  if (loopTime - passPrevTime > 4000000) {
    passSpeed = 0;
    }

  if (loopTime - rearPrevTime > 4000000) {
    rearSpeed = 0;
    }
  

  // PRINT MEASUREMENTS TO SERIAL PORT
  //Sends data via serial to pc
  Serial.print(micros());
  Serial.print(",");
  Serial.print(int(steeringPot));
  Serial.print(",");
  Serial.print(int(brakePed));
  Serial.print(",");
  Serial.print(int(accPed));
  Serial.print(",");
  Serial.print(int(brakePressure));
  Serial.print(",");
  Serial.print(int(rearSpeed));
  Serial.print(",");
  Serial.print(int(passSpeed));
  Serial.print(",");
  Serial.print(int(driverSpeed));
  Serial.print("\n"); 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// INTERRUPT CALLBACK FUNCTIONs FOR SPEED CALC (Interrupt Service Routines)

void ISR_driverSpeed() {
  currentTime = micros();
  driverDelt        = currentTime - driverPrevTime; // time between sensor ticks (in microseconds)
  driverPrevTime    = currentTime;
  driverSpeed     = 60000000.0000/driverDelt; // rpm x 10
  interrCount ++;  // debugging and testing interrupts
  
}

void ISR_passSpeed() {
  currentTime = micros();
  passDelt        = currentTime - passPrevTime; // time between sensor ticks (in microseconds)
  passPrevTime    = currentTime;
  passSpeed     = 60000000.0000/passDelt; // rpm x 10
  interrCount ++;  // debugging and testing interrupts
  
}

void ISR_rearSpeed() {
  currentTime = micros();
  rearDelt        = currentTime - rearPrevTime;
  rearPrevTime    = currentTime;
  rearSpeed     = 60000000.0000/rearDelt; // rpm x 10
  interrCount ++;  // debugging and testing interrupts
}

#include <TimerOne.h>
#include <TimerThree.h>

// Back Wheel Encoder Stuff
struct WheelEncoder
{
  int pin;
  volatile int count =0;
};
WheelEncoder back_wheel;
int back_wheel_get_count()
{
  int c = back_wheel.count;
  back_wheel.count = 0;
  return c;
}
void back_wheel_isr()
{
  back_wheel.count += 1;
}
void back_wheel_init()
{
  back_wheel.pin = 3;
  pinMode(back_wheel.pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(back_wheel.pin), back_wheel_isr, FALLING);
}

// Servo Stuff
struct Servo
{
  int pin;
  uint16_t duty;
};
Servo throttle;
void servo_init()
{
  throttle.pin = 13;
  pinMode(throttle.pin, OUTPUT);
  Timer1.initialize(20000); // standard servo pwm period
  Timer1.start();
  Timer1.pwm(throttle.pin, 0);
}
void servo_update(uint16_t duty) // duty should always be [0, 1023]
{
  throttle.duty = duty;
  Timer1.setPwmDuty(throttle.pin, throttle.duty);
  digitalWrite(5, !digitalRead(5));
}

// Sensor data sending stuff
struct Sensors
{
  uint16_t brake_pressure = -1;
  uint16_t back_wheel     = -1;
  uint16_t steering       = -1;
  uint16_t pedal          = -1;
};

struct SensorMsg
{
  Sensors sensors;
  uint8_t checksum = 0;
  bool initialized = false;
};

SensorMsg sensor_msg;
// TODO: make this function shared in the cpp drivers and the arduino code MONO REPO!!!
uint8_t calculate_crc(uint8_t* msg, int count)
{
  uint8_t crc = 0;
  for (int i = 0; i < count; i++)
    crc ^= msg[i];
  return crc;
}

void get_send_sensors()
{
  if (!sensor_msg.initialized)
    return;
  Sensors* sensors = &(sensor_msg.sensors);
  sensors->brake_pressure = analogRead(A15);
  sensors->back_wheel     = back_wheel_get_count();
  sensors->steering       = analogRead(A7);
  sensors->pedal          = analogRead(A11);
  sensor_msg.checksum = calculate_crc((uint8_t*)sensors, sizeof(Sensors));
  // 64 byte tx buffer, so we should be good here not waiting forever
  Serial.write((char*)sensors, sizeof(Sensors));
  Serial.write((char*)&(sensor_msg.checksum), sizeof(uint8_t));
}
void sensors_init()
{
  Timer3.initialize(10000); //100 Hz
  Timer3.attachInterrupt(&get_send_sensors);
  Timer3.start();
  sensor_msg.initialized = true;
}

// TODO: make this function shared in the cpp drivers and the arduino code MONO REPO!!!
bool check_checksum(uint8_t* data, int data_len)
{
  uint8_t crc = calculate_crc(data, data_len-1);
  if(crc == data[data_len-1])
    return true;
  return false;
}


void setup()
{
  Serial.begin(115200);
  back_wheel_init();
  servo_init();
  pinMode(5, OUTPUT);
}


void loop()
{
  if (Serial.available())
  {
    uint8_t input[3] = {};
    Serial.readBytes(input, 3);
    if (check_checksum(input, 3))
    {
      if (!sensor_msg.initialized)
        sensors_init();
      uint16_t servo = ((uint16_t)input[1] << 8) | input[0];
      if (servo > 1023)
        servo = 0;
      servo_update(servo);
    }
  }
}


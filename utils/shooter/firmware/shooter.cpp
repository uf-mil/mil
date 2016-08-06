#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

#include <Arduino.h>

#include <Servo.h>

const int SHOOTER_PIN = 3;
const int FEEDER_A_PIN = 8;
const int FEEDER_B_PIN = 9;
const int FEEDER_PWM_PIN = 5;

class SpeedController
{
  protected:
    bool reversed;
    virtual void _set(int s);
    virtual int _get();
    SpeedController()
    {
      reversed = false;
    }
  public:
    void set(int s)
    {
      if (reversed) _set(-s);
      else _set(s);
    }
    int get()
    {
      if (reversed) return -_get();
      else return _get();
    }
    void setReversed(bool r)
    {
      reversed = r;
    }
    void on()
    {
      set(100);
    }
    void off()
    {
      set(0);
    }
    void reverse()
    {
      set(-100);
    }
};

//Class for the Victor 883 speed controller used to control the shooter motors
class Victor : public SpeedController
{
  private:
    Servo controller;
    int goal;
    int cur;
    int pin;
    //Internal set command to write to controller PWM
    void _set(int s)
    {
      goal = map(s,-100,100, 1000,2000);
    }
    int _get()
    {
      return map(cur,1000,2000,-100,100);
    }
  public:
    Victor(int p)
    {
      pin = p;
      controller = Servo();
      goal = 1500;
      cur = goal;
    }
    void init()
    {
      controller.attach(pin);
      goal = 1500;
      cur = goal;
      controller.writeMicroseconds(cur);
    }
    //Should be called in each loop so PWM slowly ramps up, doesn't work otherwise
    void run()
    {
      if (cur != goal)
      {
        if (goal == 1500) cur = 1500;
        else if (goal < 1500)
        {
          cur -= 100;
        }
        else if (goal > 1500)
        {
          cur += 100;
        }
        controller.writeMicroseconds(cur);
      }
    }
};
Victor shooter(SHOOTER_PIN);

//Class for controlling the Pololu speed controller used for the feeder
class Pololu : public SpeedController
{
  private:
    int inA_pin;
    int inB_pin;
    int pwm_pin;
    int speed;
    //Servo controller;
    void _set(int s)
    {
      speed = s;
      if (s == 0) {
        digitalWrite(inA_pin,LOW);
        digitalWrite(inB_pin,LOW);
        analogWrite(pwm_pin,0);
        controller.writeMicroseconds(1500);
      } else if(s < 0) {
        digitalWrite(inA_pin,HIGH);
        digitalWrite(inB_pin,LOW);
        analogWrite(pwm_pin,map(s,-100,0,0,255));
      } else if (s > 0) {
        digitalWrite(inA_pin,LOW);
        digitalWrite(inB_pin,HIGH);
        analogWrite(pwm_pin,map(s,0,100,0,255));
        //controller.writeMicroseconds(map(s,0,100,1000,2000));
      }     
    }
    int _get()
    {
      return speed;
    }
  public:
    Pololu(int a, int b, int pwm)
    {
      inA_pin = a;
      inB_pin = b;
      pwm_pin = pwm;
      speed = 0;
      //controller = Servo();
    }
    void init()
    {
      pinMode(inA_pin,OUTPUT);
      pinMode(inB_pin,OUTPUT);
      pinMOde(pwm_pin,OUTPUT);
      //controller.attach(pwm_pin);
    }
};
Pololu feeder(FEEDER_A_PIN,FEEDER_B_PIN,FEEDER_PWM_PIN);

class AutoController
{
  private:
    static const unsigned long SPIN_UP_TIME = 1000; //Constant for time to spin up flywheels before feeding balls in
    static const unsigned long SHOOT_TIME = 12000; //Time to shoot all 4 balls once after they start being fed in
    static const unsigned long TOTAL_TIME = SPIN_UP_TIME + SHOOT_TIME;
    static const int FEED_SPEED = 50; //speed (out of 100) to set feeder motor to when feeding balls

    unsigned long start_shoot_time;
    bool auto_shoot;
  public:
		AutoController()
		{
			start_shoot_time = 0;
			auto_shoot = false;
		}
		void shoot()
		{
			auto_shoot = true;
			start_shoot_time = millis();
		}
		void cancel()
		{
			feeder.off();
			shooter.off();
			auto_shoot = false;
		}
		void run()
		{
			if (auto_shoot)
			{
				unsigned long time_since_start = millis() - start_shoot_time;
				if (time_since_start < SPIN_UP_TIME) shooter.on();
				else if (time_since_start > SPIN_UP_TIME && time_since_start < TOTAL_TIME) feeder.on(); //feeder.motor.set(FEED_SPEED);
				else if (time_since_start > TOTAL_TIME) cancel();
			}
		}	
};
AutoController autoController;

class Comms
{
  private:
    //ROS
    ros::NodeHandle nh;
    std_msgs::String str_msg;
    //ros::Publisher chatter;
    ros::Subscriber<std_msgs::String> sub;

    static void messageCallback(const std_msgs::String& str_msg)
    {
      String s = str_msg.data;
      if (s == "flyon")
        shooter.on();
      else if (s == "flyoff")
        shooter.off();
      else if (s == "feedon")
        feeder.on();
      else if (s == "feedoff")
        feeder.off();
      else if (s == "feedreverse")
        feeder.reverse();
      else if (s == "shoot")
        autoController.shoot();
      else if (s == "cancel")
        autoController.cancel();
      else if (s == "ledon")
        digitalWrite(13,HIGH);
      else if (s == "ledoff")
        digitalWrite(13,LOW);
    }
  public:
    Comms() :
      str_msg(),
      sub("/shooter/control",&messageCallback)
      //chatter("chatter", &str_msg)
    {
      pinMode(13,OUTPUT);
    }
    void init()
    {
      nh.initNode();
      nh.subscribe(sub);
      //nh.advertise(chatter);   
    }
    void run()
    {
      nh.spinOnce();
    }
};

Comms com;
void setup()
{
  shooter.setReversed(true);
  shooter.init();
  feeder.init();
  com.init();
}

void loop()
{
  com.run();
  autoController.run();
  shooter.run();
  delay(100);
}

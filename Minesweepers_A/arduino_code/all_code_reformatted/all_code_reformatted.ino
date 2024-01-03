// libraries
#include <QuadratureEncoder.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>


// pins
#define metal_detector 4
#define alarm 5
#define magnet 6

// objects
Encoders leftEncoder(20,21);
Encoders rightEncoder(2,3);

// global variables
unsigned long lastMilli = 0;
unsigned long last_detected = 0;
int metal = 0;
int previous_metal = 0;
float metal_average = 0;
int counter = 0;

//ROS
ros::NodeHandle  nh;

// left ticks publisher
std_msgs::Int32 left_ticks_count;
ros::Publisher left_ticks("left_ticks", &left_ticks_count);

// right ticks publisher
std_msgs::Int32 right_ticks_count;
ros::Publisher right_ticks("right_ticks", &right_ticks_count);

// metal detector publisher
std_msgs::Int8 alert;
ros::Publisher detection("detection", &alert);


// callback for subscriber
void messageCb( const std_msgs::Int16 &gripper){
  if (gripper.data == 11){
    digitalWrite(magnet,LOW);  
  }
  else if (gripper.data == 10){
    digitalWrite(magnet,HIGH);  
  }
}

// controller subscriber
ros::Subscriber<std_msgs::Int16> magnetism("controller", messageCb);

void setup() {

  // pin modes
  pinMode(metal_detector, INPUT);
  pinMode(alarm, OUTPUT);
  pinMode(magnet, OUTPUT);


  // ros nodes initialization
  nh.initNode();
  nh.advertise(left_ticks);
  nh.advertise(right_ticks); 
  nh.subscribe(magnetism);
  nh.advertise(detection);
  
  // initialize relay as off
  digitalWrite(alarm, HIGH);
  digitalWrite(magnet, HIGH);
}

void loop() {

  // metal detection and alarm code
  metal = digitalRead(metal_detector);
  //metal_average = (metal_average* 9 / 10) + (metal * 1 / 10);

  if (metal == HIGH) {
    counter++;
    digitalWrite(alarm,LOW);
    }

  else{
    counter = 0;
    digitalWrite(alarm, HIGH);
    if (previous_metal != metal){
      previous_metal = 0;
      alert.data = 0;
      detection.publish(&alert);
    }
    }

  if (counter > 7) {
    if (previous_metal != metal){
      previous_metal = 1;
      alert.data = 1;
      detection.publish(&alert);
      }
    }  
  // encoders code
  if(millis()-lastMilli > 50){ 
    
    left_ticks_count.data = long(leftEncoder.getEncoderCount());
    right_ticks_count.data = long(rightEncoder.getEncoderCount());
    left_ticks.publish( &left_ticks_count );
    right_ticks.publish( &right_ticks_count );
    lastMilli = millis();
  }

  nh.spinOnce();

}

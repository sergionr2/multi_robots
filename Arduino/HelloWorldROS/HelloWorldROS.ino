/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
ros::Publisher chatter2("chatter2", &str_msg);

char hello[14] = "bonjour monde";
char hello2[14] = "hello world";
int ledPin = 13;
bool sen = false;

void setup()
{
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.advertise(chatter2);
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data = hello;
  chatter2.publish( &str_msg );
  chatter.publish( &str_msg );
  nh.spinOnce();
  if( sen ){
    digitalWrite(ledPin, HIGH);
    chatter2.publish( &str_msg );
  }
  else
    digitalWrite(ledPin, LOW);
  sen = !sen;
  delay(1000);
}

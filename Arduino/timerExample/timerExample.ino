/*
 *  Timer1 library example
 *  June 2008 | jesse dot tane at gmail dot com
 */
 
#include "TimerOne.h"
 //Pins
const int DIRA = 12; //Connect on Pin 12
const int PWMA = 3; //Connect on Pin 3
const int DIRB = 13; //Connect on Pin 13
const int PWMB = 11; //Connect on Pin 11
const int ledPin = 10;
//Communication protocol checking variables
bool check_start = 0;
bool check_ID = 0;

//This module's variables
const char START_CHAR = '#';
const char MY_ID = '1';
const char BROADCAST_ID = '0';
const char END_CHAR = '.';

//Action variables
char action = 0;
char power = 0;
char income = 0;

void setup() {
  pinMode(ledPin, OUTPUT);  
  pinMode(DIRA,OUTPUT);
  pinMode(PWMA,OUTPUT);
  pinMode(DIRB,OUTPUT);
  pinMode(PWMB,OUTPUT);
  //pinMode(LED_PIN, OUTPUT);
  
  Serial.begin(9600);

  Serial.println("SETUP OK");
  Timer1.initialize(100000); 
  Timer1.attachInterrupt(callback);
}
 
void callback()
{
  digitalWrite(ledPin, digitalRead(ledPin) ^ 1);
}
 void loop(){
  Serial.print("income = ");
  Serial.println(income);
  Serial.print("check_start = ");
  Serial.println(check_start);
  Serial.print("check_ID = ");
  Serial.println(check_ID);
  Serial.print("action = ");
  Serial.println(action);
  Serial.print("power = ");
  Serial.println(power);
  Serial.println("-----");
  
  if ( Serial.available() > 0 ){    
    income = Serial.read();

    if(income == START_CHAR){
      check_start = 1;
      Serial.println("START CHECKED!!!");
    }else if((income == MY_ID || income == BROADCAST_ID) && check_start == 1){
      check_ID = 1;
      Serial.println("ID CHECKED!!!");
    }else if((income == 'z' || income == 'q' || income == 's' || income == 'd' || income == 'a') && check_start == 1 && check_ID == 1){
      action = income;
      Serial.println("ACTION CHECKED!!!");
    }else if((income >= 'A' && income <= 'J') && check_start == 1 && check_ID == 1){
      power = income;
      power = (power - 'A')*25 + 25;
      Serial.println("POWER CHECKED!!!");
    }else if(income == END_CHAR){
      check_start = 0;
      check_ID = 0;
      Serial.println("END CHECKED!!!");
    }
  }

  switch (action){
    case 'a':
      digitalWrite(DIRA,LOW);
      analogWrite(PWMA,0);
      digitalWrite(DIRB,LOW);
      analogWrite(PWMB,0);
      break;
    
    case 'z':
      digitalWrite(DIRA,HIGH);
      analogWrite(PWMA,power);
      digitalWrite(DIRB,HIGH);
      analogWrite(PWMB,power);
      break;
      
    case 'q':
      digitalWrite(DIRA,HIGH);
      analogWrite(PWMA,power);
      digitalWrite(DIRB,LOW);
      analogWrite(PWMB,power);
      break;
      
    case 's':
      digitalWrite(DIRA,LOW);
      analogWrite(PWMA,power);
      digitalWrite(DIRB,LOW);
      analogWrite(PWMB,power);
      break;
      
    case 'd':
      digitalWrite(DIRA,LOW);
      analogWrite(PWMA,power);
      digitalWrite(DIRB,HIGH);
      analogWrite(PWMB,power);
      break;
      
    default:
      break;
  }
  
  delay(10);
}

/*
 * This program recives N_VALUES of size N_BYTES with the following protocol
 *  to start '@''@' the values byte by byte and '.' to finish
 *  
 */
 #include "TimerOne.h"
const byte N_BYTES = 2; // size of values recived
const byte N_VALUES = 3; // number of values to recive
byte incomingData[ N_BYTES*N_VALUES + 1 ]; // +1 end character

const char START_CHAR = '@';
const char POS_CHAR = 'P';
const char GOAL_CHAR = 'G';
const char END_CHAR = '.';

int values[N_VALUES]; // values[0] = x, in mm  //CHECK SIZE IF RECIVING BIGGER DATA, now: int16
                     // values[1] = y, in mm  
                    // values[2] = theta, in Degrees 
const bool DEBUG = false;
bool firstStart = false;
int incomingByte;      // a variable to read incoming serial data into

const int rightPWM = 3; // pins PWM A
const int leftPWM = 11; // PWM B

const int rightDir = 12; // DIR A
const int leftDir = 13; // DIR B

const int ledPin = 10; // LED

int x = 0;
int y = 0;
int theta = 0;

int xGoal = 0;
int yGoal = 0;
int thetaGoal = 0;

int distError = 0;
int distAcumError = 0;

int angleError = 0;
int angleAcumError = 0;
int duty = 0;

void setup() {
  pinMode( rightPWM, OUTPUT ); //Defining pins as output
  pinMode( leftPWM, OUTPUT );
  pinMode( rightDir, OUTPUT );
  pinMode( leftDir, OUTPUT );
  pinMode( ledPin, OUTPUT ); 
  Serial.begin( 9600 );
  
  Timer1.initialize(100000); 
  Timer1.attachInterrupt( control );
  digitalWrite( ledPin, HIGH );
  analogWrite( rightPWM, 0 );
  analogWrite( leftPWM, 0 );
}

void setVelocity( int pwmPin, int dirPin, int vel ){ // pwm_pin, direction_pin, velocity
  if(DEBUG)Serial.println("Seting Velocity");
  if( vel < 0 )
    digitalWrite( dirPin, LOW );
  else
    digitalWrite( dirPin, HIGH );
  vel = abs( vel );
  analogWrite( pwmPin, vel );
  if(DEBUG)Serial.println("Velocity Set");
}
void control()
{
    // constantes PID
    float Kp = 2;
    float Ki = 0;
    angleError = theta - thetaGoal;
    angleAcumError += angleError;
    duty = Kp*angleError; //Verify limits TODO
    setVelocity( rightPWM, rightDir, duty);
    setVelocity( leftPWM, leftDir, -1*duty);
}
void setPose( int newX, int newY, int newTheta ){
  x = newX;
  y = newY;
  theta = newTheta;   
}

void setGoal( int newX, int newY, int newTheta ){
  xGoal = newX;
  yGoal = newY;
  thetaGoal = newTheta;   
}


void loop() { 
  Serial.println(duty);  
  digitalWrite( ledPin, HIGH );
  if ( Serial.available() > 0 && !firstStart) {
    incomingByte = Serial.read();
    if( incomingByte == START_CHAR){
      firstStart = true; // if ---> @
      if(DEBUG)Serial.println("in @");
    }
  }
  if (Serial.available() > 0 && firstStart ){
    incomingByte = Serial.read();  
    if( (incomingByte == POS_CHAR || incomingByte == GOAL_CHAR) && firstStart ){ // if START ---> @ G || @ P
      bool isGoal = false;
      if( incomingByte == GOAL_CHAR )
         isGoal = true;
      if(DEBUG)Serial.println("Start");
      while(Serial.available() < N_BYTES*N_VALUES + 1 ){ // wait until reciving the values and the end character
      }
      if(DEBUG)Serial.println("Getting data");
      for( int i = 0; i < N_BYTES*N_VALUES + 1; i++){
          incomingData[ i ] = Serial.read();
      } 
      if( incomingData[ N_BYTES*N_VALUES ] == END_CHAR ){ // if END ---> .
        // first byte is the least significant byte
        if(DEBUG)Serial.println("Saving Data");
        for( int i = 0; i < N_VALUES; i++) // reset values
          values[ i ] = 0;
        for( int i = 0; i < N_BYTES*N_VALUES; i++){
          values[ i/N_BYTES ] |= ( (int)incomingData[ i ] << 8*( i%N_BYTES ) );  
         /*
          * equivalent to do: values[0] = incomingData[0] | (int)incomingData[0] << 8; 
          * setting the byts and then moving the byte (8 bites) to the left and doing an OR operation
          * 00000000  0000011  OR  11000000 00000000 ----> 11000000 00000011
          * ceros least_significant_byte OR second_byte ceros
          */
        }
        digitalWrite( ledPin, LOW );
        if( isGoal ){
          setGoal( values[0], values[1], values[2] );
          if(DEBUG)Serial.println("setGoal");
        }
        else{
          setPose( values[0], values[1], values[2] );
          if(DEBUG)Serial.println("setPose");
        }


      }
    }
    firstStart = false;
  } 
  //uncomment to DEBUG  
  if(DEBUG){         
    for( int i = 0; i < N_VALUES; i++){
      Serial.print( values[i] );
      Serial.print( ' ' );
    } 
    Serial.println();
    //delay(200);
  }
}

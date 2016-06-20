/*
 * This program recives N_VALUES of size N_BYTES with the following protocol
 *  to start '@''@' the values byte by byte and '.' to finish
 *  
 */
 #include "TimerOne.h"
 #include "math.h"
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

const double R = 30;
const double L = 100;

const int W_MAX = 13; //rads/s
const int W_MIN = 2;

const int V_MAX = 9; //rads/s
const int V_MIN = 2;
const int D = 400; // if error larger than D use V_MAX

const float KP = 5;
const float KP_V = float(V_MAX)/D;

const int MAX_ANGLE_ERROR = 5;
const int MAX_POS_ERROR = 10;

int x = 0;
int y = 0;
double theta = 0;

int xGoal = 1000;
int yGoal = 1000;
double thetaGoal = 0;

int distError = 0;
int angleError = 0;

void setup() {
  pinMode( rightPWM, OUTPUT ); //Defining pins as output
  pinMode( leftPWM, OUTPUT );
  pinMode( rightDir, OUTPUT );
  pinMode( leftDir, OUTPUT );
  pinMode( ledPin, OUTPUT ); 
  Serial.begin( 9600 );
  
  Timer1.initialize(50000); // 20Hz  --> 50000 microSecs period 
  Timer1.attachInterrupt( control );
  digitalWrite( ledPin, HIGH );
  analogWrite( rightPWM, 0 );
  analogWrite( leftPWM, 0 );
}

void setVelocity( int pwmPin, int dirPin, double w ){ // pwm_pin, direction_pin, velocity
  if(DEBUG)Serial.println("Seting Velocity");
  if( w < 0 )
    digitalWrite( dirPin, LOW );
  else
    digitalWrite( dirPin, HIGH );
  w = abs( w );
  int w_pwm = 0.2154 * w * R + 64.02; 
  if( w_pwm < 65 )
    w_pwm = 0;
  
  analogWrite( pwmPin, w_pwm );
  if(DEBUG)Serial.println("Velocity Set");
}
void control()
{
  double d_x = xGoal - x;
  double d_y = yGoal - y;
  double norm = sqrt( d_x*d_x + d_y*d_y );
  double dotProduct = d_x*-1*sin(theta) + d_y*cos(theta);
  double angle_error = acos( dotProduct/norm );

  double dotProduct2 = d_x*-1*sin(theta+0.01) + d_y*cos(theta+0.01);
  double next_error = acos( dotProduct2/norm );
  if( next_error > angle_error )
    angle_error *= -1; 
  double w_r = angle_error*KP;

  if( w_r > W_MAX )
    w_r = W_MAX;
  else{
    if( w_r < -1*W_MAX )
       w_r = -1*W_MAX;
    else{
      if( w_r < W_MIN && w_r > -1*W_MIN ){
        if( w_r > 0 )
          w_r = W_MIN;
        else
          w_r = -1*W_MIN;
      }
    }
  }
  if( angle_error *180/M_PI > -1*MAX_ANGLE_ERROR && angle_error *180/M_PI < MAX_ANGLE_ERROR)
    w_r = 0;
  double w_l = -1*w_r; 
  double s = KP_V * norm;
   if( s > V_MAX )
    s = V_MAX;
  else{
    if( s < -1*V_MAX )
       s = -1*V_MAX;
    else{
      if( s < V_MIN && s > -1*V_MIN ){
        if( s > 0 )
          s = V_MIN;
        else
          s = -1*V_MIN;
      }
    }
  }
  w_r += s;
  w_l += s; 
  if( norm < MAX_POS_ERROR && norm > -1*MAX_POS_ERROR ){
    w_r = 0;
    w_l = 0;
  }
  setVelocity( rightPWM, rightDir, w_r );
  setVelocity( leftPWM, leftDir, w_l );
}
void setPose( int newX, int newY, int newTheta ){
  x = newX;
  y = newY;
  theta = newTheta*M_PI/180;   
}

void setGoal( int newX, int newY, int newTheta ){
  xGoal = newX;
  yGoal = newY;
  thetaGoal = newTheta*M_PI/180;   
}


void loop() {   
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

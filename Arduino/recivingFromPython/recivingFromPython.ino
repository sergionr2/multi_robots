/*
 * This program recives N_VALUES of size N_BYTES with the following protocol
 *  to start '@''@' the values byte by byte and '.' to finish
 *  Then writes an analog value from 0 to 255 for each value recived
 *  NOTE: the convention is set to go forward when the direction variables "rightDir" and "leftDir" are HIGH
 *  and go backwards when they are LOW
 */
const byte N_BYTES = 2; // size of values recived
const byte N_VALUES = 2; // number of values to recive
byte incomingData[ N_BYTES*N_VALUES + 1 ]; // +1 end character

const char START_CHAR = '@';
const char END_CHAR = '.';

int values[N_VALUES]; // values[0] = velocity_right,  //CHECK SIZE IF RECIVING BIGGER DATA, now: int16
                     // values[1] = velocity_left,  
const bool DEBUG = false;
bool firstStart = false;
int incomingByte;      // a variable to read incoming serial data into

const int rightVel = 3; // pins PWM A
const int leftVel = 11; // PWM B

const int rightDir = 12; // DIR A
const int leftDir = 13; // DIR B

const int ledPin = 10; // LED

void setup() {
  pinMode( rightVel, OUTPUT ); //Defining pins as output
  pinMode( leftVel, OUTPUT );
  pinMode( rightDir, OUTPUT );
  pinMode( leftDir, OUTPUT );
  pinMode( ledPin, OUTPUT ); 

  Serial.begin( 9600 );
  for( int i = 0; i < N_VALUES; i++) // initialize values to 0
    values[ i ] = 0;
  digitalWrite( ledPin, HIGH );
  analogWrite( rightVel, 0 );
  analogWrite( leftVel, 0 );
}

byte velToOmega( int vel ){
  byte omega = 0; // value from 0 to 255
  
  return 200; //TODO
}

void setVelocity( int velPin, int dirPin, int vel ){ // velocity_pin, direction_pin, velocity
  if(DEBUG)Serial.println("Seting Velocity");
  if( vel < 0 )
    digitalWrite( dirPin, LOW );
  else
    digitalWrite( dirPin, HIGH );
  vel = abs( vel );
  analogWrite( velPin, velToOmega( vel ) ); // OUTPUT from 0 to 255
  if(DEBUG)Serial.println("Velocity Set");
}

// protocol ( start start V_r V_l end )
void loop() {  
  if ( Serial.available() > 0 && !firstStart) {
    incomingByte = Serial.read();
    if( incomingByte == START_CHAR){
      firstStart = true; // if ---> @
      if(DEBUG)Serial.println("in @");
    }
  }
  if (Serial.available() > 0 && firstStart ){
    incomingByte = Serial.read();  
    if( incomingByte == START_CHAR && firstStart ){ // if START ---> @ @
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
        for( int i = 0; i < N_VALUES; i++)
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
        setVelocity( rightVel, rightDir, values[0] );
        setVelocity( leftVel, leftDir, values[1] );
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

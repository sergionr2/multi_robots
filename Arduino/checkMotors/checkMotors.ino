
const int rightPWM = 3; // pins PWM A
const int leftPWM = 11; // PWM B

const int rightDir = 12; // DIR A
const int leftDir = 13; // DIR B

const int ledPin = 10; // Led

void setup() {
  pinMode( rightPWM, OUTPUT ); //Defining pins as output
  pinMode( leftPWM, OUTPUT );
  pinMode( rightDir, OUTPUT );
  pinMode( leftDir, OUTPUT );
  pinMode( ledPin, OUTPUT ); 

  digitalWrite( ledPin, HIGH );
  analogWrite( rightPWM, 255 );
  analogWrite( leftPWM, 255 );
}

void loop() {   
    digitalWrite( leftDir, HIGH );
    digitalWrite( rightDir, HIGH );
    delay(2000);
    digitalWrite( leftDir, HIGH );
    digitalWrite( rightDir, LOW );
    delay(2000);
    digitalWrite( leftDir, LOW );
    digitalWrite( rightDir, LOW );
    delay(2000);
    digitalWrite( leftDir, LOW );
    digitalWrite( rightDir, HIGH );
    delay(2000);
    
}

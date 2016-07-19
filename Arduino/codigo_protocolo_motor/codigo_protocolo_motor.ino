//Pins
const int DIRA = 12; //Connect on Pin 12
const int PWMA = 3; //Connect on Pin 3
const int DIRB = 13; //Connect on Pin 13
const int PWMB = 11; //Connect on Pin 11
const int ledPin = 10;
//Communication protocol checking variables

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
  
  Serial.begin(9600);

  Serial.println("SETUP OK");

  
}

void loop(){
  digitalWrite(ledPin, HIGH);

  Serial.print("action = ");
  Serial.println(action);
  Serial.print("power = ");
  Serial.println((byte)power);
  Serial.println("-----");
  
  if ( Serial.available() > 0 ){    
    income = Serial.read();


   if(income == 'z' || income == 'q' || income == 's' || income == 'd' || income == 'a'){
      action = income;
      Serial.println("ACTION CHECKED!!!");
   }
    else if(income >= '0' && income <= '9'){
      power = income;
      power = (power - '0')*25 + 25;
      Serial.println("POWER CHECKED!!!");
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

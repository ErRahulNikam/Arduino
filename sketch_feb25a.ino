#define leftDIR 2     //leftDIR = LOW FOR FORWARD & leftDIR = HIGH FOR BACKWARD
#define leftPWM 3     //THIS PIN IS TO DECIDE THE SPEED OF LEFT MOTOR
#define rightDIR 4    //rightDIR = LOW FOR FORWARD & rightDIR = HIGH FOR BACKWARD
#define rightPWM 5    //THIS PIN IS TO DECIDE THE SPEED OF RIGHT MOTOR

#define Kp 5  //2
#define Ki 0.0001  //0.0001
#define Kd 4 //4

void runMotor(int ,int);
void sensorRead();
void findMid();
void detectBW();
void displayData();
int analogValue();

unsigned int maxSensor = 5;
unsigned int sensor[5];
unsigned int s[5];
unsigned int readVal = 0;
int Mid = 0;
int sensorHIGH = 0;
int sensorLOW = 1023;

int maxV = 150;
int right_speed, left_speed;
int positionValue;
int set_point = 35;
int proportional, integral = 0, derivative, last_proportional = 0, error_value;

void setup() {
  pinMode(leftDIR,OUTPUT);    //THIS PINS ARE USED FOR DRIVING THE MOTOR
  pinMode(leftPWM,OUTPUT);
  pinMode(rightDIR,OUTPUT);
  pinMode(rightPWM,OUTPUT);
  pinMode(13,OUTPUT);
  
  pinMode(sensor[0],INPUT);   //THIS PINS ARE USED FOR READ THE VALUE FROM SENSOR
  pinMode(sensor[1],INPUT);
  pinMode(sensor[2],INPUT);
  pinMode(sensor[3],INPUT);
  pinMode(sensor[4],INPUT);
  for(int t = 0; t < 1 ; t++){
    digitalWrite(13,HIGH);
    delay(300);
    digitalWrite(13,LOW);
    delay(700);
  }
  runMotor(100,100);
  
  Serial.begin(9600);
}

void loop() {
    readVal = analogValue();

    if(readVal == 140){
      runMotor(-200,200);
      delay(50);
    }
    
    positionValue = readVal;// ((float)readVal/921)*155;//155
    proportional = ((int)positionValue) - set_point;
    derivative = (proportional - last_proportional);
    integral = integral + proportional ;
    last_proportional = proportional;
    error_value = proportional * Kp + integral * Ki + derivative * Kd;
   
    if (error_value < -maxV) {
      error_value = -maxV;
    } if (error_value > maxV) {
      error_value = maxV;
    }
    // If error_value is less than zero calculate right turn speed values
    if (error_value < 0) {
      right_speed = maxV + error_value;
      left_speed = maxV;
    }
    // Iferror_value is greater than zero calculate left turn values
    else {
      right_speed = maxV;
      left_speed = maxV - error_value;
    }
    runMotor(left_speed, right_speed);  //Sends PWM signals to the motors
//  displayData();
}


void runMotor(int L, int R){
  if(L < 0)
    digitalWrite(leftDIR,HIGH);
  else
    digitalWrite(leftDIR,LOW);
  if(R < 0)
    digitalWrite(rightDIR,HIGH);
  else
    digitalWrite(rightDIR,LOW);
  analogWrite(leftPWM,L);
  analogWrite(rightPWM,R);
}

int analogValue(){
  sensorRead();
  findMid();
  detectBW();
  int lsa = 5;
  int pos = 0;

  for(int i = 0; i < maxSensor; i++){
    if(s[i] == 0)
    pos = pos + lsa;
    lsa = lsa * 2;
  }
  return(pos);
}

void sensorRead(){
  sensor[0] = analogRead(A0);
  sensor[1] = analogRead(A1);
  sensor[2] = analogRead(A2);
  sensor[3] = analogRead(A3);
  sensor[4] = analogRead(A4);
}

void findMid(){
  sensorHIGH = 0;
  sensorLOW = 1023;
  
  for(int i = 0; i < maxSensor; i++){
    if(sensor[i] > sensorHIGH)    //THIS CODICTIION IS TO DECIDE THE HIGHEST VALUE OF SENSOR
      sensorHIGH = sensor[i];
    if(sensor[i] < sensorLOW) //THIS CODICTIION IS TO DECIDE THE LOWEST VALUE OF SENSOR
      sensorLOW = sensor[i];
  }
  
  if(Mid == 0){
    Mid = (sensorHIGH + sensorLOW) / 2;  
  }
  else{
    if((sensor[0] > Mid && sensor[1] > Mid && sensor[2] > Mid && sensor[3] > Mid && sensor[4] > Mid) || (sensor[0] < Mid && sensor[1] < Mid && sensor[2] < Mid && sensor[3] < Mid && sensor[4] < Mid))
    Mid = Mid;
    else
    Mid = (sensorHIGH + sensorLOW) / 2;  
  }
}

void detectBW(){
  for(int i = 0; i < maxSensor; i++){   //THIS FOR LOOP USED TO DECIDE BLACK OR WHITE LINE
    if(sensor[i] >= Mid)
      s[i] = 1;            //THIS STATMENT SAYS THAT LINE IS WHITE
    else
      s[i] = 0;            //THIS STATMENT SAYS THAT LINE IS BLACK
  }
}


void displayData(){
  for(int i = 0; i < 5; i++){
    Serial.print(sensor[i]);
    Serial.print("\t");
  }

  Serial.print("sensorHIGH = ");
  Serial.print(sensorHIGH);
  Serial.print("\t");
  Serial.print("Sensor Ref(Mid) = ");
  Serial.print(Mid);
  Serial.print("\t");
  Serial.print("sensorLOW = ");
  Serial.print(sensorLOW);
  Serial.println();
  
  for(int i = 0; i < 5; i++){
    Serial.print(s[i]);
    Serial.print("\t");
  }
  Serial.println();
  Serial.println();
  Serial.print("readVal = ");
  Serial.print(readVal);
  Serial.print("\t");
  Serial.print("proportional = ");
  Serial.print(proportional);
  Serial.print("\t");
  Serial.print("integral = ");
  Serial.print(integral);
  Serial.print("\t");
  Serial.print("derivative = ");
  Serial.print(derivative);
  Serial.print("\t");
  Serial.print("error_value = ");
  Serial.print(error_value);
  Serial.print("\t");
  Serial.print("left_speed = ");
  Serial.print(left_speed);
  Serial.print("\t");
  Serial.print("right_speed = ");
  Serial.print(right_speed);
  Serial.print("\t");
  Serial.println();
  Serial.println();
  delay(500);
}

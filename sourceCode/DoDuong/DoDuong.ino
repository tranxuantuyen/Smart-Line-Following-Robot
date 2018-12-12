#define led 13

#define leapTime 150

#define pWM1               9
#define pWM2               10
#define leftMotor1         4
#define leftMotor2         5
#define rightMotor1        6
#define rightMotor2        7

float Kp = 10, Ki = 0.5, Kd = 3;             // He so cu 10,0,7
float P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
int sensor[5] = {0, 0, 0, 0, 0};
int initial_motor_speed = 24;
int error = 0;

char path[100] = {};
int pathLength;
int readLength=0;
int replaystage;

void read_sensor_values(void);
void calculate_pid(void);
void motor_control(void);
void stop_car(void);
void dithang(void);

void setup()
{
  pinMode(9, OUTPUT); //PWM Pin 1
  pinMode(10, OUTPUT); //PWM Pin 2
  TCCR1B = (TCCR1B & 0xF8) | 5; //set PWM frequency to 112Hz for pin 9 and 10
  pinMode(4, OUTPUT); //Left Motor Pin 1
  pinMode(5, OUTPUT); //Left Motor Pin 2
  pinMode(6, OUTPUT); //Right Motor Pin 1
  pinMode(7, OUTPUT); //Right Motor Pin 2
  pinMode(led, OUTPUT);
  Serial.begin(9600); //Enable Serial Communications
}

void loop()
{
  read_sensor_values();
  calculate_pid();

  if(sensor[0]==1 && sensor[2]==0 && sensor[4]==1){
    motor_control();
    }
  else if(sensor[0]==0 || sensor[4]==0 || error == 100){
    leftHandWall();
    }
}

void leftHandWall(){
  if(sensor[0]==0 && sensor[4]==0){
    //Serial.print("Nga ba");
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, HIGH);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, HIGH);
    analogWrite(pWM1,initial_motor_speed+30);   //Left Motor Speed
    analogWrite(pWM2,initial_motor_speed+30);  //Right Motor Speed
    delay(leapTime);
    analogWrite(pWM1,0);   //Left Motor Speed
    analogWrite(pWM2,0);  //Right Motor Speed
    delay(100);
    read_sensor_values();
    if(sensor[0]==0 && sensor[2]==0 && sensor[4] == 0){
      //Serial.println("Done");
      done();
      }
    else {
      //Serial.println("Turn Left");
      turnLeft();
      return;
      }
    }

   if(sensor[0]==0){
    //Serial.print("Nga re trai ");
          digitalWrite(leftMotor1, LOW);
          digitalWrite(leftMotor2, HIGH);
          digitalWrite(rightMotor1, LOW);
          digitalWrite(rightMotor2, HIGH);
          analogWrite(pWM1,initial_motor_speed+30);   //Left Motor Speed
          analogWrite(pWM2,initial_motor_speed+30);  //Right Motor Speed
          delay(leapTime);
          analogWrite(pWM1,0);   //Left Motor Speed
          analogWrite(pWM2,0);  //Right Motor Speed
          delay(100);
          read_sensor_values();
    if(sensor[0]==0 && sensor[2]==0 && sensor[4] == 0){
      //Serial.println("Done");
      done();
      }
    else {
      //Serial.println("Turn Left");
      turnLeft();
      }
      }

    if(sensor[4] == 0){
      //Serial.print("Nga re phai ");
          digitalWrite(leftMotor1, LOW);
          digitalWrite(leftMotor2, HIGH);
          digitalWrite(rightMotor1, LOW);
          digitalWrite(rightMotor2, HIGH);
          analogWrite(pWM1,initial_motor_speed+30);   //Left Motor Speed
          analogWrite(pWM2,initial_motor_speed+30);  //Right Motor Speed
          delay(20);
          analogWrite(pWM1,0);   //Left Motor Speed
          analogWrite(pWM2,0);  //Right Motor Speed
          delay(300);
          read_sensor_values();
          if(sensor[0]==0){
            analogWrite(pWM1,initial_motor_speed+30);   //Left Motor Speed
            analogWrite(pWM2,initial_motor_speed+30);  //Right Motor Speed
            delay(leapTime-50);
            turnLeft();
            return;
//            analogWrite(pWM1,0);   //Left Motor Speed
//            analogWrite(pWM2,0);  //Right Motor Speed
//            delay(100);
//            if (sensor[0]==0 && sensor[2]==0 && sensor[4]==0) {
//              //Serial.println("Done");
//            done();
//            }
//            else {
//              //Serial.println("Turn Left");
//              turnLeft();
//              return;
//            }
            
          }

          analogWrite(pWM1,initial_motor_speed+30);   //Left Motor Speed
          analogWrite(pWM2,initial_motor_speed+30);  //Right Motor Speed
          delay(100);
          read_sensor_values();
          if(sensor[0]==1 && sensor[2]==1 && sensor[4]==1){
            //Serial.println("Turn Right");
            turnRight();
            return;
          }
          path[pathLength] = 'S';
          // Serial.println("s");
          pathLength++;
          //Serial.print("Path length: ");
          //Serial.println(pathLength);
          if (path[pathLength - 2] == 'B') {
          //Serial.println("shortening path");
          shortPath();
          }
          //Serial.println("Go straight");
          analogWrite(pWM1,initial_motor_speed);   //Left Motor Speed
          analogWrite(pWM2,initial_motor_speed);  //Right Motor Speed
          delay(100);
          
      }

    if(sensor[0]==1 && sensor[1]==1 && sensor[2]==1 && sensor[3]==1 && sensor[4]==1){
    turnAround();
    }
  }
void turnLeft(){

    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, HIGH);
    analogWrite(pWM1,initial_motor_speed+15);   //Left Motor Speed
    analogWrite(pWM2,initial_motor_speed+15);  //Right Motor Speed
    delay(400);
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
    
  while(digitalRead(A2)){
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, HIGH);
    analogWrite(pWM1,initial_motor_speed+15);   //Left Motor Speed
    analogWrite(pWM2,initial_motor_speed+15);  //Right Motor Speed
    delay(2);
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
    delay(1);
  }

  if(replaystage==0){
    path[pathLength]='L';
    //Serial.println("l");
    pathLength++;
    //Serial.print("Path length: ");
    //Serial.println(pathLength);
      if(path[pathLength-2]=='B'){
        //Serial.println("shortening path");
        shortPath();
      }
  }
}

void turnRight(){

//    digitalWrite(leftMotor1, LOW);
//    digitalWrite(leftMotor2, HIGH);
//    digitalWrite(rightMotor1, LOW);
//    digitalWrite(rightMotor2, HIGH);
//    analogWrite(pWM1,initial_motor_speed+15);   //Left Motor Speed
//    analogWrite(pWM2,initial_motor_speed+15);  //Right Motor Speed
//    delay(leapTime);
    
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, HIGH);
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
    analogWrite(pWM1,initial_motor_speed+15);   //Left Motor Speed
    analogWrite(pWM2,initial_motor_speed+15);  //Right Motor Speed
    delay(400);
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
    
    while(digitalRead(A2)){
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, HIGH);
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
    analogWrite(pWM1,initial_motor_speed+15);   //Left Motor Speed
    analogWrite(pWM2,initial_motor_speed+15);  //Right Motor Speed
    delay(2);
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
    delay(1);
  }
  
  if(replaystage==0){
  path[pathLength]='R';
  //Serial.println("r");
  pathLength++;
  //Serial.print("Path length: ");
  //Serial.println(pathLength);
    if(path[pathLength-2]=='B'){
      //Serial.println("shortening path");
      shortPath();
    }
  }
 
}
void read_sensor_values()
{
  sensor[0] = digitalRead(A0);
  sensor[1] = digitalRead(A1);
  sensor[2] = digitalRead(A2);
  sensor[3] = digitalRead(A3);
  sensor[4] = digitalRead(11);

//  if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0))
//    error = 0;
//  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
//    error = 0;
  if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 1))
    error = 2;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1))
    error = 1;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1))
    error = 0;
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1))
    error = -1;
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))
    error = -2;
//  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))
//    error = 0;
//  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))
//    error = 0;
//  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))
//    if (error <= 0) error = -5;
//    else error = 5;

  //Serial.println(error);
  if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) error =100;
}

void calculate_pid()
{
  P = error;
  I = I + previous_I;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_I = I;
  previous_error = error;
}

void motor_control()
{
  // Calculating the effective motor speed:
  int left_motor_speed = initial_motor_speed - PID_value;
  int right_motor_speed = initial_motor_speed + PID_value;

  // The motor speed should not exceed the max PWM value
  left_motor_speed = constrain(left_motor_speed, 0, 55);
  right_motor_speed = constrain(right_motor_speed, 0, 55);

  //    Serial.print("Left: ");Serial.print(left_motor_speed);
  //    Serial.print(" Right: ");Serial.println(right_motor_speed);
  analogWrite(pWM1, left_motor_speed);  //Left Motor Speed
  analogWrite(pWM2, right_motor_speed); //Right Motor Speed
  //following lines of code are to make the bot move forward
  /*The pin numbers and high, low values might be different
    depending on your connections */
  digitalWrite(4, LOW);
  digitalWrite(5, HIGH);
  digitalWrite(6, LOW);
  digitalWrite(7, HIGH);
}

void turnAround(){
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, HIGH);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, HIGH);
    analogWrite(pWM1,initial_motor_speed);   //Left Motor Speed
    analogWrite(pWM2,initial_motor_speed);  //Right Motor Speed
    delay(150);
    while(digitalRead(A2)){
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, HIGH);
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
    analogWrite(pWM1,initial_motor_speed+30);   //Left Motor Speed
    analogWrite(pWM2,initial_motor_speed+30);  //Right Motor Speed
    delay(2);
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
    delay(1);
    }
  path[pathLength]='B';
  pathLength++;
  motor_control();
//  Serial.println("b");
//  Serial.print("Path length: ");
//  Serial.println(pathLength);
}

void shortPath() {
  int shortDone = 0;
  if (path[pathLength - 3] == 'L' && path[pathLength - 1] == 'R') {
    pathLength -= 3;
    path[pathLength] = 'B';
    //Serial.println("test1");
    shortDone = 1;
  }

  if (path[pathLength - 3] == 'L' && path[pathLength - 1] == 'S' && shortDone == 0) {
    pathLength -= 3;
    path[pathLength] = 'R';
    //Serial.println("test2");
    shortDone = 1;
  }

  if (path[pathLength - 3] == 'R' && path[pathLength - 1] == 'L' && shortDone == 0) {
    pathLength -= 3;
    path[pathLength] = 'B';
    //Serial.println("test3");
    shortDone = 1;
  }


  if (path[pathLength - 3] == 'S' && path[pathLength - 1] == 'L' && shortDone == 0) {
    pathLength -= 3;
    path[pathLength] = 'R';
    //Serial.println("test4");
    shortDone = 1;
  }

  if (path[pathLength - 3] == 'S' && path[pathLength - 1] == 'S' && shortDone == 0) {
    pathLength -= 3;
    path[pathLength] = 'B';
    //Serial.println("test5");
    shortDone = 1;
  }
  if (path[pathLength - 3] == 'L' && path[pathLength - 1] == 'L' && shortDone == 0) {
    pathLength -= 3;
    path[pathLength] = 'S';
    //Serial.println("test6");
    shortDone = 1;
  }

  path[pathLength + 1] = 'D';
  path[pathLength + 2] = 'D';
  pathLength++;
  //Serial.print("Path length: ");
  //Serial.println(pathLength);
  //printPath();
}

void replay() {
  read_sensor_values();
  calculate_pid();

  if(sensor[0]==1 && sensor[2]==0 && sensor[4]==1){
    motor_control();
    }
  else if(sensor[0]==0 || sensor[4]==0){
    if (path[readLength] == 'D') {
      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, HIGH);
      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, HIGH);
      analogWrite(pWM1,initial_motor_speed+15);   //Left Motor Speed
    analogWrite(pWM2,initial_motor_speed+15);  //Right Motor Speed
      delay(100);
      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, LOW);
      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, LOW);
      endMotion();
    }
    if (path[readLength] == 'L') {
      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, HIGH);
      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, HIGH);
      analogWrite(pWM1,initial_motor_speed+15);   //Left Motor Speed
    analogWrite(pWM2,initial_motor_speed+15);  //Right Motor Speed
      delay(leapTime);
      turnLeft();
    }
    if (path[readLength] == 'R') {
      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, HIGH);
      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, HIGH);
      analogWrite(pWM1,initial_motor_speed+15);   //Left Motor Speed
    analogWrite(pWM2,initial_motor_speed+15);  //Right Motor Speed
      delay(leapTime);
      turnRight();
    }
    if (path[readLength] == 'S') {
      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, HIGH);
      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, HIGH);
      delay(leapTime);
      analogWrite(pWM1,initial_motor_speed);   //Left Motor Speed
      analogWrite(pWM2,initial_motor_speed);  //Right Motor Speed
      delay(100);
    }

    readLength++;
  }

  replay();

}

void done() {
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
  replaystage = 1;
  path[pathLength] = 'D';
  pathLength++;
  while (digitalRead(A0)==0) {
    digitalWrite(led, LOW);
    delay(500);
    digitalWrite(led, HIGH);
    delay(500);
  }
  delay(5000);
  replay();
}

void endMotion() {
  digitalWrite(led, LOW);
  delay(500);
  digitalWrite(led, HIGH);
  delay(200);
  digitalWrite(led, LOW);
  delay(200);
  digitalWrite(led, HIGH);
  delay(500);
  endMotion();
}

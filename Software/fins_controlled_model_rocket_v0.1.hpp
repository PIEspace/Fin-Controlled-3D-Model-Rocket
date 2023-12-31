
#include<Arduino.h>
#include<Wire.h>
#include<Adafruit_Sensor.h>
#include<Adafruit_MPU6050.h>
#include<Servo.h>

// create a mpu6050 object 
Adafruit_MPU6050 mpu6050_sensor;

// define the servo 
Servo first_servo;
Servo second_servo;
Servo third_servo;
Servo four_servo;

const int first_servo_pin= 2;
const int second_servo_pin = 3;
const int third_servo_pin = 4;
const int four_servo_pin = 5;

// define the buzzer 
const int buzzer = A0;

// define the millis function 
unsigned long Previous_time = 0;
const long Delay_time = 1000; //delay for 1 second 

// define the global variable accelerationX , accelerationY , accelerationZ 
float AccX ;
float AccY ;
float AccZ ;


// check the mpu6050 sensor both accelerometer and gyroscope
void check_mpu6050(){
  sensors_event_t a , g , temp ; //define the accelerometer , gyroscope , temperature
  mpu6050_sensor . getEvent(&a , &g , &temp);

  // define the accelerometer 
  float accelerometerX = a.acceleration.x; //define the acceleration x 
  float accelerometerY = a.acceleration.y; //define the acceleration y 
  float accelerometerZ = a.acceleration.z; //define the acceleration z

  // define the gyroscope 
  float GyroX = g.gyro.x; //define the gyro x 
  float GyroY = g.gyro.y; //define the gyro y
  float GyroZ = g.gyro.z; //define the gyro z 

  // print on the serial monitor 
  Serial.print("The value of acceleration [X,Y,Z]");
  Serial.print(accelerometerX);
  Serial.print(",");
  Serial.print(accelerometerY);
  Serial.print(",");
  Serial.print(accelerometerZ);
  Serial.print("==");
  Serial.print("The value of gyroscope [X,Y,Z]");
  Serial.print(GyroX);
  Serial.print(",");
  Serial.print(GyroY);
  Serial.print(",");
  Serial.println(GyroZ);

  delay(100);
}

void check_servo_buzzer(){
  tone(buzzer , 440);
  delay(100); //delay for 0.1 second 
  noTone(buzzer );
  delay(100); //delay for 0.1 second 
}

// check the servo motor
void servo_check(){
  // first servo check 
  Serial.println("Check first servo motor");
  first_servo.write(80); //curren-position of servo is 90 degree 
  delay(300);//delay for 300 millisecond 
  first_servo.write(0);//it move 90 degree to 0 degree
  delay(300); //delay for 300 millisecond
  first_servo.write(80); //it move 0 degree to 90 degree
  delay(300);//delay for 300 millisecond 
  first_servo.write(180);//it move 90 degree to 180 degree
  delay(300); //delay for 300 millisecond 
  first_servo.write(80); //it move 180 to 90 degree
  delay(300);
  // tone for buzzer 
  check_servo_buzzer();

  Serial.println("DONE............1");

  // define the second servo motor 
  Serial.println("Check second servo motor");
  second_servo.write(90); //curren-position of servo is 90 degree 
  delay(300);//delay for 300 millisecond 
  second_servo.write(0);//it move 90 degree to 0 degree
  delay(300); //delay for 300 millisecond
  second_servo.write(90); //it move 0 degree to 90 degree
  delay(300);//delay for 300 millisecond 
  second_servo.write(180);//it move 90 degree to 180 degree
  delay(300); //delay for 300 millisecond 
  second_servo.write(90); //it move 180 to 90 degree
  delay(300);
  // tone for buzzer 
  check_servo_buzzer();
  Serial.println("DONE............2");


  // define the third servo motor 
  Serial.println("Check third servo motor");
  third_servo.write(100); //curren-position of servo is 90 degree 
  delay(300);//delay for 300 millisecond 
  third_servo.write(0);//it move 90 degree to 0 degree
  delay(300); //delay for 300 millisecond
  third_servo.write(100); //it move 0 degree to 90 degree
  delay(300);//delay for 300 millisecond 
  third_servo.write(180);//it move 90 degree to 180 degree
  delay(300); //delay for 300 millisecond 
  third_servo.write(100); //it move 180 to 90 degree
  delay(300);
  // tone for buzzer 
  check_servo_buzzer();

  Serial.println("DONE............3");


  // define the four  servo motor 
  Serial.println("Check four servo motor");
  four_servo.write(90); //curren-position of servo is 90 degree 
  delay(300);//delay for 300 millisecond 
  four_servo.write(0);//it move 90 degree to 0 degree
  delay(300); //delay for 300 millisecond
  four_servo.write(90); //it move 0 degree to 90 degree
  delay(300);//delay for 300 millisecond 
  four_servo.write(180);//it move 90 degree to 180 degree
  delay(300); //delay for 300 millisecond 
  four_servo.write(90); //it move 180 to 90 degree
  delay(300);
  // tone for buzzer 
  check_servo_buzzer();
  Serial.println("DONE............4");
}

void welcome_weep(){

  for(int i = 0 ; i<=4 ; i++){
      // Play a guitar-like tone
  tone(buzzer, 440); // A4 (440 Hz)
  delay(100); // Play for 0.1 seconds
  noTone(buzzer); // Stop the tone
  delay(100); // Pause for 0.1 seconds

  // Play another note
  tone(buzzer, 523); // C5 (523 Hz)
  delay(100); // Play for 0.1 seconds
  noTone(buzzer); // Stop the tone
  delay(100); // Pause for 0.1 seconds

  }
}

// start program sound 
void start_program(){

  for(int start_tone = 0; start_tone<=3 ; start_tone++)
  {
  tone (buzzer , 422); //(422hz);
  delay(100); //play for 0.1second 
  noTone(buzzer); //stop the tone
  delay(100); //pause for 0.1 second 

  tone (buzzer , 522); //(422hz);
  delay(100); //play for 0.1second 
  noTone(buzzer); //stop the tone
  delay(100); //pause for 0.1 second 

  tone (buzzer , 400); //(422hz);
  delay(100); //play for 0.1second 
  noTone(buzzer); //stop the tone
  delay(100); //pause for 0.1 second 

  tone (buzzer , 300); //(422hz);
  delay(100); //play for 0.1second 
  noTone(buzzer); //stop the tone
  delay(100); //pause for 0.1 second 

  }
  


}
void setup(){
  Serial.begin(9600);

  if(!mpu6050_sensor.begin()){
    Serial.println("Not Connect The MPU6050 Sensor To The Board Please Check The Wire ");
    Serial.println("SCL = SCL");
    Serial.println("SDA = SDA");
    Serial.println("VCC = 5V");
    Serial.println("GND = GND");

    while(true);
  }

  // define the servo Pin_out
  first_servo.attach(first_servo_pin);
  second_servo.attach(second_servo_pin);
  third_servo.attach(third_servo_pin);
  four_servo.attach(four_servo_pin);

  // define the buzzer 
  pinMode(buzzer , OUTPUT);

  Serial.println("Check mpu6050 sensor ");
  delay(1000); //delay for 1 second 
  for(int a = 0  ; a<= 10 ; a++){
    check_mpu6050();
  }
  delay(1000); //delay  for 1 second 
  Serial.println("########## DONE ###########");
  welcome_weep();

  // check the servo motor
  servo_check();
  delay(100);//delay for 0.1 second
  start_program();
}

void main_program(){
    sensors_event_t a , g , temp;
    mpu6050_sensor.getEvent(&a, &g ,&temp);

    // define the accelerationX   roll
    // connect the servo first and third 

    float AccX = a.acceleration.x;

    float AngleX = map(AccX , -3 , 3 , 0 , 180);
    first_servo.write(AngleX);

    float AngleX_2 = map(AccX, -3 , 3 , 180 , 0);
    third_servo.write(AngleX_2);

    // define the accelerationY  pitch 
    // connect the servo second and four 

    float AccY = a.acceleration.y;

    float AngleY = map(AccY , -3 , 3 , 0 , 180);
    second_servo.write(AngleY);

    float AngleY_2 = map(AccY , -3 , 3 , 180 , 0);
    four_servo.write(AngleY_2);

    // define the accelerationZ yaw
    // connect the four servo motor

    float AccZ = a.acceleration.z;
    delay(10);
}

void serial_monitor(){
  unsigned long Current_time = millis();
  if(Current_time - Previous_time >=Delay_time){
    Previous_time = Current_time;

    sensors_event_t a , g , temp;
    mpu6050_sensor.getEvent(&a , &g , &temp);

    AccX = a.acceleration.x;
    AccY = a.acceleration.y;
    AccZ = a.acceleration.z;

    Serial.print("roll");
    Serial.print(AccX);
    Serial.print(",");
    Serial.print("pitch");
    Serial.print(AccY);
    Serial.print(",");
    Serial.print("yaw");
    Serial.println(AccZ);


  }

}

void loop(){

    main_program();
    serial_monitor();

}
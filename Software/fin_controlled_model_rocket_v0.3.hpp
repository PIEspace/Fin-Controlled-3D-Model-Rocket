// this is the main code of our fins control model rocket 
// code is publish on github 

#include<Arduino.h>
#include<Servo.h>
#include<FastLED.h>
#include<Wire.h>
#include<Adafruit_Sensor.h>
#include<Adafruit_MPU6050.h>


// define the servo motor object 
Servo First_Servo;
Servo Second_Servo;
Servo Third_Servo;
Servo Four_Servo;

// connect the servo motor to the arduino nano board 
const int First_Servo_Attach = 2; //first servo connect to arduino pin 2 
const int Second_Servo_Attach = 3; //second servo connect to arduino pin 3 
const int Third_Servo_Attach = 4 ; //third servo connect to arduino pin 4 
const int Four_Servo_Attach = 5; //four servo connect to arduino pin 5 

// all servo motor position is 90 degree
const int First_Servo_Position = 80; //first servo position is 80 degree 
const int Second_Servo_Position = 90; //second servo position is 90 degree
const int Third_Servo_Position = 100;  //third servo position is 100 degree
const int Four_Servo_Position = 95;  //four servo position is 95 degree

// define the stepper motor 
#define ENABLE_PIN 6 //DEFINE THE ENABLE PIN (ENA)
#define DIR_PIN 9 //define the direction pin (DIR)
#define PUL_PIN 10 //define the pulse pin (PUL)

// define the stepper motor per rotation 
const int STEP_PER_REV = 5000;
unsigned long previous_time = 0;
const long delay_time_millis = 10;

// define the buzzer 
const int BUZZER = A0;

// define the led 
// How many led in your strip?
#define NUM_LED 1
#define DATA_PIN A1
#define CLOCK_PIN 13
// Define the array of led
CRGB led[NUM_LED];

// define the mpu6050 sensor
Adafruit_MPU6050 mpu6050_sensor;


void ALL_SERVO_90_DEGREE(){
    First_Servo.write(First_Servo_Position);
    Second_Servo.write(Second_Servo_Position);
    Third_Servo.write(Third_Servo_Position);
    Four_Servo.write(Four_Servo_Position);
}

void SERVO_90_0_90_180_90_FIRST_SERVO(){
    Serial.println("Check the first servo motor ");
    // DEFINE ALL THE ANGLE 
    #define SERVO_ANGLE_90 80 //you can set the angle (90)
    #define SERVO_ANGLE_0 0  //you can set the angle (0)
    #define SERVO_ANGLE_180 180 //you can set the angle (180)

    // define the delay 
    const long FIRST_DELAY = 300;

    First_Servo.write(SERVO_ANGLE_90);
    delay(FIRST_DELAY);
    First_Servo.write(SERVO_ANGLE_0);
    delay(FIRST_DELAY);
    First_Servo.write(SERVO_ANGLE_90);
    delay(FIRST_DELAY);
    First_Servo.write(SERVO_ANGLE_180);
    delay(FIRST_DELAY);
    First_Servo.write(SERVO_ANGLE_90);
    delay(FIRST_DELAY);

    Serial.println("#################### DONE SERVO ONE ####################");
    tone(BUZZER , 480); //480Hz
    delay(100); //start tone for 0.1 second
    noTone(BUZZER);
    delay(100); //delay for 0.1 second 
    // define for led 


}

void SERVO_90_0_90_180_90_SECOND_SERVO(){
    Serial.println("Check the second servo motor ");
    // DEFINE ALL THE ANGLE 
    #define SERVO_ANGLE_90 90 //you can set the angle (90)
    #define SERVO_ANGLE_0 0  //you can set the angle (0)
    #define SERVO_ANGLE_180 180 //you can set the angle (180)

    // define the delay 
    const long FIRST_DELAY = 300;

    Second_Servo.write(SERVO_ANGLE_90);
    delay(FIRST_DELAY);
    Second_Servo.write(SERVO_ANGLE_0);
    delay(FIRST_DELAY);
    Second_Servo.write(SERVO_ANGLE_90);
    delay(FIRST_DELAY);
    Second_Servo.write(SERVO_ANGLE_180);
    delay(FIRST_DELAY);
    Second_Servo.write(SERVO_ANGLE_90);
    delay(FIRST_DELAY);

    Serial.println("#################### DONE SERVO TWO ####################");
    tone(BUZZER , 480); //480Hz
    delay(100); //start tone for 0.1 second
    noTone(BUZZER);
    delay(100); //delay for 0.1 second 

}

void SERVO_90_0_90_180_90_THIRD_SERVO(){
    Serial.println("Check the third servo motor ");
    // DEFINE ALL THE ANGLE 
    #define SERVO_ANGLE_90 100 //you can set the angle (90)
    #define SERVO_ANGLE_0 0  //you can set the angle (0)
    #define SERVO_ANGLE_180 180 //you can set the angle (180)

    // define the delay 
    const long FIRST_DELAY = 300;

    Third_Servo.write(SERVO_ANGLE_90);
    delay(FIRST_DELAY);
    Third_Servo.write(SERVO_ANGLE_0);
    delay(FIRST_DELAY);
    Third_Servo.write(SERVO_ANGLE_90);
    delay(FIRST_DELAY);
    Third_Servo.write(SERVO_ANGLE_180);
    delay(FIRST_DELAY);
    Third_Servo.write(SERVO_ANGLE_90);
    delay(FIRST_DELAY);

    Serial.println("#################### DONE SERVO THREE ####################");
    tone(BUZZER , 480); //480Hz
    delay(100); //start tone for 0.1 second
    noTone(BUZZER);
    delay(100); //delay for 0.1 second 

}

void SERVO_90_0_90_180_90_FOUR_SERVO(){
    Serial.println("Check the four servo motor ");
    // DEFINE ALL THE ANGLE 
    #define SERVO_ANGLE_90 95 //you can set the angle (90)
    #define SERVO_ANGLE_0 0  //you can set the angle (0)
    #define SERVO_ANGLE_180 180 //you can set the angle (180)

    // define the delay 
    const long FIRST_DELAY = 300;

    Four_Servo.write(SERVO_ANGLE_90);
    delay(FIRST_DELAY);
    Four_Servo.write(SERVO_ANGLE_0);
    delay(FIRST_DELAY);
    Four_Servo.write(SERVO_ANGLE_90);
    delay(FIRST_DELAY);
    Four_Servo.write(SERVO_ANGLE_180);
    delay(FIRST_DELAY);
    Four_Servo.write(SERVO_ANGLE_90);
    delay(FIRST_DELAY);

    Serial.println("#################### DONE SERVO FOUR ####################");
    tone(BUZZER , 480); //480Hz
    delay(100); //start tone for 0.1 second
    noTone(BUZZER);
    delay(100); //delay for 0.1 second 

}

void CHECK_STEPPER_MOTOR(){
    unsigned long current_time = millis();
    if(current_time - previous_time >= delay_time_millis ){
        previous_time = current_time;

        // rotate the stepper motor in clockwise 
        digitalWrite(DIR_PIN , HIGH);
        for(int rotate = 0; rotate <= STEP_PER_REV ; rotate++)
        {
            digitalWrite(PUL_PIN , HIGH);
            delayMicroseconds(500);
            digitalWrite(PUL_PIN , LOW);
            delayMicroseconds(500);
        }
    }

}

void START_TONE (){

    for(int buzzer_tone_start = 0 ; buzzer_tone_start <= 4 ; buzzer_tone_start++)
    {
    tone(BUZZER , 440); //(440Hz)
    delay(100); //tone for 0.1 second 
    noTone(BUZZER);
    delay(100); //delay for 0.1 second 

    tone(BUZZER , 540); //(540Hz)
    delay(100); //tone for 0.1 second 
    noTone(BUZZER);
    delay(100); //delay for 0.1 second 

    tone(BUZZER , 640); //(640Hz)
    delay(100); //tone for 0.1 second 
    noTone(BUZZER);
    delay(100); //delay for 0.1 second 
    }

}

void FAST_LED_CHECK(){

  // Turn the LED on, then pause
  led[0] = CRGB::Red;
  FastLED.show();
  delay(500);
  // Now turn the LED off, then pause
  led[0] = CRGB::Black;
  FastLED.show();
  delay(500);

}

// check the mpu6050sensor 
void MPU6050_DATA_CHECK(){
    sensors_event_t a , g  , temp;
    mpu6050_sensor.getEvent(&a , &g , &temp);

    float AccelerometerX = a.acceleration.x;
    float AccelerometerY = a.acceleration.y;
    float AccelerometerZ = a.acceleration.z;

    float GyroX = g.gyro.x;
    float GyroY = g.gyro.y;
    float GyroZ = g.gyro.z;

    // print on the serial monitor
    for (int  i = 0; i <= 10; i++)
    {
        Serial.print("The value of accelerometer is [X,Y,Z]");
        Serial.print(AccelerometerX);
        Serial.print(",");
        Serial.print(AccelerometerY);
        Serial.print(",");
        Serial.print(AccelerometerZ);
        Serial.print(",");
        Serial.print("The value of gyroscope is [X,Y,Z]");
        Serial.print(GyroX);
        Serial.print(",");
        Serial.print(GyroY);
        Serial.print(",");
        Serial.println(GyroZ);
    }

    delay(100); //delay for 0.1 second 
}

void MOVE_FOR_LOOP_SERVO_FIRST (){
    #define START_POSITION 0 
    #define END_POSITION 180
    const long delay_for_loop = 15; //15 millisecond 
     
    for(int move_for_loop_first_servo =START_POSITION  ; move_for_loop_first_servo <= END_POSITION ; move_for_loop_first_servo++)
    {
        First_Servo.write(move_for_loop_first_servo);
        Second_Servo.write(move_for_loop_first_servo);
        Third_Servo.write(move_for_loop_first_servo);
        Four_Servo.write(move_for_loop_first_servo);
        delay(delay_for_loop);
    }


    for(int move_for_loop_first_servo = END_POSITION ; move_for_loop_first_servo >= START_POSITION ; move_for_loop_first_servo--)
    {
        First_Servo.write(move_for_loop_first_servo);
        Second_Servo.write(move_for_loop_first_servo);
        Third_Servo.write(move_for_loop_first_servo);
        Four_Servo.write(move_for_loop_first_servo);
        delay(delay_for_loop);
    }

    // all servo will we 90 degree
    First_Servo.write(90);
    Second_Servo.write(90);
    Third_Servo.write(90);
    Four_Servo.write(90);

    delay(500); //delay for 500 millisecond 

    #define START_POSITION_A 0
    #define END_POSITION_A 180

    for(int a = START_POSITION_A ; a <=END_POSITION_A ; a++ ){
        First_Servo.write(a);
        Second_Servo.write(a);
        Third_Servo.write(a);
        Four_Servo.write(a);
        delay(10);
    }

    for(int a = END_POSITION_A ; a >=START_POSITION_A ; a-- ){
        First_Servo.write(a);
        Second_Servo.write(a);
        Third_Servo.write(a);
        Four_Servo.write(a);
        delay(10);
    }

    First_Servo.write(90);
    Second_Servo.write(90);
    Third_Servo.write(90);
    Four_Servo.write(90);   
}

void setup(){
    // start tone 
    START_TONE ();
    // define the serial monitor bud rate 
    Serial.begin(9600);
    // define the servo motor
    First_Servo.attach(First_Servo_Attach);
    Second_Servo.attach(Second_Servo_Attach);
    Third_Servo.attach(Third_Servo_Attach);
    Four_Servo.attach(Four_Servo_Attach);

    // all servo angle will we 90 degree 
    ALL_SERVO_90_DEGREE();
    delay(1000);
    SERVO_90_0_90_180_90_FIRST_SERVO();
    SERVO_90_0_90_180_90_SECOND_SERVO();
    SERVO_90_0_90_180_90_THIRD_SERVO();
    SERVO_90_0_90_180_90_FOUR_SERVO();

    // define the PB6600 stepper motor driver 
    pinMode(ENABLE_PIN , OUTPUT);
    pinMode(DIR_PIN , OUTPUT);
    pinMode(PUL_PIN , OUTPUT);
    // enable the motor driver 
    digitalWrite(ENABLE_PIN , LOW);


    // for loop servo check 
    MOVE_FOR_LOOP_SERVO_FIRST ();
    
    // check the stepper motor 
    CHECK_STEPPER_MOTOR();

    // define the buzzer 
    pinMode(BUZZER , OUTPUT);

    // define the mpu6050sensor
    if(!mpu6050_sensor.begin()){
        Serial.println("Not connect the mpu6050 sensor to arduino please check the wire ");
        Serial.println("SCL == SCL");
        Serial.println("SDA == SDA");
        Serial.println("vcc == 5v");
        Serial.println("GND == GND ");

        while (true);
    }

      mpu6050_sensor.setAccelerometerRange(MPU6050_RANGE_16_G);
      mpu6050_sensor.setGyroRange(MPU6050_RANGE_250_DEG);

    // check mpu6050 sensor 
    MPU6050_DATA_CHECK();

    // define the fast led 
    FastLED.addLeds<NEOPIXEL, DATA_PIN>(led, NUM_LED);
    FAST_LED_CHECK();
}

void MAIN_MPU6050_PROGRAM(){
    sensors_event_t a , g , temp;
    mpu6050_sensor.getEvent(&a , &g , &temp);

    // float roll = atan2(accelerationY , accelerationZ)*180.0/PI;
    // float pitch = atan(-accelerationX/sqrt(accelerationY*accelerationY+accelerationZ*accelerationZ))*180.0/PI;
      // Calculate yaw acceleration
//   float yawAcceleration = atan2(a.acceleration.y, a.acceleration.x) * 180 / PI;

    float AccX = a.acceleration.x;
    // define the first servo motor 
    float angle_one = map(AccX , -3 , 3 , 0 , 180);
    First_Servo.write(angle_one);
    // define the second servo motor 
    float angle_two = map(AccX , -3 , 3 , 180 , 0 );
    Third_Servo.write(angle_two);

    // define the acc
    float AccY = a.acceleration.y;
    // define the third servo motor 
    float angle_three = map(AccY , -3 , 3 , 0 , 180);
    Second_Servo.write(angle_three);
    // define the four servo motor 
    float angle_four = map(AccY , -3 , 3 , 180 , 0 );
    Four_Servo.write(angle_four);
}

// void CONT_RUN_STEPPER (){

//     // enable the motor driver 
//     digitalWrite(ENABLE_PIN , HIGH);
//         // rotate the stepper motor in clockwise 
//         digitalWrite(DIR_PIN , HIGH);
//         // for(int rotate = 0; rotate <= STEP_PER_REV ; rotate++)
//         // {
//             digitalWrite(PUL_PIN , HIGH);
//             delayMicroseconds(500);
//             digitalWrite(PUL_PIN , LOW);
//             delayMicroseconds(500);
//         // }
// }

void loop(){
    // execute the function 
    MAIN_MPU6050_PROGRAM();
    // CONT_RUN_STEPPER ();
}
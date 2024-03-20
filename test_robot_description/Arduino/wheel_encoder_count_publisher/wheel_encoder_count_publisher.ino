// Execute "rosrun rosserial_python serial_node.py" in the terminal after starting roscore, 
// but before checking for the rostopics.

// Libraries
#include <ros.h>
#include <std_msgs/Int16.h>

// Handles startup and shutdown of ROS
ros::NodeHandle nh;

// GPIOs for Cytron MDD10A Motor driver
// Left Motor PWM
#define pwm_1 6
#define dir_1 7
int rpm_1 = 0;  
// Right Motor PWM  
#define pwm_2 8
#define dir_2 9
int rpm_2 = 0;    
// Encoder pins
#define enc_1_A 2 // Interrupt
#define enc_1_B 4
#define enc_2_A 3 // Interrupt
#define enc_2_B 5

// Encoder Parameters
int gear_ratio = 90;
int cpr = 3960;
int multiplication_factor = cpr / 2; 

// Keep track of the number of wheel counts
std_msgs::Int16 count_pulses_1;
ros::Publisher leftPub("left_count", &count_pulses_1);

std_msgs::Int16 count_pulses_2;
ros::Publisher rightPub("right_count", &count_pulses_2);

// 100ms interval for measurements
const int interval = 100;
long previousMillis = 0;
long currentMillis = 0;

void setup() {
  // put your setup code here, to run once:
  // Drive motor and their pins en -> enable/pwm , in -> direction
  pinMode(pwm_1, OUTPUT);
  pinMode(dir_1, OUTPUT);
  pinMode(pwm_2, OUTPUT);
  pinMode(dir_2, OUTPUT);

  pinMode(enc_1_A, INPUT_PULLUP); // sets the Encoder_output_A pin as the input
  pinMode(enc_1_B, INPUT); // sets the Encoder_output_B pin as the input
  attachInterrupt(digitalPinToInterrupt(enc_1_A), DC_Motor_Encoder_1, RISING);

  pinMode(enc_2_A, INPUT_PULLUP); // sets the Encoder_output_A pin as the input
  pinMode(enc_2_B, INPUT); // sets the Encoder_output_B pin as the input
  attachInterrupt(digitalPinToInterrupt(enc_2_A), DC_Motor_Encoder_2, RISING);
  
  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(leftPub);
  nh.advertise(rightPub);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Read Time
  currentMillis = millis(); 
  // If 100ms have passed, print the number of ticks
  if (currentMillis - previousMillis > interval) {
    leftPub.publish(&count_pulses_1);
    rightPub.publish(&count_pulses_2);
    previousMillis = currentMillis;
    nh.spinOnce();
  }

  // forward(rpm);
  // reverse(rpm); 
  //  Serial.print("PWM L: "); 
  //  Serial.print(rpm_1);
  //  Serial.print(" : Count L: ");  
  //  Serial.print(count_pulses_1);
  //  Serial.print("PWM R: "); 
  //  Serial.print(rpm_2);
  //  Serial.print(" : Count R: ");  
  //  Serial.println(count_pulses_2);
}
void forward(int rpm_1, int rpm_2){
      analogWrite(pwm_1, rpm_1);
      analogWrite(pwm_2, rpm_2);
      digitalWrite(dir_1, HIGH);
      digitalWrite(dir_2, HIGH);
}

void reverse(int rpm_1, int rpm_2){
      analogWrite(pwm_1, rpm_1);
      analogWrite(pwm_2, rpm_2);
      digitalWrite(dir_1, LOW);
      digitalWrite(dir_2, LOW);
}

void left(int rpm_1, int rpm_2){
      analogWrite(pwm_1, rpm_1);
      analogWrite(pwm_2, rpm_2);
      digitalWrite(dir_1, HIGH);
      digitalWrite(dir_2, LOW);
}

void right(int rpm_1, int rpm_2){
      analogWrite(pwm_1, rpm_1);
      analogWrite(pwm_2, rpm_2);
      digitalWrite(dir_1, LOW);
      digitalWrite(dir_2, HIGH);
}

void brake(){
      analogWrite(pwm_1, 0);
      analogWrite(pwm_2, 0);
}

void DC_Motor_Encoder_1(){
  int b = digitalRead(enc_1_B);
  if(b > 0){
    count_pulses_1.data++;
  }
  else{
    count_pulses_1.data--;
  }
}

void DC_Motor_Encoder_2(){
  int q = digitalRead(enc_2_B);
  if(q > 0){
    count_pulses_2.data++;
  }
  else{
    count_pulses_2.data--;
  }
}

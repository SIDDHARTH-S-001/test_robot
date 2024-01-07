// GPIOs for L298N Motor driver
#define en_1 4
#define in_1 5
#define in_2 6
int rpm = 0;    // Motor PWM

// Encoder pins 
#define enc_1_A 2
#define enc_1_B 3


// GPIO for Potentiometer
# define pot_pin A0
int pot_value = 0;

// Encoder Parameters
int count_pulses_1 = 0;
int gear_ratio = 90;
int cpr = 3960; 

void setup() {
  // put your setup code here, to run once:
  // Drive motor and their pins en -> enable/pwm , in -> direction
  pinMode(en_1, OUTPUT);
  pinMode(in_1, OUTPUT);
  pinMode(in_2, OUTPUT);

  pinMode(enc_1_A, INPUT_PULLUP); // sets the Encoder_output_A pin as the input
  pinMode(enc_1_B, INPUT); // sets the Encoder_output_B pin as the input
  attachInterrupt(digitalPinToInterrupt(enc_1_A), DC_Motor_Encoder_1, RISING);

  pinMode(pot_pin, INPUT);
  
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  pot_value = analogRead(pot_pin);
  rpm = map(pot_value, 0, 1023, 0, 255);
  

  forward(rpm);
  // reverse (rpm); 
  Serial.print("PWM: "); 
  Serial.print(rpm);
  Serial.print(" : Count: ");  
  Serial.println(count_pulses_1);
}

void forward(int rpm){
      analogWrite(en_1, rpm);
      digitalWrite(in_1, HIGH);
      digitalWrite(in_2, LOW);
}

void reverse(int rpm){
      analogWrite(en_1, rpm);
      digitalWrite(in_1, LOW);
      digitalWrite(in_2, HIGH);
}

void brake(){
      analogWrite(en_1, 0);
}

void DC_Motor_Encoder_1(){
  int b = digitalRead(enc_1_B);
  if(b > 0){
    count_pulses_1 = count_pulses_1 + 1;
  }
  else{
    count_pulses_1 = count_pulses_1 - 1;
  }
}

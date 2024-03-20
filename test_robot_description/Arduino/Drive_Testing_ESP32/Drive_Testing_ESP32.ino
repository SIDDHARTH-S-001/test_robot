// Libraries
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
// #include <Servo.h>

// Network Credentials -> ESP32 in Station Mode
const char* ssid = "Micromax-HS2";
const char* password = "Micromax";

WiFiClient client;
WiFiServer server(80); // Port 80 -> HTTP 
String data = "";

// Object for MG996R servo that controls camera orientation
// Servo cam;
// #define cam 21;
// int cam_pos = 0;  // Current position
// int cam_ppos = 0; // Previous position

// GPIOs for L298N Motor driver
// Left Motor
#define en_1 4
#define in_1 2
#define in_2 16
// Right Motor
#define en_2 5
#define in_3 18
#define in_4 19
int rpm = 0;    // Motor PWM

// Encoder pins - Left Motor
#define enc_1_A 32
#define enc_1_B 33
// Encoder pins - Left Motor
#define enc_2_A 25
#define enc_2_B 26

// Parameters
unsigned int count_pulses_1 = 0;
unsigned int count_pulses_2 = 0;
int gear_ratio = 90;
int cpr = 3960; 

void setup() {
  // put your setup code here, to run once:
  // Drive motors and their pins en -> enable/pwm , in -> direction
  pinMode(en_1, OUTPUT);
  pinMode(in_1, OUTPUT);
  pinMode(in_2, OUTPUT);
  pinMode(en_2, OUTPUT);
  pinMode(in_3, OUTPUT);
  pinMode(in_4, OUTPUT);

  pinMode(enc_1_A, INPUT_PULLUP); // sets the Encoder_output_A pin as the input
  pinMode(enc_1_B, INPUT); // sets the Encoder_output_B pin as the input
  attachInterrupt(digitalPinToInterrupt(enc_1_A), DC_Motor_Encoder_1, RISING);

  pinMode(enc_2_A, INPUT_PULLUP); // sets the Encoder_output_A pin as the input
  pinMode(enc_2_B, INPUT); // sets the Encoder_output_B pin as the input
  attachInterrupt(digitalPinToInterrupt(enc_2_A), DC_Motor_Encoder_2, RISING);

  // cam.attach(cam_pin);
  // cam.write(cam_ppos);

  Serial.begin(115200); // Baud Rate
  Serial.println("Connecting to WIFI");
  WiFi.begin(ssid, password);
  while ((!(WiFi.status() == WL_CONNECTED))){
    delay(500);
    Serial.print("...");
  }
  Serial.println("WiFi connected");
  Serial.println("ESP32 Local IP is : ");
  Serial.print((WiFi.localIP()));
  server.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  client = server.available(); // Checking for availability of client (mobile app) before processing request
    if (!client){
      return; 
      }
    data = checkClient();
    String dataVal = data.substring(1, data.length());
    String rt = data.substring(0, 1); // Route
    Serial.println(data); 

    if (rt == "f"){
      Serial.println("Forward");
      rpm = dataVal.toInt();
      forward(rpm);
    }
    else if (rt == "b"){
      Serial.println("Reverse");
      rpm = dataVal.toInt();
      reverse(rpm);
    }
    else if (rt == "l"){
      Serial.println("Left");
      rpm = dataVal.toInt();
      left(rpm);
    }    
    else if (rt == "r"){
      Serial.println("Right");
      rpm = dataVal.toInt();
      right(rpm);
    }
    else if (rt == "s"){
      Serial.println("Stop");
      brake();
    }

    // else if(rt == "c"){
    //   Serial.println("Camera Servo");
    //   cam_pos = dataVal.toInt();

    //   if (cam_pos > cam_ppos){
    //     for (int i = cam_ppos; i <= cam_pos; i++){
    //       cam.write(i);
    //       delay(20);
    //     }
    //   }
    //   else if(cam_pos < cam_ppos){
    //     for (int j = cam_ppos; j >= cam_pos; j--){
    //       cam.write(j);
    //       delay(20);
    //     }
    //   }
    //   cam_ppos = cam_pos;
    // }

    String response = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n";
    response += String(" ");
    client.print(response);

}

String checkClient (void){
  while(!client.available()){
  delay(1); 
  Serial.println(".");}
  Serial.println("Client Found");
  String request = client.readStringUntil('\r');
  request.remove(0, 5);
  request.remove(request.length()-9,9);
  return request;
}

void forward(int rpm){
      analogWrite(en_1, rpm);
      analogWrite(en_2, rpm);
      digitalWrite(in_1, HIGH);
      digitalWrite(in_2, LOW);
      digitalWrite(in_3, HIGH);
      digitalWrite(in_4, LOW);
}

void reverse(int rpm){
      analogWrite(en_1, rpm);
      analogWrite(en_2, rpm);
      digitalWrite(in_1, LOW);
      digitalWrite(in_2, HIGH);
      digitalWrite(in_3, LOW);
      digitalWrite(in_4, HIGH);
}

void left(int rpm){
      analogWrite(en_1, rpm);
      analogWrite(en_2, rpm);
      digitalWrite(in_1, LOW);
      digitalWrite(in_2, HIGH);
      digitalWrite(in_3, HIGH);
      digitalWrite(in_4, LOW);
}

void right(int rpm){
      analogWrite(en_1, rpm);
      analogWrite(en_2, rpm);
      digitalWrite(in_1, HIGH);
      digitalWrite(in_2, LOW);
      digitalWrite(in_3, LOW);
      digitalWrite(in_4, HIGH);
}

void brake(){
      analogWrite(en_1, 0);
      analogWrite(en_2, 0);
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

void DC_Motor_Encoder_2(){
  int q = digitalRead(enc_2_B);
  if(q > 0){
    count_pulses_2 = count_pulses_2 + 1;
  }
  else{
    count_pulses_2 = count_pulses_2 - 1;
  }
}
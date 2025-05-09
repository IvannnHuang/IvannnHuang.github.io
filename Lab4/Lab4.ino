#define AB1IN_LEFT 3
#define AB2IN_LEFT 14
#define AB1IN_RIGHT 16
#define AB2IN_RIGHT 15
#define dc 60
#define dc_dif 1.5

void setup() {
  pinMode(AB1IN_LEFT,OUTPUT);
  pinMode(AB2IN_LEFT,OUTPUT);
  pinMode(AB1IN_RIGHT,OUTPUT);
  pinMode(AB2IN_RIGHT,OUTPUT);
}
void backward() {
  analogWrite(AB1IN_LEFT,dc); 
  analogWrite(AB2IN_LEFT,0);
  analogWrite(AB1IN_RIGHT,dc*dc_dif); 
  analogWrite(AB2IN_RIGHT,0);
}
void forward() {
  analogWrite(AB1IN_LEFT,0); 
  analogWrite(AB2IN_LEFT,dc);
  analogWrite(AB1IN_RIGHT,0); 
  analogWrite(AB2IN_RIGHT,dc*dc_dif);
}
void turn_right() {
  analogWrite(AB1IN_LEFT,0); 
  analogWrite(AB2IN_LEFT,dc);
  analogWrite(AB1IN_RIGHT,0); 
  analogWrite(AB2IN_RIGHT,dc*dc_dif*0.25);
}
void turn_left() {
  analogWrite(AB1IN_LEFT,0); 
  analogWrite(AB2IN_LEFT,dc*0.25);
  analogWrite(AB1IN_RIGHT,0); 
  analogWrite(AB2IN_RIGHT,dc*dc_dif);
}
void pause() {
  analogWrite(AB1IN_LEFT,0); 
  analogWrite(AB2IN_LEFT,0);
  analogWrite(AB1IN_RIGHT,0); 
  analogWrite(AB2IN_RIGHT,0);
}

void loop() {
  pause();
  delay(2000);
  forward();
  delay(2000);
  pause();
  delay(2000);
  turn_left();
  delay(1000);
  pause();
  delay(2000);
  backward();
  delay(2000);
  pause();
  delay(2000);
  turn_right();
  delay(2000);
  pause();
  delay(2000);
}
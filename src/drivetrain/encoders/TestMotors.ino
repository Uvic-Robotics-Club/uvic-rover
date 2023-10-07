#define FORWARD LOW
#define REVERSE HIGH


#define DIR_MOTOR 10
#define PWM_MOTOR 11

int valx;
int valy;
int valMap;
void setup() {
  // put your setup code here, to run once:
  pinMode(DIR_MOTOR, OUTPUT);
  pinMode(PWM_MOTOR, OUTPUT);
  Serial.begin(9600) ;
}

void loop() {
  // put your main code here, to run repeatedly:
//  valx = digitalRead(JOY_X);
//  valy = digitalRead(JOY_Y);
  valx = analogRead(A1);
  valMap = map(valx, 0, 1023, -255, 255);


  if(valMap < 0){
    digitalWrite(DIR_MOTOR, 0);
    analogWrite(PWM_MOTOR, abs(valMap));

  }else{
    digitalWrite(DIR_MOTOR, 1);
    analogWrite(PWM_MOTOR, abs(valMap));
  }
  
 delay(200);

}
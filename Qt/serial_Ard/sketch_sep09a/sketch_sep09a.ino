int led_pin = 13;
int val = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(led_pin,OUTPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("100");
  delay(1000);
  Serial.println("200");
  delay(1000);
  Serial.println("300");
  delay(1000);
}

void serialEvent(){
  if(Serial.available()){
    val = Serial.parseInt();
    if (val == 1){
      digitalWrite(led_pin,HIGH);
    }
    if (val == 2){
      digitalWrite(led_pin,LOW);
    }
  }
}

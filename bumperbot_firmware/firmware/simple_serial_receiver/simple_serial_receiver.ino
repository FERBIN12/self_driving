#define LED_PIN 13

void setup() {
  pinMode(LED_PIN, OUTPUT);  // Set pin 13 as an output
  digitalWrite(LED_PIN,LOW);
  Serial.begin(115200);
  Serial.print("Serial_begin");

}

void loop() {

  if (Serial.available()){
    int x = Serial.readString().toInt();
    if (x ==0){
      Serial.print("OFF");
      digitalWrite(LED_PIN,LOW);
    }else{
      Serial.print("ON");
      digitalWrite(LED_PIN,HIGH);}
  }
}

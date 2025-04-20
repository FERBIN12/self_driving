// L298N H-Bridge Connection PINs
#define L298N_enA 9    // PWM for Motor A
#define L298N_enB 10   // PWM for Motor B

#define L298N_in1 12   // Direction Motor A
#define L298N_in2 8   // Direction Motor A

#define L298N_in3 4    // Direction Motor B
#define L298N_in4 7    // Direction Motor B

float cmd = 0;

void setup() {
  // Set pin modes
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);

  pinMode(L298N_enB, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);
  
  // Set Motor Rotation Direction (both forward)
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);

  digitalWrite(L298N_in4, HIGH);
  digitalWrite(L298N_in3, LOW);

  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    cmd = Serial.readString().toFloat();

    // Safety: constrain value from 0.0 to 1.0
//    cmd = constrain(cmd, 0.0, 1.0);

    int pwmVal = (int)(cmd * 100);

    analogWrite(L298N_enA, pwmVal);
    analogWrite(L298N_enB, pwmVal);
  }
}

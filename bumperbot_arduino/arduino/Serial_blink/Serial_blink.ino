

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
}

void loop() {
  
  if(Serial.available())
  {
    int x = Serial.readString().toInt();
    Serial.print("Received: ");
    Serial.print(x);
    Serial.print("\n");
    if (x == 0)
    {
      digitalWrite(LED_BUILTIN, LOW);
    }
    else{
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }

}

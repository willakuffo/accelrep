
void setup() {
  Serial.begin(2000000);
  pinMode(1, INPUT);
  pinMode(0, OUTPUT);
}

void loop() {
  if(Serial.available()){
int data = Serial.read();
if(data ==49){
  Serial.println(true);
  }
  if(data ==48){
  Serial.println(false);
  }
Serial.println(data);  
}
}

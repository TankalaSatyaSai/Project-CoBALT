#define enA 9
#define enB 8
#define in1 6
#define in2 7
#define in3 5
#define in4 4 
int Direction_1, Direction_2 = 0;
//data reading
bool readingEnabled = false;
String inputString = ""; 
int incomingByte = 0; 

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);
  Serial.begin(9600);
  Serial.println("<Arduino is ready>");

  digitalWrite(in3, 0);
  digitalWrite(in4, 0);
  digitalWrite(in1, 0);
  digitalWrite(in2, 0);
}

void loop() {
  ReadData();
  Serial.print("Direction 1 : ");
  Serial.print(Direction_1);
  Serial.print("Direction 2 : ");
  Serial.println(Direction_2);

  if (Direction_2 == 0){
    digitalWrite(in3, 0);
    digitalWrite(in4, 0);
  }
  else if (Direction_2 == 1){
    digitalWrite(in3, 1);
    digitalWrite(in4, 0);
  }
  else if (Direction_2 == -1){
    digitalWrite(in3, 0);
    digitalWrite(in4, 1);
  }

  if (Direction_1 == 0){
    digitalWrite(in1, 0);
    digitalWrite(in2, 0);
  }
  else if (Direction_1 == 1){
    digitalWrite(in1, 1);
    digitalWrite(in2, 0);
  }
  else if (Direction_1 == -1){
    digitalWrite(in1, 0);
    digitalWrite(in2, 1);
  }

  analogWrite(enA, 255); // Send PWM signal to L298N Enable pin
  analogWrite(enB, 255); // Send PWM signal to L298N Enable pin
  delay(1000);
}

void ReadData(){
  while (Serial.available()) {
    char inputChar = Serial.read();

    if (inputChar == '<') {
      // Start reading
      inputString = ""; // Clear the string buffer
      readingEnabled = true;
    } else if (inputChar == '>') {
      // Stop reading and process data
      readingEnabled = false;
      processInput();
    } else if (readingEnabled) {
      // Append character to inputString if reading is enabled
      inputString += inputChar;
    }
  }
}

void processInput() {
  // Split inputString into two substrings based on ','
  int commaIndex = inputString.indexOf(',');
  
  if (commaIndex != -1) {
    // Extract substrings for a and b
    String aString = inputString.substring(0, commaIndex);
    String bString = inputString.substring(commaIndex + 1);
    
    // Convert strings to integers
    Direction_1 = aString.toInt();
    Direction_2 = bString.toInt();
  }
}
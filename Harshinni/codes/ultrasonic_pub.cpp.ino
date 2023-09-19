const int trigPin1 = 5;
const int echoPin1 = 4;
const int trigPin2 = 2;
const int echoPin2 = 3;
const int trigPin3 = 12;
const int echoPin3 = 13;
const int trigPin4 = 9;
const int echoPin4 = 10;

long duration1, duration2, duration3, duration4;
int distance1, distance2, distance3, distance4;

void setup() {
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(trigPin4, OUTPUT);
  pinMode(echoPin4, INPUT);
  
  Serial.begin(9600);
}

void loop() {
  // Clear the input and output buffers
  Serial.flush();

  // Sensor 1
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  distance1 = duration1 * 0.034 / 2;

  // Sensor 2
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  distance2 = duration2 * 0.034 / 2;

  // Sensor 3
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  duration3 = pulseIn(echoPin3, HIGH);
  distance3 = duration3 * 0.034 / 2;

  // Sensor 4
  digitalWrite(trigPin4, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin4, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin4, LOW);
  duration4 = pulseIn(echoPin4, HIGH);
  distance4 = duration4 * 0.034 / 2;

  // Print the distances as a tuple
  Serial.print("Distances: (");
  Serial.print(distance1);
  Serial.print(", ");
  Serial.print(distance2);
  Serial.print(", ");
  Serial.print(distance3);
  Serial.print(", ");
  Serial.print(distance4);
  Serial.println(")");

  delay(1000); // Adjust the delay as needed for your application
}

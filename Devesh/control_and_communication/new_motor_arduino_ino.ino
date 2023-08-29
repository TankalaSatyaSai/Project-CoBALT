// Pins for Motor 1
#define F_RH_PWM 6
#define F_RH_DIR 7

// Pins for Motor 2
#define F_LH_PWM 5
#define F_LH_DIR 4

// Pins for motor 3
#define R_LH_PWM 5
#define R_LH_DIR 4

// Pins for motor 4
#define R_RH_PWM 5
#define R_RH_DIR 4

#define GR 50.9 // Gear Ratio
#define PPR 7.0 // Pulse Per Revolution

#define MIN_PWM 25
#define MAX_PWM 255
#define MIN_SPD 0.4 // in revolutions per second
#define MAX_SPD 4 // in revolutions per second

//for communication
char receivedChars[12];
boolean newData = false;

class Motor {
public:
  float desiredSpeed = 0;
  float currentSpeed = 0;
  float Kp = 0.5;
  int output;
  
  void setDesiredSpeed(float s)
  // s is desired speed in revolutions per second
  {
    if (abs(s) > MAX_SPD)
    {
      int signofspeed = (s > 0) - (s < 0); //This expression evaluates to -1 for negative speeds, 0 for zero speed, and 1 for positive speeds.
      desiredSpeed = signofspeed*MAX_SPD;
      return;
    }

    if (abs(s) < MIN_SPD)
    {
      desiredSpeed = 0;
      return;
    }
    desiredSpeed = s; //if neither condition is satisfied then 
   
  }

 

//  void computeOutput() {
//    //PID code    
//    float error = desiredSpeed - currentSpeed;
//    float controlValue = Kp * error; // Simple proportional control
//    output = int(controlValue);
//  }
};

Motor front_leftMotor;
Motor front_rightMotor;
Motor rear_leftMotor;
Motor rear_rightMotor;

void setup() {
  pinMode(F_RH_DIR, OUTPUT);
  pinMode(F_LH_DIR, OUTPUT);
  pinMode(R_LH_DIR, OUTPUT);
  pinMode(R_RH_DIR, OUTPUT);
  pinMode(F_RH_PWM, OUTPUT);
  pinMode(F_LH_PWM, OUTPUT);
  pinMode(R_LH_PWM, OUTPUT);
  pinMode(R_RH_PWM, OUTPUT);

  Serial.begin(9600);
  

  Serial.println("<Arduino is ready>");
}

void loop() {
  // Get commanded speed
  recvWithStartEndMarkers();
  if (newData == true) {
    parseData();
    newData = false;
  }

//  front_rightMotor.computeOutput();
//  front_leftMotor.computeOutput();
//  rear_leftMotor.computeOutput();
//  rear_rightMotor.computeOutput();

  // Apply the motor outputs here
  analogWrite(F_LH_PWM, front_leftMotor.output);
  analogWrite(F_RH_PWM, front_rightMotor.output);
  analogWrite(R_LH_PWM, rear_leftMotor.output);
  analogWrite(R_RH_PWM, rear_rightMotor.output);
}

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false)
  {
    rc = Serial.read();

    if (recvInProgress == true)
    {
      if (rc != endMarker)
      {
        receivedChars[ndx] = rc;
        ndx++;
      }
      else
      {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker)
    {
      recvInProgress = true;
    }
  }

}

void parseData() {
  // split the data into its parts
{
  char *strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(receivedChars, ","); // get the first part - front left speed
  front_leftMotor.setDesiredSpeed(atof(strtokIndx)/100);

  strtokIndx = strtok(NULL, ","); // get the second part - front right speed
  front_rightMotor.setDesiredSpeed(atof(strtokIndx)/100);
  
  strtokIndx = strtok(NULL, ","); // get the third part - rear left speed
  rear_leftMotor.setDesiredSpeed(atof(strtokIndx)/100);
  
  strtokIndx = strtok(NULL, ","); // get the fourth part - rear right speed
  rear_rightMotor.setDesiredSpeed(atof(strtokIndx)/100);
}
}


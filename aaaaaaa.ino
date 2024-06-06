double count = 0; //set the counts of the encoder
double rot = 0;//set the angles
bool direction = true; //true = forward, false = reverse

double pwm = 9; // this is the PWM pin for the motor for how much we move it to correct for its error
const int dir = 4; // DIR pin to control the direction of the motor (clockwise/counter-clockwise)
const int idir = 5; // Reverse direction, digital
const int enc = 3; // digital

double setpoint = 20;//I am setting it to move through 360 degrees
double Kp = 5.00;
double Ki = 0.05;
double Kd = 0.6;

float last_error = 0;
float error = 0;
float changeError = 0;
float totalError = 0;
float pidTerm = 0;
float pidTerm_scaled = 0;// if the total gain we get is not in the PWM range we scale it down so that it's not bigger than |255|


void setup() {
  Serial.begin(9600);
  pinMode(3, INPUT);
  attachInterrupt(digitalPinToInterrupt(3), counting, CHANGE); //interrupt pins for encoder
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(idir, OUTPUT);
}

void loop(){
  
  PIDcalculation();// find PID value
  
  if (rot < setpoint) {
    digitalWrite(dir, HIGH);// Forward motion
    digitalWrite(idir, LOW);
    direction = true;
  } else {
    digitalWrite(dir, LOW);//Reverse motion
    digitalWrite(idir, HIGH);
    direction = false;
  }

  analogWrite(pwm, pidTerm_scaled);

  Serial.println(" WHEEL ROTATIONS: ");
  Serial.print(pidTerm_scaled);
  Serial.print("Volts: ");
  Serial.print(rot);

  delay(100);
}

void counting(){
  if(direction){
    count++;
  } else {
    count--;
  }
}

void PIDcalculation(){
  rot = (count / 40); //count to angle conversion
  error = setpoint - rot;
  
  changeError = error - last_error; // derivative term
  totalError += error; //accumalate errors to find integral term
  pidTerm = (Kp * error) + (Ki * totalError) + (Kd * changeError);//total gain
  pidTerm = constrain(pidTerm, -230, 230);//constraining to appropriate value
  pidTerm_scaled = abs(pidTerm);//make sure it's a positive value

  last_error = error;
}
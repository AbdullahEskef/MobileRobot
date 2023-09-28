#define PI 3.14
#define LEFTDIR1 6 
#define LEFTDIR2 5
#define LEFTPWM 4
#define RIGHTDIR1 9
#define RIGHTDIR2 8
#define RIGHTPWM 7
#define CHANNEL_A_LEFT 3
#define CHANNEL_B_RIGHT 11
#define CHANNEL_A_LEFT 2
#define CHANNEL_B_LEFT 10
#define ENC_COUNTREV 700
#define WMAX 95 
#define VMAX .34819
#define DRIVE_RADIUS .5

double totalVelocity = .25;
double distanceBetweenWheels = .195;
double radiusOfWheel = .035;

double RWVelocity = totalVelocity + (totalVelocity/DRIVE_RADIUS)*distanceBetweenWheels/2;
int rightPosition = 0;
long rightPrevT = 0;
double rightEprev = 0;
double rightEintergal = 0;
double rpmRight = 0;
double angularVelocityRight = 0;
double rightMeasuredVelocity = 0;
double kpR = 1;
double kdR = 0;
double kiR = 0;

double LWVelocity = totalVelocity - (totalVelocity/DRIVE_RADIUS)*distanceBetweenWheels/2;
int leftPosotion = 0;
long leftPrevT = 0;
double leftEprev = 0;
double leftEintergal = 0;
double rpmLeft = 0;
double angularVelocityLeft = 0;
double leftMeasuredVelocity = 0;
double kpL = 1;
double kdL = 0;
double kiL = 0;

void readRightEncoder();
void readLeftEncoder();
void TurnON(int pwmVal, int pwm, int in1,int in2);
void leftPID();
void rightPID();


void
setup()
{
    pinMode(CHANNEL_A_LEFT,INPUT_PULLUP);
    pinMode(CHANNEL_A_LEFT,INPUT_PULLUP);
    pinMode(CHANNEL_B_LEFT,INPUT_PULLUP);
    pinMode(CHANNEL_B_RIGHT,INPUT_PULLUP);
    pinMode(LEFTPWM,OUTPUT);
    pinMode(RIGHTPWM,OUTPUT);
    pinMode(LEFTDIR1,OUTPUT);
    pinMode(LEFTDIR2,OUTPUT);
    pinMode(RIGHTDIR1,OUTPUT);
    pinMode(RIGHTDIR2,OUTPUT);
    attachInterrupt(digitalPinToINterrupt(CHANNEL_A_LEFT),readRightEncoder,RISING);
    attachInterrupt(digitalPinToINterrupt(CHANNEL_A_LEFT),readLeftEncoder,RISING);
}

void
loop()
{
    rightPID();
    leftPID();
}


void
readRightEncoder()
{
    int pulse = digitalRead(CHANNEL_B_RIGHT);
    if(pulse > 0) rightPosition++;
    long currentTime = micros();
    double rightDeltaT = ((double)(currentTime - rightPrevT))/1.0e6;
    rightPrevT = currentTime;
    rpmRight = (double)60.0/(ENC_COUNTREV*leftDeltaT);
    angularVelocityRight = rpmRight*2*PI/60;
    rightMeasuredVelocity = angularVelocityRight * DRIVE_RADIUS;
}

void
readLeftEncoder()
{
int pulse = digitalRead(CHANNEL_B_LEFT);
    if(pulse > 0) leftPosotion++;
    long currentTime = micros();
    double leftDeltaT = ((double)(currentTime - leftPrevT))/1.0e6;
    leftPrevT = currentTime;
    rpmLeft = (double)60.0/(ENC_COUNTREV*leftDeltaT);
    angularVelocityLeft = rpmLeft*2*PI/60;
    leftMeasuredVelocity = angularVelocityLeft * DRIVE_RADIUS;
}

void
TurnON(int pwmVal, int pwm, int in1,int in2)
{
    analogWrite(pwm,pwmVal);
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
}

void
rightPID()
{
    int rightError = RWVelocity - rightMeasuredVelocity;
    rightEintergal = rightEintergal + rightError*kiR;
    double rightDeriv = (rightError - rightEprev)/(0.001);
    double rightVelocity = kpR*rightError + rightEintergal + kdR*rightDeDt;
    rightEprev = rightError;
    RWVelocity = fabs(rightVelocity);
    RWVelocity = 255 - (255*((VMAX-RWVelocity)/VMAX));
    RWVelocity = constrain(RWVelocity,0,255);
    TurnON(RWVelocity,RIGHTPWM,RIGHTDIR1,RIGHTDIR2);
}

void
leftPID()
{
    int leftError = LWVelocity - leftMeasuredVelocity;
    leftEintergal = leftEintergal +  leftError*kiL;
    double leftDeriv = (leftError - leftEprev)/(0.001);
    double leftVelocity = kpL*leftError + leftEintergal + kdL*leftDeriv;
    leftEprev = leftError;
    LWVelocity = fabs(leftVelocity);
    LWVelocity = 255 - (255*((VMAX-LWVelocity)/VMAX));
    LWVelocity = constrain(LWVelocity,0,255);
    TurnON(LWVelocity,LEFTPWM,LEFTDIR1,LEFTDIR2);
}

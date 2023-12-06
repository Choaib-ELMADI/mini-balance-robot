#include "PID_v1.h"


//****************************
//       Set Up MPU6050      *
//****************************

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
 
#define INTERRUPT_PIN 2

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];
VectorInt16 gy;

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}


//***************************
//         PID Control      *
//***************************

#define PID_MIN_LIMIT -255
#define PID_MAX_LIMIT 255
#define PID_SAMPLE_TIME_IN_MILLI 10

#define SETPOINT_PITCH_ANGLE_OFFSET .9 // -2.2   

#define MIN_ABSOLUTE_SPEED 0

double setpointPitchAngle = SETPOINT_PITCH_ANGLE_OFFSET;
double pitchGyroAngle = 0;
double pitchPIDOutput = 0;

double setpointYawRate = 0;
double yawGyroRate = 0;
double yawPIDOutput = 0;

#define PID_PITCH_KP 10 // 10
#define PID_PITCH_KI 50 // 80 
#define PID_PITCH_KD .6 // .8

#define PID_YAW_KP 0
#define PID_YAW_KI 0
#define PID_YAW_KD 0

PID pitchPID(&pitchGyroAngle, &pitchPIDOutput, &setpointPitchAngle, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, DIRECT);
PID yawPID(&yawGyroRate, &yawPIDOutput, &setpointYawRate, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, DIRECT);


//***************************
//       Set Up DC Motors   *
//***************************

// Motor A: ==> RIGHT
const int IN1 = 7, IN2 = 8;
const int ENA = 9;

// Motor B: ==> LEFT
const int IN3 = 12, IN4 = 11;
const int ENB = 10;

void setupPID() {
    pitchPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
    pitchPID.SetMode(AUTOMATIC);
    pitchPID.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI);

    yawPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
    yawPID.SetMode(AUTOMATIC);
    yawPID.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI);
}

void setupMotors() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    rotateMotor(0, 0);
}

void setupMPU() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
  
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();

    mpu.setXAccelOffset(-234); 
    mpu.setYAccelOffset(1763); 
    mpu.setZAccelOffset(536);   
    mpu.setXGyroOffset(154);
    mpu.setYGyroOffset(67);
    mpu.setZGyroOffset(43);
    
    if (devStatus == 0) {
        // mpu.CalibrateAccel(6);
        // mpu.CalibrateGyro(6);
        
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
}

void setup() {
    Serial.begin(115200);
    
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    
    setupMotors();
  
    setupMPU();
  
    setupPID();

    digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
    if (!dmpReady) return;

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetGyro(&gy, fifoBuffer);

        yawGyroRate = gy.z;
        pitchGyroAngle = ypr[1] * 180/M_PI;
        Serial.println(pitchGyroAngle);

        if (pitchGyroAngle >= 40 || pitchGyroAngle <= -40) {
            digitalWrite(ENA, LOW);
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, LOW);
            
            digitalWrite(ENB, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, LOW);

            return;
        }

        pitchPID.Compute();
        yawPID.Compute();

        rotateMotor(pitchPIDOutput+yawPIDOutput, pitchPIDOutput-yawPIDOutput);
    }
}

void rotateMotor(int speed1, int speed2) {
    if (speed1 <= 0) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);    
    }
    else if (speed1 > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);      
    }

    if (speed2 <= 0) {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);    
    }
    else if (speed2 > 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);      
    }
 
    speed1 = abs(speed1) + MIN_ABSOLUTE_SPEED;
    speed2 = abs(speed2) + MIN_ABSOLUTE_SPEED;

    speed1 = constrain(speed1, MIN_ABSOLUTE_SPEED, 255);
    speed2 = constrain(speed2, MIN_ABSOLUTE_SPEED, 255);
    
    analogWrite(ENA, speed1);
    analogWrite(ENB, speed2);    
}

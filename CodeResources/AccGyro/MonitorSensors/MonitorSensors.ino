//********************************************************************
//********************************************************************
//***********************Setting parameters***************************
//Setting parameters in ADXL372 part
//********************************************************************
//********************************************************************

#include "Wire.h"
#include "adxl372.h"

#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
    #define SERIAL Serial
    #define SYS_VOL   3.3
#else
    #define SERIAL Serial
    #define SYS_VOL   5
#endif


float cali_data[3];

#define CALI_BUF_LEN           15
#define CALI_INTERVAL_TIME     250

float cali_buf[3][CALI_BUF_LEN];


ADXL372 acc;
xyz_t xyz;

int i,j,currentTime,previousTime;
float accX[512];
float accY[512];
float accZ[512];


//Setting parameters in MPU6050 part
//********************************************************************
//********************************************************************
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
MPU6050 mpu;


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


//********************************************************************
//********************************************************************

//******************'SETUP' part of the code**************************
void setup() {
  //Setting up in ADXL372
    
    SERIAL.begin(115200); // Setting baud rate at 115200 
    acc.begin(); //Start transmitting of ADXL372
    acc.timing_ctrl(RATE_3200); //Setting the Sample time (400/800/1600/3200/6400 Hz)
    acc.power_ctrl(MEASUREMENT_MODE); //Setting the ADXL372 to Measurement Mode
    acc.measurement_ctrl(BW_1600, true); //Setting the bandwidth of the LPF(Need bo be no more than half of the sampling rate)
    //***Hint: All these pre-set functions can be found in the "adxl372.h" library which is inside the .zip file "Accelerometer_ADXL372-master"
    acc.setActiveTime(0);  //***      And specfic information can be linked to the information from ADXL372 Datasheet
                            //Setting the value of Activity Timer Register. The timer period depends on the ODR selected. At 3200 Hz and below, it is ~6.6MS
                            //For activity detection to trigger, above threshold activity must be sustained for a time equal to the number of activity timer periods specified in the activity timer register
                            //Example: A register value of 0 results in single sample activity detection.
    calibration(); //Start Calibration function at the beginning
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    mpu.setDLPFMode(MPU6050_DLPF_BW_256); //Setting the Output rate at 8K Hz mode
    mpu.setRate(1.5); // Setting the sampling rate at 3200Hz(8000/(1+1.5))
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000); // Setting the Gyro scale range to +/- 2000 degrees
    

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;
        

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    previousTime = millis();
}
//********************************************************************


//******************'LOOP' part of the code**************************
void loop() {
  currentTime = millis();
  //********************************************************************
  //********************Main Lopp for ADXL372***************************
  //******************************************************************** 
    if (acc.status() & DATA_READY) {
        acc.read(&xyz);
        SERIAL.print("ACC\t");
        SERIAL.print((xyz.x - cali_data[0]) / 10.0);
        SERIAL.print("g\t");
        SERIAL.print((xyz.y - cali_data[1]) / 10.0);
        SERIAL.print("g\t");
        SERIAL.print((xyz.z - cali_data[2]) / 100.0);
        SERIAL.print("g\t");
        
  }
  //********************************************************************
  //********************Main Lopp for MPU6050***************************
  //********************************************************************
  // if programming failed, don't try to do anything
    if (!dmpReady) return;
  // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
  // get current FIFO count
    fifoCount = mpu.getFIFOCount();
  // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
             
        
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("Gyro\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[2] * 180/M_PI);
            Serial.print("\n");
        
    }
}






//*******************FUNCTIONS****************************************
void calibration(void) {
    SERIAL.println("Please Place the module horizontally!");
    delay(1000);
    SERIAL.println("Start calibration........");


    for (int i = 0; i < CALI_BUF_LEN; i++) {
        while (!(acc.status() & DATA_READY));
        acc.read(&xyz);
        cali_buf[0][i] = xyz.x;
        cali_buf[1][i] = xyz.y;
        cali_buf[2][i] = xyz.z;
        delay(CALI_INTERVAL_TIME);
        SERIAL.print('.');
    }
    SERIAL.println('.');
    for (int i = 0; i < 3; i++) {
        cali_data[i] =  deal_cali_buf(cali_buf[i]);
        if (2 == i) {

            cali_data[i] -= 10;
        }
        SERIAL.println(cali_data[i]);
    }
    SERIAL.println("Calibration OK!!");
}

float deal_cali_buf(float* buf) {
    float cali_val = 0;

    for (int i = 0; i < CALI_BUF_LEN; i++) {
        cali_val += buf[i];
    }
    cali_val = cali_val / CALI_BUF_LEN;
    return (float)cali_val;
}


#include "Wire.h" // Wire library for I2C communication
#include "adxl372.h" // ADXL372 library for setting configures

#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
    #define SERIAL Serial
    #define SYS_VOL   3.3
#else
    #define SERIAL Serial
    #define SYS_VOL   5
#endif

/// Calibration Setting
float cali_data[3]; 
#define CALI_BUF_LEN           15
#define CALI_INTERVAL_TIME     250
float cali_buf[3][CALI_BUF_LEN];//Buffer that for containing calibration data
float x_out,y_out,z_out;

ADXL372 acc;
xyz_t xyz;

float deal_cali_buf(float* buf) {
    float cali_val = 0;
    for (int i = 0; i < CALI_BUF_LEN; i++) {
        cali_val += buf[i];
    }
    cali_val = cali_val / CALI_BUF_LEN;
    return (float)cali_val;
}


void calibration(void) {
    SERIAL.println("Please Place the module horizontally!");
    delay(1000);
    SERIAL.println("Start calibration........");
    for (int i = 0; i < CALI_BUF_LEN; i++) {
        // Checking the status and while the data available in the I2C bus, 
        // the data on the bus will be read by the acc.read() function
        while (!(acc.status() & DATA_READY));
        acc.read(&xyz);
        // The reading data are in a set that contain 3-axis values
        // Save these three axis values in separate buffer below
        cali_buf[0][i] = xyz.x;
        cali_buf[1][i] = xyz.y;
        cali_buf[2][i] = xyz.z;
        delay(CALI_INTERVAL_TIME);
        SERIAL.print('.');
    }
    SERIAL.println('.');
    // Doing the average of the detecing values that used for calibration
    // Then print out on the serial
    for (int i = 0; i < 3; i++) {
        cali_data[i] =  deal_cali_buf(cali_buf[i]);
        if (2 == i) {
            cali_data[i] -= 10;
        }
        SERIAL.println(cali_data[i]);
    }
    SERIAL.println("Calibration OK!!");
}


void setup() {
    SERIAL.begin(115200); // Setting baud rate at 115200 
    acc.begin(); //Start transmitting of ADXL372
    acc.timing_ctrl(RATE_3200); //Setting the Sample time (400/800/1600/3200/6400 Hz)
    acc.power_ctrl(MEASUREMENT_MODE); //Setting the ADXL372 to Measurement Mode
    acc.measurement_ctrl(BW_1600, true); //Setting the bandwidth of the LPF(Need bo be no more than half of the sampling rate)
    acc.setActiveTime(10);  
    //***    And specfic information can be linked to the information from ADXL372 Datasheet
    //Setting the value of Activity Timer Register. The timer period depends on the ODR selected. At 3200 Hz and below, it is ~6.6MS
    //For activity detection to trigger, above threshold activity must be sustained for a time equal to the number of activity timer 
    //periods specified in the activity timer register
    calibration(); //Start Calibration function at the beginning
}

void loop() {
    if (acc.status() & DATA_READY) {
        acc.read(&xyz);
        x_out = (xyz.x - cali_data[0]) / 10.0;
        y_out = (xyz.y - cali_data[1]) / 10.0;
        z_out = (xyz.z - cali_data[2]) / 10.0;
        SERIAL.print((String)x_out+" "+(String)y_out+" "+(String)z_out+"\n"); 
        delay(200);
 }
}

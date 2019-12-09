#include "mbed.h"

#include <vector>
#include <math.h>

#define BMA220_ADDR     0x0A // I2C Address of the sensor

#define XAXIS           0x04 // x-axis acceleration value register
#define YAXIS           0x06 // y-axis acceleration value register
#define ZAXIS           0x08 // z-axis acceleration value register

#define SLOPE_REG       0x12 // slope detection parameter

#define INTRP_MODE_REG  0x1A // interrupt selection
#define SLEEP_REG       0x1E // sleep enable and duration
#define SOFTRESET_REG   0x32 // softreset (triggered by reading)
#define SENSITIVITY_REG 0x22 // sensitivity selection
#define FILTER_REG      0x20 // filter selection
#define CHIPID_REG      0x00 // chip id
#define REVISIONID_REG  0x02 // revision id
#define INTRP_RES_REG   0x1C // interrupt reset, latch mode

#define SENS_2g         0x00 // Empfindlichkeit: +- 2g
#define SENS_4g         0x01 // Empfindlichkeit: +- 4g
#define SENS_8g         0x02 // Empfindlichkeit: +- 8g
#define SENS_16g        0x03 // Empfindlichkeit: +- 16g

#define FILTER_32Hz     0x05 // cutoff frequency: 32 Hz
#define FILTER_64Hz     0x04 // cutoff frequency: 64 Hz
#define FILTER_125Hz    0x03 // cutoff frequency: 125 Hz
#define FILTER_250Hz    0x02 // cutoff frequency: 250 Hz
#define FILTER_500Hz    0x01 // cutoff frequency: 500 Hz
#define FILTER_1kHz     0x00 // cutoff frequency: 1 kHz

#define LATCH_0s        0x80 // reset interrupt controller, latch time 0s
#define LATCH_025s      0x90 // reset interrupt controller, latch time 0.25s
#define LATCH_05s       0xA0 // reset interrupt controller, latch time 0.5s
#define LATCH_1s        0xB0 // reset interrupt controller, latch time 1s
#define LATCH_2s        0xC0 // reset interrupt controller, latch time 2s
#define LATCH_4s        0xD0 // reset interrupt controller, latch time 4s
#define LATCH_8s        0xE0 // reset interrupt controller, latch time 8s
#define LATCH_PERM      0xF0 // reset interrupt controller, latch permanently

#define SLEEP_2ms       0b01000000 // sleep enabled, 2 ms
#define SLEEP_10ms      0b01001000 // sleep enabled, 10 ms
#define SLEEP_25ms      0b01010000 // sleep enabled, 25 ms
#define SLEEP_50ms      0b01011000 // sleep enabled, 50 ms
#define SLEEP_100ms     0b01100000 // sleep enabled, 100 ms
#define SLEEP_500ms     0b01101000 // sleep enabled, 500 ms
#define SLEEP_1s        0b01110000 // sleep enabled, 1 s
#define SLEEP_2s        0b01111000 // sleep enabled, 2 s

#define SLOPEDETECT     0x38 // select envelope slope detection
#define SLOPEPAR1       0x0D // slope detection parameter: threshold: 0011, duration: 01

#define ONEBYTE         0x01 // read one byte

#define PI 3.1415926535897932384626433832795

template<typename T>
std::vector<std::vector<float> > MatrixMult(const std::vector<std::vector<T> > Matrix1, const std::vector<std::vector<T> > Matrix2){

    if((int)Matrix1[0].size()!=(int)Matrix2.size()){
        //pc.printf("Wrong Matrix Size\n");
        return Matrix1;
    }

    std::vector<std::vector<float> > result;
    result.resize(Matrix1.size(), std::vector<float>(Matrix2[0].size()));

    for (int i=0; i<(int)Matrix1.size(); i++){
        for (int j = 0; j<(int)Matrix2[0].size(); j++){
            result[i][j]=0;
            for (int k = 0; k<(int)Matrix1[0].size(); k++){
                result[i][j]+=((float)Matrix1[i][k])*((float)Matrix2[k][j]);
            }
        }
    }
    return result;
}

/*template<typename T>
void display(std::vector<std::vector<T> > A){
    for (int i=0; i<(int)A.size(); i++){
        for (int j=0; j<(int)A[0].size(); j++){
            pc.printf("%i", A[i][j]);
            pc.printf("\t");
        }
        pc.printf("\n");
    }
}*/


struct Acceleration{
    float Ax, Ay, Az;
};

struct Quanterion{
    float q0, q1, q2, q3, angle;

    /*void display(){
        Angle();
        pc.printf("%f+ %fi+ %fj+ %fk \t ANgle is: %f",q0, q1, q2, q3, angle);
    }*/

    void Angle(){
        angle=2*atan(sqrt(pow(q1, 2) + pow(q2, 2)+ pow(q3, 2))/q0)*180/PI;
    }
};

class BMA220{
    private:
        float Ax, Ay, Az, pitch, roll, theta;
        uint8_t resetvalue;

        Acceleration G[2];
        Quanterion Q[3];

        void Calculate_Q1();
        void Calculate_Q2();
        void Calculate_Q3();

        std::vector<std::vector<float> > Quanterion_2_Matrix(const Quanterion &Q);

    public:
        BMA220();
        bool begin(void);

        bool set(uint8_t reg, uint8_t value);
        bool read(uint8_t reg, uint8_t *pvalue);
        
        void setRegister(uint8_t reg, uint8_t value);
        int8_t readRegister(uint8_t reg);

        void readAcceleration(int sensitivity);
        float getAcceleration_X() const;
        float getAcceleration_Y() const;
        float getAcceleration_Z() const;
        float getPitch() const;
        float getRoll() const;
        float getTheta() const;

        float getMag() const;

        void reset(void);

        void FallDetection();
        Quanterion Q_result1, Q_result2;

        /*uint8_t reset(void);
        uint8_t chipID(void);
        uint8_t revisionID(void);*/
};
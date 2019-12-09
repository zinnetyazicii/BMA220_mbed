#include "bma220.h"

I2C i2c(I2C_SDA, I2C_SCL);
Serial dbug(USBTX, USBRX);

BMA220::BMA220(){
    G[0].Ax=0; G[0].Ay=0; G[0].Az=0;
}

bool BMA220::begin(void){
    resetvalue=readRegister(SOFTRESET_REG); // change mode
    if (resetvalue == 0xFF) {  // was in reset mode, is in operation mode now
        resetvalue = readRegister(SOFTRESET_REG);  // sensor should be in reset mode now
    }

    if (resetvalue != 0x00) {
        return false;
    }
    // sensor should be in reset mode
    resetvalue = readRegister(SOFTRESET_REG);
    if (resetvalue != 0xFF) {  // sensor was not in reset mode
        return false;
    }
    return true;
}

bool BMA220::set(uint8_t reg, uint8_t value){
  char ach_i2c_data[2];
  ach_i2c_data[0]=reg;
  ach_i2c_data[1]=value;
  
  if(i2c.write(BMA220_ADDR, ach_i2c_data, 2, false)==0)
    return true;
  else
    return false;
}

bool BMA220::read(uint8_t reg, uint8_t *pvalue){
  char data=reg;
  
  if(i2c.write(BMA220_ADDR, &data, 1, true)!=0)
    return false;
  if(i2c.read(BMA220_ADDR, &data, 1, false)==0)
  {
    *pvalue=(uint8_t) data;
    return true;
  }
  else return false;
}

void BMA220::setRegister(uint8_t reg, uint8_t value){
  if(set(reg, value)){return;}
  //pc.printf("Error: Register Not Set\n");
}

int8_t BMA220::readRegister(uint8_t reg){
  uint8_t pvalue;
//  char data=reg;
//  i2c.read(BMA220_ADDR, &data, 1);
//  pvalue=(uint8_t) data;
  if(read(reg, &pvalue)){return pvalue;}
  dbug.printf("Error: Register Not read\n");
  //return pvalue;
}

void BMA220::readAcceleration(int sensitivity){
    G[1].Ax=G[0].Ax; G[1].Ay=G[0].Ay; G[1].Az=G[0].Az;      //Acc After= Acc Before

    this->Ax=((float)readRegister(XAXIS))/sensitivity;
    this->Ay=((float)readRegister(YAXIS))/sensitivity;
    this->Az=((float)readRegister(ZAXIS))/sensitivity;
    
    // Calculate Roll and Pitch (rotation around X-axis, rotation around Y-axis)
    // Pitch (rho) is defined as the angle of the X-axis relative to ground. Roll (phi) is defined as the angle of the Y-axis relative to the ground. Theta is the angle of the Z axis relative to gravity.”
    int r= atan(Ay / sqrt(pow(Ax, 2) + pow(Az, 2)))*180/PI;
    int p= atan(Ax / sqrt(pow(Ay, 2) + pow(Az, 2)))*180/PI;
    int t=atan(sqrt(pow(Ax, 2) + pow(Ay, 2))/Az)*180/PI;

    // Low-pass filter
    this->roll=r;//(0.94*this->roll)+0.06*r;
    this->pitch=p;//(0.94*this->pitch)+0.06*p;
    this->theta=t;//(0.94*this->theta)+0.06*t;

    G[0].Ax=Ax; G[0].Ay=Ay; G[0].Az=Az;     //Acc Before=Present Acc

    FallDetection();
}

void BMA220::FallDetection(){
    Calculate_Q1();
    Calculate_Q2();
    Calculate_Q3();

    /*Serial.println("Q0: ");
    display(Quanterion_2_Matrix(this->Q[0]));
    Serial.println("Q1: ");
    display(Quanterion_2_Matrix(this->Q[1]));
    Serial.println("Q2: ");
    display(Quanterion_2_Matrix(this->Q[2]));*/


    std::vector<std::vector<float> > Q;
    Q.resize(4, std::vector<float>(4));

    Q=MatrixMult(MatrixMult(Quanterion_2_Matrix(this->Q[0]), Quanterion_2_Matrix(this->Q[1])), Quanterion_2_Matrix(this->Q[2]));

    this->Q_result1.q0=Q[0][0]; this->Q_result1.q1=Q[1][0]; this->Q_result1.q2=Q[2][0]; this->Q_result1.q3=Q[3][0]; 
    this->Q_result2.q0=Q[3][3]; this->Q_result2.q1=-1*Q[2][3]; this->Q_result2.q2=Q[1][3]; this->Q_result2.q3=-1*Q[0][3]; 
    
    //result1.display();
    //result2.display();
}

void BMA220::Calculate_Q1(){
    float theta1=atan(G[0].Az/ sqrt(pow(G[0].Ax, 2) + pow(G[0].Ay, 2)))*180/PI;
    float sin_alpha=(-1*(G[0].Ay/ sqrt(pow(G[0].Ax, 2) + pow(G[0].Ay, 2))))*180/PI;
    float cos_alpha=(G[0].Ax/ sqrt(pow(G[0].Ax, 2) + pow(G[0].Ay, 2)))*180/PI;

    this->Q[0].q0=cos(theta1/2);
    this->Q[0].q1=sin(theta1/2)*sin_alpha;
    this->Q[0].q2=sin(theta1/2)*cos_alpha;
    this->Q[0].q3=0;
}

void BMA220::Calculate_Q2(){
    float theta2=(atan((2*G[1].Ay)/G[1].Ax)-atan((2*G[0].Ay)/G[0].Ax))*180/PI;

    this->Q[1].q0=cos(theta2/2);
    this->Q[1].q1=0;
    this->Q[1].q2=0;
    this->Q[1].q3=sin(theta2/2);
}

void BMA220::Calculate_Q3(){
    float theta3=-1*atan(G[1].Az/ sqrt(pow(G[1].Ax, 2) + pow(G[1].Ay, 2)))*180/PI;
    float sin_beta=(-1*(G[1].Ay/ sqrt(pow(G[1].Ax, 2) + pow(G[1].Ay, 2))))*180/PI;
    float cos_beta=(G[1].Ax/ sqrt(pow(G[1].Ax, 2) + pow(G[1].Ay, 2)))*180/PI;

    this->Q[2].q0=cos(theta3/2);
    this->Q[2].q1=sin(theta3/2)*sin_beta;
    this->Q[2].q2=sin(theta3/2)*cos_beta;
    this->Q[2].q3=0;
}

std::vector<std::vector<float> > BMA220::Quanterion_2_Matrix(const Quanterion &Q){
    std::vector<std::vector<float> > result;
    result.resize(4, std::vector<float>(4));

    result[0][0]=Q.q0; result[0][1]=-1*Q.q1; result[0][2]=-1*Q.q2; result[0][3]=-1*Q.q3; 
    result[1][0]=Q.q1; result[1][1]=Q.q0; result[1][2]=-1*Q.q3; result[1][3]=-1*Q.q2; 
    result[2][0]=Q.q2; result[2][1]=Q.q3; result[2][2]=Q.q0; result[2][3]=-1*Q.q1;
    result[3][0]=Q.q3; result[3][1]=-1*Q.q2; result[3][2]=Q.q1; result[3][3]=Q.q0;  
    return result;
}


float BMA220::getAcceleration_X() const{
    return Ax;
}

float BMA220::getAcceleration_Y() const{
    return Ay;
}

float BMA220::getAcceleration_Z() const{
    return Az;
}

float BMA220::getPitch() const{
    return pitch;
}

float BMA220::getRoll() const{
    return roll;
}

float BMA220::getTheta() const{
    return theta;
}

float BMA220::getMag() const{
    return  sqrt(pow(Ax, 2) + pow(Ay, 2)+ pow(Az, 2));
}

void BMA220::reset(void) {;
    resetvalue = readRegister(SOFTRESET_REG);
    if (resetvalue == 0x00){
    // BMA220 is in reset mode now. Reading soft reset register
    // again, brings the sensor back to operation mode.
    resetvalue = readRegister(SOFTRESET_REG);
    }
}

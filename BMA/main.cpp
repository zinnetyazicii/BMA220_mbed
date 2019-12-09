#include <bma220.h>

BMA220 Sensor;
Serial pc(USBTX, USBRX);

void display(Quanterion Q){
    pc.printf("%f+ %fi+ %fj+ %fk \t ANgle is: %f\n",Q.q0, Q.q1, Q.q2, Q.q3, Q.angle);
}

int main() {
    pc.baud(115200);
    if (!Sensor.begin()) {
        pc.printf("No valid BMA220 sensor found, check wiring");
        while (true){  // stop here, no reason to go on...
            pc.printf("Nothing...\n");
            wait(5);
        }
    }
    // Set sensor sensitivity to 4g
    Sensor.setRegister(SENSITIVITY_REG, SENS_2g);
    
    while(1){
        Sensor.readAcceleration(64);
        pc.printf("x-axis: ");
        pc.printf("%f\n", Sensor.getAcceleration_X());
        pc.printf("y-axis: ");
        pc.printf("%f\n",Sensor.getAcceleration_Y());
        pc.printf("z-axis: ");
        pc.printf("%f\n",Sensor.getAcceleration_Z());
    
        pc.printf("rho: ");
        pc.printf("%f\n",Sensor.getPitch());
    
        pc.printf("phi: ");
        pc.printf("%f\n",Sensor.getRoll());
    
        pc.printf("theta: ");
        pc.printf("%f\n",Sensor.getTheta());
    
        pc.printf("Mag: ");
        pc.printf("%f\n",Sensor.getMag());
    
        Sensor.FallDetection();
        display(Sensor.Q_result1);
        display(Sensor.Q_result2);
        
    //    pc.printf(Sensor.getRoll());
    //    pc.printf("/");
    //    pc.printf(Sensor.getPitch());
        wait(1);
    }
}
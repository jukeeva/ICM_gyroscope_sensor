#include <Wire.h>
#include "ICM20649.h"

ICM20649 sensor;
void setup()
{
    Serial.begin(115200);
    sensor.init();
   // sensor.zeroCalibrate(200,10);//sample 200 times to calibrate and it will take 200*10ms

}

void loop()
{
    
    int16_t a,b,c;
    int16_t x,y,z;
    int8_t testX, testY, testZ;
    sensor.getXYZ(&x,&y,&z, &testX, &testY, &testZ); // x y z and test values stand for raw gyroscope values
    sensor.getACC_XYZ( &a, &b, &c); // a b c stand for raw accelerometer values
    /*Serial.print("\n RAW values of gyro X , Y , Z: ");
    Serial.print(x);
    Serial.print(" , ");
    Serial.print(y);
    Serial.print(" , ");
    Serial.println(z);
    
    Serial.print("\n RAW values of acc X , Y , Z: ");
    Serial.print(a);
    Serial.print(" , ");
    Serial.print(b);
    Serial.print(" , ");
    Serial.println(c);*/
    Serial.print("\n self test values of gyro X , Y , Z: ");
    Serial.print(testX);
    Serial.print(" , ");
    Serial.print(testY);
    Serial.print(" , ");
    Serial.println(testZ);
    
    /*float ax,ay,az; //  GYRO x y z degree/sec values
    float ga,gb,gc; // ACC x y z meters/ sec^2 vaLues
    sensor.getAngularVelocity(&ax,&ay,&az);
    Serial.print("\n  GX: ");
    Serial.print(ax);
    Serial.print(" GY: ");
    Serial.print(ay);
    Serial.print(" GZ: ");
    Serial.print(az);
    Serial.print(" the angular velocity ");
    sensor.getG_force(&ga, &gb, &gc);
    Serial.print("\n AX: ");
    Serial.print(ga);
    Serial.print(" AY: ");
    Serial.print(gb);
    Serial.print(" AZ: ");
    Serial.print(gc);
    Serial.print(" the g force ");*/
    

   // delay(1000);
}

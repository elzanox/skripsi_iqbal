/**
  @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
  @file         CarRun.c
  @author       liusen
  @version      V1.0
  @date         2017.07.25
  @brief       CarRun
  @details
  @par History

*/
#include <Wire.h>
#include "./Adafruit_PWMServoDriver.h"
#include <RPLidar.h>
#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor (MOTOCTRL).

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
RPLidar lidar;

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

float distance, angle;

/**
  Function       setup
  @author        liusen
  @date          2017.07.25
  @brief         initialization configure
  @param[in]     void
  @retval        void
  @par History
*/
void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);  // For RPLidar
  lidar.begin(Serial1);
  pinMode(RPLIDAR_MOTOR, OUTPUT);  // set pin modes

  pwm.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~60 Hz updates
  delay(5000);
  Serial.println("setup");
}

/**
  Function       run
  @author        liusen
  @date          2017.07.25
  @brief         run
  @param[in]     time
  @param[out]    void
  @retval        void
  @par History
*/
float minDistance = 100000;
float angleAtMinDist = 0;

void back(int Speed)
{

  pwm.setPWM(8, 0, Speed);
  pwm.setPWM(9, 0, 0);
  pwm.setPWM(11, 0, Speed);
  pwm.setPWM(10, 0, 0);

  pwm.setPWM(0, 0, Speed);
  pwm.setPWM(1, 0, 0);
  pwm.setPWM(3, 0, Speed);
  pwm.setPWM(2, 0, 0);

  pwm.setPWM(12, 0, Speed);
  pwm.setPWM(13, 0, 0);
  pwm.setPWM(15, 0, Speed);
  pwm.setPWM(14, 0, 0);
}

/**
  Function       back
  @author        liusen
  @date          2017.07.25
  @brief         back
  @param[in]     time
  @param[out]    void
  @retval        void
  @par History
*/
void run(int Speed)
{
  pwm.setPWM(8, 0, 0);
  pwm.setPWM(9, 0, Speed);
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(10, 0, Speed);

  pwm.setPWM(0, 0, 0);
  pwm.setPWM(1, 0, Speed);
  pwm.setPWM(3, 0, 0);
  pwm.setPWM(2, 0, Speed);

  pwm.setPWM(12, 0, 0);
  pwm.setPWM(13, 0, Speed);
  pwm.setPWM(15, 0, 0);
  pwm.setPWM(14, 0, Speed);

}

/**
  Function       brake
  @author        liusen
  @date          2017.07.25
  @brief         brake
  @param[in]     time
  @param[out]    void
  @retval        void
  @par History   no
*/
void brake()
{

  pwm.setPWM(8, 0, 0);
  pwm.setPWM(9, 0, 0);
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(10, 0, 0);

  pwm.setPWM(0, 0, 0);
  pwm.setPWM(1, 0, 0);
  pwm.setPWM(3, 0, 0);
  pwm.setPWM(2, 0, 0);

  pwm.setPWM(12, 0, 0);
  pwm.setPWM(13, 0, 0);
  pwm.setPWM(15, 0, 0);
  pwm.setPWM(14, 0, 0);
}

/**
  Function       left
  @author        liusen
  @date          2017.07.25
  @brief         turn left(left wheel stop,right wheel advance)
  @param[in]     time
  @param[out]    void
  @retval        void
  @par History
*/
void left(int Speed)
{
  pwm.setPWM(8, 0, Speed);
  pwm.setPWM(9, 0, 0);
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(10, 0, Speed);

  pwm.setPWM(0, 0, Speed);
  pwm.setPWM(1, 0, 0);
  pwm.setPWM(3, 0, 0);
  pwm.setPWM(2, 0, Speed);

  pwm.setPWM(12, 0, Speed);
  pwm.setPWM(13, 0, 0);
  pwm.setPWM(15, 0, 0);
  pwm.setPWM(14, 0, Speed);


}

/**
  Function       right
  @author        liusen
  @date          2017.07.25
  @brief         turn right(left wheel advance,right wheel stop)
  @param[in]     time
  @param[out]    void
  @retval        void
  @par History   no
*/
void right(int Speed)
{
  pwm.setPWM(8, 0, 0);
  pwm.setPWM(9, 0, Speed);
  pwm.setPWM(11, 0, Speed);
  pwm.setPWM(10, 0, 0);

  pwm.setPWM(0, 0, 0);
  pwm.setPWM(1, 0, Speed);
  pwm.setPWM(3, 0, Speed);
  pwm.setPWM(2, 0, 0);


  pwm.setPWM(12, 0, 0);
  pwm.setPWM(13, 0, Speed);
  pwm.setPWM(15, 0, Speed);
  pwm.setPWM(14, 0, 0);


}

/**
  Function       spin_left
  @author        liusen
  @date          2017.07.25
  @brief         turn left in place(left wheel back,right wheel advance)
  @param[in]     time
  @param[out]    void
  @retval        void
  @par History   no
*/
void spin_left(int Speed)
{
  pwm.setPWM(8, 0, Speed);
  pwm.setPWM(9, 0, 0);
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(10, 0, Speed);

  pwm.setPWM(0, 0, Speed);
  pwm.setPWM(1, 0, 0);
  pwm.setPWM(3, 0, 0);
  pwm.setPWM(2, 0, Speed);


  pwm.setPWM(12, 0, Speed);
  pwm.setPWM(13, 0, 0);
  pwm.setPWM(15, 0, 0);
  pwm.setPWM(14, 0, Speed);

}

/**
  Function       spin_right
  @author        liusen
  @date          2017.07.25
  @brief         turn right in place(left wheel adavnce,right wheel back)
  @param[in]     time
  @param[out]    void
  @retval        void
  @par History   no
*/
void spin_right(int Speed)
{
  pwm.setPWM(8, 0, 0);
  pwm.setPWM(9, 0, Speed);
  pwm.setPWM(11, 0, Speed);
  pwm.setPWM(10, 0, 0);

  pwm.setPWM(0, 0, 0);
  pwm.setPWM(1, 0, Speed);
  pwm.setPWM(3, 0, Speed);
  pwm.setPWM(2, 0, 0);


  pwm.setPWM(12, 0, 0);
  pwm.setPWM(13, 0, Speed);
  pwm.setPWM(15, 0, Speed);
  pwm.setPWM(14, 0, 0);
}


/**
  Function       loop
  @author        liusen
  @date          2017.07.25
  @brief         delay 2s，run 1s，back 1s,turn left 2s,turn right 2s,
                 turn left in place 3s,turn right in place 3s,stop 0.5s
  @param[in]     void
  @retval        void
  @par History
*/
void printData(float angle, float distance)
{
  Serial.print("dist: ");
  Serial.print(distance);
  Serial.print("    angle: ");
  Serial.println(angle);
}
void loop()
{
  //  analogWrite(RPLIDAR_MOTOR, 255); //atur 0-255 untuk kecepatan motor
  //
  if (IS_OK(lidar.waitPoint())) {
    //perform data processing here...
    distance = lidar.getCurrentPoint().distance;
    angle = lidar.getCurrentPoint().angle;  // 0-360 deg



    //    Serial.println(minDistance);
    if (lidar.getCurrentPoint().startBit) {
      // a new scan, display the previous data...
      //      printData(angleAtMinDist, minDistance);

      if ((minDistance > 200) && (minDistance <= 400) && (angleAtMinDist > 310 || angleAtMinDist <= 50) ) {
        Serial.print("Tahap 1 = (Kurangi Kecepatan > )");
        Serial.print("Jarak: ");
        Serial.print(minDistance);
        Serial.print(" angle: ");
        Serial.println(angleAtMinDist);
        run(2048);
      }
      else if ((minDistance >= 50) && (minDistance <= 200) && (angleAtMinDist > 310 || angleAtMinDist <= 50)) {
        Serial.print("Tahap 2 = Sudah Dekat Objek > ");
        
        Serial.print("Jarak: " );
        Serial.print(minDistance);
        Serial.print(" angle: ");
        Serial.println(angleAtMinDist);
        brake();
        Serial.println("Robot Berhenti");
        delay(3000);
        back(2048);
        Serial.println("Robot Mundur");
        delay(1000);
        spin_left(2024);
        Serial.println("Robot Berbelok");
        delay(2000);
      }

      else {
        Serial.print("Robot Jalan > ");
        Serial.print("Jarak: ");
        Serial.print(minDistance);
        Serial.print(" angle: ");
        Serial.println(angleAtMinDist);
        run(4095);
        //          delay(1000);
      }
      minDistance = 100000;
      angleAtMinDist = 0;
    } else {
      if ( distance > 0 &&  distance < minDistance) {
        minDistance = distance;
        angleAtMinDist = angle;
      }
    }
  }
  else {
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
    // Try to detect RPLIDAR
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
      // Detected
      lidar.startScan();
      analogWrite(RPLIDAR_MOTOR, 255); //atur 0-255 untuk kecepatan motor
      delay(1000);
    }
  }

  //  if (distance < 150) { /*if there's an obstacle 25 centimers, ahead, do the following: */
  //    Serial.println ("Close Obstacle detected!" );
  //    Serial.println ("Obstacle Details:");
  //    Serial.print ("Distance From Robot is " );
  //    Serial.print ( distance);
  //    Serial.print ( " CM!");// print out the distance in centimeters.
  //
  //    Serial.println (" The obstacle is declared a threat due to close distance. ");
  //    Serial.println (" Turning !");
  ////    spin_left(4095);
  ////    delay(2000);
  //
  //  }
  //  else {
  //    Serial.println ("No obstacle detected. going forward");
  //    delay (15);
  //    run(100);
  //
  //  }
  //    run(4095);  //The speed range is: 0~4095
  //    delay(5000);
  //    back(4095);  //The speed range is: 0~4095
  //    delay(5000);
  //    spin_left(4095);  //The speed range is: 0~4095
  //    delay(2000);
  //    spin_right(4095);  //The speed range is: 0~4095
  //    delay(2000);
  //    brake();
  //    delay(2000);
  //  Serial.println("loop");
}

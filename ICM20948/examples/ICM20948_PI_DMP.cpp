#include "Pi-ICM20948.h"
#include <iostream>
#include "CircularBuffer.h"
#include "SensorTypes.h"



CircularBuffer cb;

inv_sensor_event_t event;

PIICM20948 icm20948;
PIICM20948Settings icmSettings =
{
  .i2c_speed = 400000,                // i2c clock speed
  .is_SPI = true,                    // Enable SPI, if disable use i2c
  .spi_speed = 100000,               // SPI clock speed in Hz, max speed is 7MHz
  .mode = 1,                          // 0 = low power mode, 1 = high performance mode
  .enable_gyroscope = true,           // Enables gyroscope output
  .enable_accelerometer = true,       // Enables accelerometer output
  .enable_magnetometer = true,        // Enables magnetometer output // Enables quaternion output
  .enable_gravity = true,             // Enables gravity vector output
  .enable_linearAcceleration = true,  // Enables linear acceleration output
  .enable_quaternion6 = true,         // Enables quaternion 6DOF output
  .enable_quaternion9 = true,         // Enables quaternion 9DOF output
  .enable_har = true,                 // Enables activity recognition
  .enable_steps = true,               // Enables step counter
  .gyroscope_frequency = 1,           // Max frequency = 225, min frequency = 1
  .accelerometer_frequency = 1,       // Max frequency = 225, min frequency = 1
  .magnetometer_frequency = 1,        // Max frequency = 70, min frequency = 1 
  .gravity_frequency = 1,             // Max frequency = 225, min frequency = 1
  .linearAcceleration_frequency = 1,  // Max frequency = 225, min frequency = 1
  .quaternion6_frequency = 50,        // Max frequency = 225, min frequency = 50
  .quaternion9_frequency = 50,        // Max frequency = 225, min frequency = 50
  .har_frequency = 50,                // Max frequency = 225, min frequency = 50
  .steps_frequency = 50,               // Max frequency = 225, min frequency = 50
  
  
};

const uint8_t number_i2c_addr = 2;
uint8_t poss_addresses[number_i2c_addr] = {0X69, 0X68};
uint8_t ICM_address;
bool ICM_found = false;



void run_icm20948_quat6_controller(bool inEuler = false)
{
    float quat_w, quat_x, quat_y, quat_z;
    float roll, pitch, yaw;
    char sensor_string_buff[128];
    if (inEuler)
    {
        if (icm20948.euler6DataIsReady())
        {
            icm20948.readEuler6Data(&roll, &pitch, &yaw);
            std::cout << "Euler6 roll, pitch, yaw (deg): ["<< roll << ", " << pitch << ", " << yaw << "]" << std::endl;
        }
    }
    else
    {
        if (icm20948.quat6DataIsReady())
        {
            icm20948.readQuat6Data(&quat_w, &quat_x, &quat_y, &quat_z);
            std::cout<< "Quat6 w, x, y, z (deg): ["<<quat_w<<","<< quat_x <<","<<quat_y<<","<<quat_z<<"]"<<std::endl;
        }
    }
    
}
void run_icm20948_quat9_controller(bool inEuler = false)
{
    float quat_w, quat_x, quat_y, quat_z;
    float roll, pitch, yaw;
    char sensor_string_buff[128];
    if (inEuler)
    {
        if (icm20948.euler9DataIsReady())
        {
            icm20948.readEuler9Data(&roll, &pitch, &yaw);
            std::cout << "Euler9 roll, pitch, yaw (deg): ["<< roll << ", " << pitch << ", " << yaw << "]" << std::endl;

          
        }
    }
    else
    {
        if (icm20948.quat9DataIsReady())
        {
            icm20948.readQuat9Data(&quat_w, &quat_x, &quat_y, &quat_z);
            std::cout<< "Quat9 w, x, y, z (deg): ["<<quat_w<< ","<<quat_w<<","<< quat_x <<","<<quat_y<<","<<quat_z<<"]"<<std::endl;
            
        }
    }
    
}
void run_icm20948_accel_controller()
{
    float x, y, z;
    char sensor_string_buff[128];
    if (icm20948.accelDataIsReady())
    {
        icm20948.readAccelData(&x, &y, &z);
        std::cout<<sensor_string_buff, "Acceleration x, y, z (g): [%f,%f,%f]", x, y, z;
        std::cout<<sensor_string_buff;
    }
    
}
void run_icm20948_gyro_controller()
{
    float x, y, z;
    char sensor_string_buff[128];
    if (icm20948.gyroDataIsReady())
    {
        icm20948.readGyroData(&x, &y, &z);
        std::cout<<sensor_string_buff, "Gyroscope x, y, z (rad/s): [%f,%f,%f]", x, y, z;
        std::cout<<sensor_string_buff;
    }
    
}
void run_icm20948_mag_controller()
{
    float x, y, z;
    char sensor_string_buff[128];
    if (icm20948.magDataIsReady())
    {
        icm20948.readMagData(&x, &y, &z);
        std::cout<<sensor_string_buff, "Magnetometer x, y, z (mT): [%f,%f,%f]", x, y, z;
        std::cout<<sensor_string_buff;
    }
    
}
void run_icm20948_linearAccel_controller()
{
    float x, y, z;
    char sensor_string_buff[128];
    if (icm20948.linearAccelDataIsReady())
    {
        icm20948.readLinearAccelData(&x, &y, &z);
        std::cout<<sensor_string_buff, "Linear Acceleration x, y, z (g): [%f,%f,%f]", x, y, z;
        std::cout<<sensor_string_buff;
    }
    
}
void run_icm20948_grav_controller()
{
    float x, y, z;
    char sensor_string_buff[128];
    if (icm20948.gravDataIsReady())
    {
        icm20948.readGravData(&x, &y, &z);
        std::cout<<sensor_string_buff, "Gravity Vector x, y, z (g): [%f,%f,%f]", x, y, z;
        std::cout<<sensor_string_buff;
    }
    
}
void run_icm20948_har_controller()
{
    char activity;
    char sensor_string_buff[128];
    if (icm20948.harDataIsReady())
    {
        icm20948.readHarData(&activity);
        std::cout<<sensor_string_buff, "Current Activity : %c", activity;
        std::cout<<sensor_string_buff;
    }
    
}
void run_icm20948_steps_controller()
{
    unsigned long steps;
    char sensor_string_buff[128];
    if (icm20948.stepsDataIsReady())
    {
        icm20948.readStepsData(&steps);
        std::cout<<sensor_string_buff, "Steps Completed : %lu", steps;
        std::cout<<sensor_string_buff;
    }
    
}



int main(){
    
    init_buffer(&cb, 20, sizeof(event));
    icm20948.init(icmSettings);

    while(true){

        icm20948.task();
       
        if(!check_empty(&cb)){
            pull_data(&cb,&event); 
            std::cout<<event.data.quaternion6DOF.quat[0]<<","<<event.data.quaternion6DOF.quat[1]<<","<<event.data.quaternion6DOF.quat[2]<<","<<event.data.quaternion6DOF.quat[3]<<","<<std::endl;
            std::cout<<event.timestamp<<std::endl;
        }
        
        
    }
    exit(0);
}


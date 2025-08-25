#include "Pi-ICM20948.h"
#include <iostream>
#include <thread>
#include <atomic>
#include "CircularBuffer.h"
#include "SensorTypes.h"



CircularBuffer cb;
std::atomic<bool> running(true);
inv_sensor_event_t event;

PIICM20948 icm20948;
PIICM20948Settings icmSettings =
{
  .i2c_speed = 400000,                // i2c clock speed
  .is_SPI = true,                    // Enable SPI, if disable use i2c
  .spi_speed = 7000000,               // SPI clock speed in Hz, max speed is 7MHz
  .mode = 1,                          // 0 = low power mode, 1 = high performance mode
  .enable_gyroscope = false,           // Enables gyroscope output
  .enable_accelerometer = true,       // Enables accelerometer output
  .enable_magnetometer = false,        // Enables magnetometer output // Enables quaternion output
  .enable_gravity = false,             // Enables gravity vector output
  .enable_linearAcceleration = false,  // Enables linear acceleration output
  .enable_quaternion6 = true,         // Enables quaternion 6DOF output
  .enable_quaternion9 = false,         // Enables quaternion 9DOF output
  .enable_har = false,                 // Enables activity recognition
  .enable_steps = false,               // Enables step counter
  .gyroscope_frequency = 1,           // Max frequency = 225, min frequency = 1
  .accelerometer_frequency = 225,       // Max frequency = 225, min frequency = 1
  .magnetometer_frequency = 1,        // Max frequency = 70, min frequency = 1 
  .gravity_frequency = 1,             // Max frequency = 225, min frequency = 1
  .linearAcceleration_frequency = 1,  // Max frequency = 225, min frequency = 1
  .quaternion6_frequency = 225,        // Max frequency = 225, min frequency = 50
  .quaternion9_frequency = 50,        // Max frequency = 225, min frequency = 50
  .har_frequency = 50,                // Max frequency = 225, min frequency = 50
  .steps_frequency = 50,               // Max frequency = 225, min frequency = 50
  
  
};






int main(){
    
    init_buffer(&cb, 20, sizeof(event));
    icm20948.init(icmSettings);
    
    std::thread t1([](){
                    while(running){        
                            if(!check_full(&cb)){icm20948.task();};};});
    std::thread t2([](){
                    while(running){
                            if(!check_empty(&cb)){  
                            pull_data(&cb,&event); 
                            std::cout<<event.data.quaternion6DOF.quat[0]<<","
                                     <<event.data.quaternion6DOF.quat[1]<<","
                                     <<event.data.quaternion6DOF.quat[2]<<","
                                     <<event.data.quaternion6DOF.quat[3]<<","
                                     <<event.timestamp<<","<<std::endl;   
                            };
                            std::this_thread::sleep_for(std::chrono::milliseconds(20));
                            };});
 
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    running =false;
    t1.join();
    t2.join();
    exit(0);
}


#include "MCP23017.h"
#include "WattBob_TextLCD.h"
#include "mbed.h"
#include <queue>

//Definitions for switch ports
#define ENGINE_SWITCH 8                 //Switch 1
#define ACCEL_SWITCH 9                  //Switch 2
#define BRAKES_SWITCH 10                //Switch 3
#define CC_SWITCH 11                    //Switch 4    

//Definitions for simulation variables
#define MIN_SPEED 0
#define MAX_SPEED 300
#define LEGAL_SPEED 142                 //88 mph in km/h , km/h used instead of m/s for more realism for a car's display
#define CRUISE_SPEED 80                 //50 mph in km/h
#define CRUISE_BIAS 0.1                 //Bias to improve cruise control's ability to reach cruise speed swiftly
#define FRICTION 0.001
#define FRICTION_BIAS 0.8               //Bias required for cruise control to reach cruise speed, generally speaking should be set to CRUISE_SPEED * FRICTION

WattBob_TextLCD *lcd;                   //pointer to 2*16 character LCD object
MCP23017 *par_port;                     //pointer to 16-bit parallel I/O object

DigitalOut engine_indicator(LED1);      //output for LED1
DigitalOut cruising_indicator(LED2);    //output for LED2
DigitalOut speeding_indicator(LED3);    //output for LED3

//Task functions
void calcAverageSpeed();
void displayToLCD();
void cruiseControl();
void readInputs();
void simulateCar();

//Init variables
bool ignition(0);
bool cruise_mode(0);
float accel(0);
float brakes(0);
float current_speed(0);
float average_speed(0);
float odometry(0);

//Queue to store previous speeds
std::deque<int> avg_speed_queue; 

//Init mutexes for sharing resources between threads
Mutex engineMutex;
Mutex portMutex;
Mutex avgSpeedMutex;

//Init Threads
Thread thread2Hz;
Thread thread5Hz;
Thread threadSim;
Thread thread20Hz;
Thread thread25Hz;

/*
################################################################################
Function to perform Task 1, 2 and 3
Reads switch inputs for ignition, accelerator and brakes.
Disables setting of digital input values if cruise mode is enabled.
Runs at 25 Hz
################################################################################
*/
void readInputs(){
    while(true){
        portMutex.lock();                                   //Let Mutexes wait  
        
        ignition = par_port->read_bit(ENGINE_SWITCH);       //Read ignition switch and set digital input
        engine_indicator = ignition;                        //Set LED to digital input's value (on/off)
        
        if(!cruise_mode){                                   //Only read accel & brake inputs when not in cruise mode
            accel = par_port->read_bit(ACCEL_SWITCH);       //Read accel switch and set digital input
            brakes = par_port->read_bit(BRAKES_SWITCH);     //Read brakes switch and set digital input
        }
                                                            
        portMutex.unlock();                                 //Unlock Mutexes
        ThisThread::sleep_for(40);                          //Sleep thread
    }
}

/*
################################################################################
Function to perform Task 5
Monitors speed and calculates the average speed over 3 readings.
If average speed goes above 142 km/h (=88mph) turns on speeding indicator.
Runs at 5 Hz
################################################################################
*/
void calcAverageSpeed(){
    while(true){
        avgSpeedMutex.lock();                               //Let Mutexes wait
        
        float sum(0);                                       //Init sum variable
        for(auto it = avg_speed_queue.begin();it!=avg_speed_queue.end(); ++it){      
            sum+= *it;                                      //Add average speeds stored in queue to sum
        }
        average_speed = sum/avg_speed_queue.size();         //Update average speed
        speeding_indicator = (average_speed > LEGAL_SPEED); //Turn LED on if average speed is over the allowed speed
        
        avgSpeedMutex.unlock();                             //Unlock Mutexes
        ThisThread::sleep_for(200);                         //Sleep thread
    }
}

/*
################################################################################
Function to perform Task 6
Displays odometer value and average speed on LCD display
Runs at 2 Hz
################################################################################
*/
void displayToLCD(){
    while(true){
        avgSpeedMutex.lock();                               //Let Mutexes wait                                 
        portMutex.lock();
                                                            //Print average speed and odometry to LCD
        lcd->locate(0,0);                                   
        lcd->printf("speed: %9.1f", average_speed);
        lcd->locate(1,0);
        lcd->printf("odom : %9.1f", odometry);
                                                            
        avgSpeedMutex.unlock();                             //Unlock Mutexes
        portMutex.unlock();
        
        ThisThread::sleep_for(500);                         //Sleep thread 
    }
}

/*
################################################################################
Function to perform Task 7
Reads cruise control switch input, and sets digital inputs for accelerator and 
brakes to the result of a very simple proportional controller.
FRICTION_BIAS is used to counter the deceleration due to drag, and CRUISE_BIAS
is used to ensure the rate at which the cruise control reaches its cruise speed
is not too slow.
Runs at 20 Hz
################################################################################
*/
void cruiseControl(){
    while(true){
        portMutex.lock();                                           //Let Mutexes wait
        
        cruise_mode = par_port->read_bit(CC_SWITCH);                //Read cruise control switch and set digital input
        cruising_indicator = ignition ? cruise_mode : 0;            //Set cruise control indicator LED to digital input value (on/off), but always off if ignition is off

        if(cruise_mode && ignition){                                //In cruise control mode only when ignition is on
                                                                    //If speed is above cruise speed, set accelarator to 0 and set brakes to proportional value
            if(current_speed>CRUISE_SPEED + FRICTION_BIAS){         
                accel = 0;
                brakes = (current_speed - CRUISE_SPEED)/CRUISE_SPEED + CRUISE_BIAS;
            }                                                       
                                                                    //If speed is below cruise speed, set brakes to 0 and set accelerator to proportional value
            else if(current_speed<CRUISE_SPEED + FRICTION_BIAS){    
                accel = (CRUISE_SPEED - current_speed)/CRUISE_SPEED + CRUISE_BIAS;
                brakes = 0;
            }
            else{                                                   //If speed is exactly the cruise speed, set both digital inputs to 0
                accel = 0;
                brakes = 0;
            }
        }
        
        portMutex.unlock();                                         //Unlock Mutexes
        ThisThread::sleep_for(50);                                  //Sleep thread
    }
}

/*
################################################################################
Function to perform simulation
Changes current speed depending on digital inputs, and regulates speed between
to keep speed between minimum and maximum speed defined in definitions above.
Updates the queue storing previous speed readings with new speed, and updates 
odometry value.
Runs at 25 Hz
################################################################################
*/
void simulateCar(){
    while(true){
        avgSpeedMutex.lock();                                       //Lock Mutexes
        
        if(ignition){                           
            current_speed += (accel-brakes);                        //Update current speed using digital inputs
        }
        else{
            accel = 0;
            current_speed += accel-0.5*brakes;                      //Accelator disabled if ignition is off, and reduced braking due to lack of assisted breaking
        }
        
       
        current_speed -= FRICTION*current_speed;                    //Speed reduction with a very basic implementation of drag
        
        if(current_speed<MIN_SPEED) current_speed = MIN_SPEED;      //Keep speed between minimum and maximum values previously defined
        if(current_speed>MAX_SPEED) current_speed = MAX_SPEED;
        if(avg_speed_queue.size()>3){                               //Update queue holding previous average speeds, only keep 3 latest average speeds
            avg_speed_queue.pop_front();
        }
        avg_speed_queue.push_back(current_speed);

        odometry += current_speed * 1.0/25.0;                       //Update odometry, keep in mind this thread runs at 25 Hz
        
        avgSpeedMutex.unlock();                                     //Unlock Mutexes
        ThisThread::sleep_for(40);                                  //Sleep thread
    }
}

/*
################################################################################
Main function
################################################################################
*/
int main(){
    par_port = new MCP23017(p9,p10,0x40);
    lcd = new WattBob_TextLCD(par_port);
    par_port->write_bit(1,BL_BIT);                                  //Turn LCD backlight on
    lcd->cls();                                                     //Clear LCD point to first element
    lcd->locate(0,0);
                                                                    //Start threads
    thread2Hz.start(displayToLCD);
    thread5Hz.start(calcAverageSpeed);
    thread20Hz.start(cruiseControl);
    threadSim.start(simulateCar);
    thread25Hz.start(readInputs);
                                                                    //Keep threads running
    while(1);
}

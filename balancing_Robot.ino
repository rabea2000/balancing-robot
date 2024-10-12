
#include <Wire.h>     



int MPU6050_ADDR = 0x68;   

float Kp = 6;                    
float Ki = 0.001; // 0.006 ;                  
float Kd = 5;

const int dir_1 = 2;
const int step_1 = 3;

const int dir_2 = 4;
const int step_2 = 5;


unsigned  long Loop_Time , temp , step_time  ;

int Activated ;
int left_motor;
int Left_Motor_Speed;
int Counter_Speed_Left_Motor; 
int Left_Motor_Speed_Prev;
int right_motor;
int Right_Motor_Speed;
int Counter_Speed_Right_Motor;
int Right_Motor_Speed_Prev;




float  PID_Value ;
float  PID_Value_left;
float  PID_Value_right;
float Moving_Speed = 10;          //Moving speed with Bluetooth Control; Mine was 20
float Max_Speed = 75;            //Max mooving speed; Mine was 160

float Gyro_X_Offset ,Gyro_X_Angle   ;

float   Acc_Angle ;

float   Gyro_X_Raw   ;
int     Acc_X_Raw ,  Acc_Y_Raw  ,Acc_Z_Raw  ;

float  Temp_Error, Auto_Setpoint , Setpoint ,Last_D_Error ,PID_I , Angle ; 
byte  Received_byte;
int Received_Since;




void set_config_mpu6050() ;
void set_timer ()  ;
void gyro_offest ();

void setup()
{

  Serial.begin(115200);
  Wire.begin();  
          
  TWBR = 12; 

  pinMode(dir_1  , OUTPUT);
  pinMode(step_1 , OUTPUT);

  pinMode(dir_2  , OUTPUT);
  pinMode(step_2, OUTPUT);


  set_config_mpu6050() ;          //config the mpu setting acc scale +/- 4g ---gyr scal +/- 250 degree/sec  

  
  set_timer ()         ;         // set timer2 to genrate puls every 20us

  gyro_offest ()       ;        // calclaut the error in gyro
 

 // --- the time sample of gyro and PID is 40 m/ses
  Loop_Time = micros() + 4000; 
}

void loop()
{
  
   temp = millis () ;


     if (Serial.available()) {                                    
    Received_byte = Serial.read();                              
    Received_Since = 0;                                         
  }
  if (Received_Since <= 25) {
    Received_Since ++;                                          
  }
  else Received_byte = 0x00;                                    

  
  // -----------------------------------CALCUALTE ANGLE----------------------------------------------------

  Wire.beginTransmission(MPU6050_ADDR);                          
  Wire.write(0x3B);                                               //Start reading register 3B where acc_x data storث
  Wire.endTransmission(false);                                         
  Wire.requestFrom(MPU6050_ADDR, 6,true);                              

  Acc_X_Raw = Wire.read() << 8 | Wire.read();                       
  Acc_Y_Raw = Wire.read() << 8 | Wire.read();
  Acc_Z_Raw = Wire.read() << 8 | Wire.read();
  
  
         // -------------limit the acc data 
         
   if (Acc_X_Raw > 8200)   Acc_X_Raw = 8200;                              
   if (Acc_X_Raw < -8200)  Acc_X_Raw = -8200;                            

   if (Acc_Y_Raw > 8200)   Acc_Y_Raw = 8200;
   if (Acc_Y_Raw < -8200)   Acc_Y_Raw = -8200;

   if (Acc_Z_Raw > 8200)   Acc_Z_Raw = 8200;
   if (Acc_Z_Raw < -8200)   Acc_Z_Raw = -8200;


    // ------ calcluate acc_angle 
    Acc_Angle = atan2(  Acc_Y_Raw   ,   Acc_Z_Raw  )*RAD_TO_DEG ;
  
  if ( Activated == 0 && Acc_Angle > -0.5 && Acc_Angle < 0.5)
   { 
       Gyro_X_Angle = 0   ; 
       Activated = 1      ;
   }  
                                       
  Wire.beginTransmission(MPU6050_ADDR);                           
  Wire.write(0x43);                                               
  Wire.endTransmission();                                         
  Wire.requestFrom(MPU6050_ADDR, 2 ,true);                             
  Gyro_X_Raw = Wire.read() << 8 | Wire.read();                    
  
  
  // ---- subtract the offest value (error) --
  
    Gyro_X_Raw   -=  Gyro_X_Offset ;                                    

  /* depende on data sheet whene we choose scale +/-250% 
    every 131 in X_gyro rigister is like 1 deg/sec
    so in 4 millescond
   
   
   */
   Gyro_X_Angle += Gyro_X_Raw * 0.000031;                            
                                                  

 // --Correct the drift of the gyro angle with the accelerometer angle  ComplementaryFilter-- 
  
  Angle = Gyro_X_Angle * 0.9996 + Acc_Angle * 0.0004;          
 

 

   //-----------------------------------------------------------------------
   //---------------------------------PID_CONTROLER ------------------------
   //-----------------------------------------------------------------------
 
  Temp_Error = Angle  - Setpoint;

  
      
 
 // --calculate integral value -- - Auto_Setpoint 
  PID_I += Ki * Temp_Error;   
   // --- limit the I valu                                           
  if (PID_I > 400)PID_I = 400;                                              
  else if (PID_I < -400)PID_I = -400;
   

  //--------Calculate the PID output value  --   

  
  PID_Value = Kp * Temp_Error + PID_I   + Kd * (Temp_Error - Last_D_Error) ;

  //  ------Limit the PID to the maximum output
  
  if (PID_Value > 400)PID_Value = 400;                                      
  else if (PID_Value < -400)PID_Value = -400;


  // -- Store the error for the next loop
  Last_D_Error = Temp_Error;                                                

  // to cut off the noise movemonte where the robot is impossible to still balance in exactly at 0* degree
  if (PID_Value < 5 && PID_Value > - 5 ) PID_Value = 0;                       

   // -----------If the robot falls we shoule stop the motor from moving and zero the pid value 
   // -----------and reset  the activated varibal to indecate the robot to new start  
  if (Angle > 30 || Angle < -30  ) {              
      PID_Value = 0;                                        //Set the PID output to 0 so the motors are stopped
      PID_I = 0;                                           //Reset the I-controller memory
      Activated = 0 ;
      Auto_Setpoint = 0 ;                              //Reset the Auto_Setpoint variable
  }
/*

 if (Setpoint == 0) {                                      //If the setpoint is zero degrees
    if (PID_Value < 0)Auto_Setpoint += 0.0015;               //Increase the Auto_Setpoint if the robot is still moving forewards
    if (PID_Value > 0)Auto_Setpoint -= 0.0015;               //Decrease the Auto_Setpoint if the robot is still moving backwards
  }

*/
     


 PID_Value_left = PID_Value;                               
 PID_Value_right = PID_Value;  
/*
//   ----------------control motion -----------------------
  if (Received_byte == 'c') {                          //We receive a a 00000001 so we turn Right
    PID_Value_left += Moving_Speed;                         //Increase the left motor speed
    PID_Value_right -= Moving_Speed;                        //Decrease the right motor speed
  }
  if (Received_byte == 'd') {                          //We receive a a 00000010 so we turn Left
    PID_Value_left -= Moving_Speed;                         //Decrease the left motor speed
    PID_Value_right += Moving_Speed;                        //Increase the right motor speed
  }

  if (Received_byte == 'a' ) {                          //We receive a a 00000100 so we go forward
    if (Setpoint > -2.5)Setpoint -= 0.05;                   //Change the setpoint angle so the robot leans forwards
    if (PID_Value > Max_Speed * -1)Setpoint -= 0.005;       //Change the setpoint angle so the robot leans forwards
  }
  if (Received_byte == 'b') {                          //We receive a a 00001000 so we go backwards
    if (Setpoint < 2.5)Setpoint += 0.05;                    //Change the setpoint angle so the robot leans backwards
    if (PID_Value < Max_Speed)Setpoint += 0.005;            //Change the setpoint angle so the robot leans backwards
  }

  if (Received_byte == 0) {                       //We receive a a 00001100 so no movement
    if (Setpoint > 0.5)Setpoint -= 0.05;                    //If the PID setpoint is higher than 0.5, reduce setpoint by 0.05 every loop
    else if (Setpoint < -0.5)Setpoint += 0.05;              //If the PID setpoint is lower than -0.5, increase setpoint by 0.05 every loop
    else Setpoint = 0;                                      //If the PID setpoint is lower than 0.5 or highert than -0.5, set the setpoint to 0
  }

 
*/
  


  /* ---------------------- compensate the pid value with speed of motors -------------
   *    -1-      the pid controller is use to control liner system   
   *    -2-      the speed of stepper is conrtlled by the time between the pulse   
   *               speed_stepper =  1 / time_between_pulses 
   *    -3-      the realtione between the TBP and speed_stepper is logarmithic (non liner )
        -4-      these formula below is to compensate the liner output pid scale
                    with  the speed of motor logarmithic scale 
   */
  
  if (PID_Value_left > 0){
    PID_Value_left = 405 - (1 / (PID_Value_left + 9)) * 5500;
  }
  else if (PID_Value_left < 0){
    PID_Value_left = -405 - (1 / (PID_Value_left - 9)) * 5500;
  }
  if (PID_Value_right > 0){
    PID_Value_right = 405 - (1 / (PID_Value_right + 9)) * 5500;
  }
  else if (PID_Value_right < 0){
    PID_Value_right = -405 - (1 / (PID_Value_right - 9)) * 5500;
  }

  //Calculate the pulse time for the left and right motor 
  if (PID_Value_left > 0){
    left_motor = 400 - PID_Value_left;
  }
  else if (PID_Value_left < 0){
    left_motor = -400 - PID_Value_left;
  }
  else left_motor = 0;

  if (PID_Value_right > 0){
    right_motor = 400 - PID_Value_right;
  }
  else if (PID_Value_right < 0){
    right_motor = -400 - PID_Value_right;
  }
  else right_motor = 0;



   Left_Motor_Speed  = -1* left_motor  ;
   Right_Motor_Speed =  right_motor ;  
      
      

         
      Serial.print (      Angle      ) ;
      Serial.print ( "   pid      "     ) ;
      Serial.print (  PID_Value  ) ;
      Serial.print (  "   step_time  "     ) ;

      step_time = millis() - temp ;
      Serial.println (step_time  ) ; 
      /*Serial.print (  "     Acc_Angle    "   ) ;
      Serial.print ( Acc_Angle   ) ;
      Serial.print (  "   Gyro_X_Angle    "   ) ;
      Serial.print (    Gyro_X_Angle     ) ;
      Serial.print (  "    angle    "   ) ;
      Serial.print (  Angle     ) ; 
      Serial.print ( "   pid      "     ) ;
      Serial.print (  PID_Value  ) ;  

   //   Serial.print ( "  error     "     ) ;
    //  Serial.print (  Temp_Error) ;  
      
    //  Serial.print (  "   left   "     ) ; 
    //  Serial.print ( left_motor ) ; 
      
    // Serial.print (  "   right  "     ) ; 
    //  Serial.print ( right_motor ) ;  
      
      Serial.print (  "   step_time  "     ) ;

      step_time = millis() - temp ;
      Serial.println (step_time  ) ; 
     */
    while (Loop_Time > micros());
  Loop_Time += 4000;
}


  /*-------------- this ISR function is called every 20us to genertate pulse to control stepper motor
   *  ------------   this counter is incrment every time it called the ISR and it determen the 
   * -------------   time between pulse where when we set new speede value :
   * ------------ -1-  reset the counter  -2-  determen the direction of speed 
   * ------------ -3-  in the next calling (counter == 1)output if step pin set high output
   * ----------- 
   * ى4ة
   * ةةةةةةةةةةةةىآ-- -4- whene  (counter == 2 )output if step pin set high low 
   * ---------------  and still low until reach the prevent value which is the the valu time between pulse
   
   */
ISR(TIMER2_COMPA_vect) {
  //Left motor pulses 
  Counter_Speed_Left_Motor ++;                                        
 
  if (Counter_Speed_Left_Motor > Left_Motor_Speed_Prev) {            
    Counter_Speed_Left_Motor   = 0 ;                             
    Left_Motor_Speed_Prev = Left_Motor_Speed;                   //Load the next Left_Motor_Speed variable
    if (Left_Motor_Speed_Prev < 0) {                            //If the Left_Motor_Speed_Prev is negative
      PORTD &= 0b11111011;                                      //Set D2 low. Reverse  direction
      Left_Motor_Speed_Prev *= -1;                              //Invert the Left_Motor_Speed_Prev variable
    }
    else PORTD |= 0b00000100;                                   //Set output D2 high. Forward direction.
  }
 
  else if (Counter_Speed_Left_Motor == 1)PORTD |= 0b00001000;        //Set output D3 high to create a pulse for the stepper
  else if (Counter_Speed_Left_Motor == 2)PORTD &= 0b11110111;        //Set output D3 low because the pulse only has to last for 20us



  //Right motor pulses
  Counter_Speed_Right_Motor ++;                                      
  if (Counter_Speed_Right_Motor > Right_Motor_Speed_Prev) {          
    Counter_Speed_Right_Motor = 0;                                 
    Right_Motor_Speed_Prev = Right_Motor_Speed;                 
    if (Right_Motor_Speed_Prev < 0) {                           //If the Right_Motor_Speed_Prev is negative
      PORTD &= 0b11101111;                                      //Set output D4 low. Reverse the direction 
      Right_Motor_Speed_Prev *= -1;                             //Invert the Right_Motor_Speed_Prev variable
    }
    else PORTD |= 0b00010000;                                   //Set D4 high. Forward direction.
  }
  else if (Counter_Speed_Right_Motor == 1)PORTD |= 0b00100000;       //Set output D5 high to create a pulse for the stepper controller
  else if (Counter_Speed_Right_Motor == 2)PORTD &= 0b11011111;       //Set output D5 low because the pulse only has to last for 20us
}// End of timer rroutine



void set_config_mpu6050()
{
  
    //Start MPU6050 communication
  Wire.beginTransmission(MPU6050_ADDR);       // the address  of mpu 6050 is 0x68
  Wire.write(0x6B);                           //Write on 0x6B register
  Wire.write(0x00);                           //Set register to 00000000 and activate gyro
  Wire.endTransmission();                     //End the i2c transmission
  
  //Change gyro scale to +/-250deg/sec
  Wire.beginTransmission(MPU6050_ADDR);       
  Wire.write(0x1B);                           //Write on 0x1B register
  Wire.write(0x00);                           //Set scale to 250dps, full scale
  Wire.endTransmission();                     //End the i2c transmission
  //Change accelerometer scale to +/-4g.
  Wire.beginTransmission(MPU6050_ADDR);       //My MPU6050 address is 0x68, change it at the begginning of the code
  Wire.write(0x1C);                           //Write on 0x1C register
  Wire.write(0x08);                           //Set scale to +/-4g
  Wire.endTransmission();                     //End the i2c transmission
  //Enable some filters
  Wire.beginTransmission(MPU6050_ADDR);       //My MPU6050 address is 0x68, change it at the begginning of the code
  Wire.write(0x1A);                           //Write on 0x1A register
  Wire.write(0x03);                           //Set Digital Low Pass Filter to ~43Hz
  Wire.endTransmission();                     //End the i2c transmission



}

  void set_timer () 
    {

        //initalizing timer to generate  pulse every 20 u sec
  
  TCCR2A = 0 ;               
  TCCR2B = 0 ;               
  TIMSK2 |= (1 << OCIE2A) ;  //  Interupt enable bit OCIE2A set to 1
  TCCR2B |= (1 << CS21)  ;   //  Set CS21 bit: We set prescaler to 8
  OCR2A = 39 ;               //  Compare register is 39, so...   20us/(1s/(16MHz/8))-1
  TCCR2A |= (1 << WGM21) ;   //  Mode: Clear timer on compare
  
     }


void gyro_offest ()
{
  
  
     // offset value. We make 1000 readdings and get that calibration value of gyro


   Serial.println ("calclaut offest ");
  
  for (int i = 0; i < 1000; i++) {                               
   
    Wire.beginTransmission(MPU6050_ADDR);                       
    Wire.write(0x43);                                          
    Wire.endTransmission();                                    
    Wire.requestFrom(MPU6050_ADDR, 2);                          
    Gyro_X_Offset += Wire.read() << 8 | Wire.read();           
   
    delayMicroseconds(4000);                                   
  }
  Gyro_X_Offset /= 1000;                                         
 
  Serial.print ("offest  x ");
  Serial.println (Gyro_X_Offset);

  delay(200);       
  }
    

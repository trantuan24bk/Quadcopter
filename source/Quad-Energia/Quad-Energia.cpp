// #include "Energia.h"
// #include "Wire.h"
// #include "quadcopter.h"
// #include "motor.h"
// #include "ReceiverRF.h"
// #include "MPU6050_6Axis_MotionApps20.h"

// MPU6050 mpu;

// /* =========================================================================
//    NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
//    depends on the MPU-6050's INT pin being connected to the Arduino's
//    external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
//    digital I/O pin 2.
//  * ========================================================================= */

// // uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// // quaternion components in a [w, x, y, z] format (not best for parsing
// // on a remote host such as Processing or something though)
// //#define OUTPUT_READABLE_QUATERNION

// // uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// // (in degrees) calculated from the quaternions coming from the FIFO.
// // Note that Euler angles suffer from gimbal lock (for more info, see
// // http://en.wikipedia.org/wiki/Gimbal_lock)
// //#define OUTPUT_READABLE_EULER

// // uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// // pitch/roll angles (in degrees) calculated from the quaternions coming
// // from the FIFO. Note this also requires gravity vector calculations.
// // Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// // more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
// #define OUTPUT_READABLE_YAWPITCHROLL

// // uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// // components with gravity removed. This acceleration reference frame is
// // not compensated for orientation, so +X is always +X according to the
// // sensor, just without the effects of gravity. If you want acceleration
// // compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
// //#define OUTPUT_READABLE_REALACCEL

// // uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// // components with gravity removed and adjusted for the world frame of
// // reference (yaw is relative to initial orientation, since no magnetometer
// // is present in this case). Could be quite handy in some cases.
// //#define OUTPUT_READABLE_WORLDACCEL

// // uncomment "OUTPUT_TEAPOT" if you want output that matches the
// // format used for the InvenSense teapot demo
// // #define OUTPUT_TEAPOT



// #define LED_RED 	8 			// LED Red
// #define LED_GREEN 	9 			// LED Green
// #define LED_YELLOW 	10 			// LED Yellow

// #define MPU_INT_PIN 42 			// PD5 - INT


// // MPU control/status vars
// bool dmpReady = false;  		// set true if DMP init was successful
// uint8_t mpuIntStatus;   		// holds actual interrupt status byte from MPU
// uint8_t devStatus;      		// return status after each device operation (0 = success, !0 = error)
// uint16_t packetSize;    		// expected DMP packet size (default is 42 bytes)
// uint16_t fifoCount;     		// count of all bytes currently in FIFO
// uint8_t fifoBuffer[64]; 		// FIFO storage buffer

// // orientation/motion vars
// Quaternion q;           		// [w, x, y, z]         quaternion container
// VectorInt16 aa;         		// [x, y, z]            accel sensor measurements
// VectorInt16 aaReal;     		// [x, y, z]            gravity-free accel sensor measurements
// VectorInt16 aaWorld;    		// [x, y, z]            world-frame accel sensor measurements
// VectorFloat gravity;    		// [x, y, z]            gravity vector
// float euler[3];         		// [psi, theta, phi]    Euler angle container
// float ypr[3];           		// [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// // packet structure for InvenSense teapot demo
// uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// // ================================================================
// // ===               INTERRUPT DETECTION ROUTINE                ===
// // ================================================================

// volatile bool flagMPUInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// void dmpDataReady(void)
// {
// 	flagMPUInterrupt = true;

//     if(digitalRead(LED_YELLOW) == LOW)
//         digitalWrite(LED_YELLOW, HIGH);
//     else
//         digitalWrite(LED_YELLOW, LOW);

// 	GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_5);
// 	GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_5);
// }

// // ================================================================
// // ===                      INITIAL SETUP                       ===
// // ================================================================

// void setup()
// {
//     init_Clock();
//     // init_ReceiverRF();
//     init_PWM();

//     Serial.begin(19200);
//     Wire.begin();
//     delay(5);

//     /* Enable global interrupt */
//     IntMasterEnable();
    
//     // Configure LED for output
//     pinMode(LED_RED, OUTPUT);
//     pinMode(LED_GREEN, OUTPUT);
//     pinMode(LED_YELLOW, OUTPUT);
    
//     digitalWrite(LED_RED, LOW);
//     digitalWrite(LED_GREEN, LOW);
//     digitalWrite(LED_YELLOW, LOW);

//     // initialize device
//     Serial.println(F("Initializing I2C devices..."));
//     mpu.initialize();

//     // verify connection
//     Serial.println(F("Testing device connections..."));
//     Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

//     // load and configure the DMP
//     Serial.println(F("Initializing DMP..."));
//     devStatus = mpu.dmpInitialize();

//     // supply your own gyro offsets here, scaled for min sensitivity
//     mpu.setXGyroOffsetTC(220);
//     mpu.setYGyroOffsetTC(76);
//     mpu.setZGyroOffsetTC(-85);
//     mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

//     // make sure it worked (returns 0 if so)
//     if (devStatus == 0)
//     {
//         // turn on the DMP, now that it's ready
//         Serial.println(F("Enabling DMP..."));
//         mpu.setDMPEnabled(true);

//         // enable Arduino interrupt detection
//         Serial.println(F("Enabling interrupt detection (PD5 <-> INT)"));
        
//         // ===================================================
//         // Case 1:
//         // attachInterrupt(MPU_INT_PIN, dmpDataReady, RISING);

//         // Case 2:
//         SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
//         GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_5);
// 		GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

// 		GPIOIntEnable(GPIO_PORTD_BASE, GPIO_INT_PIN_5);
// 		GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_INT_PIN_5, GPIO_RISING_EDGE);
// 		GPIOIntRegister(GPIO_PORTD_BASE, dmpDataReady);

// 		IntMasterEnable();
//         // ===================================================

//         mpuIntStatus = mpu.getIntStatus();

//         // set our DMP Ready flag so the main loop() function knows it's okay to use it
//         Serial.println(F("DMP ready! Waiting for first interrupt..."));
//         dmpReady = true;

//         // get expected DMP packet size for later comparison
//         packetSize = mpu.dmpGetFIFOPacketSize();
//         Serial.print(F("packetSize = "));
// 	    Serial.println(packetSize);
// 	    Serial.print(F("fifoCount = "));
// 	    Serial.println(fifoCount);
//     } 
//     else
//     {
//         // ERROR!
//         // 1 = initial memory load failed
//         // 2 = DMP configuration updates failed
//         // (if it's going to break, usually the code will be 1)
//         Serial.print(F("DMP Initialization failed (code "));
//         Serial.print(devStatus);
//         Serial.println(F(")"));
//     }
// }


// // ================================================================
// // ===                    MAIN PROGRAM LOOP                     ===
// // ================================================================

// void loop()
// {
//     // if programming failed, don't try to do anything
//     if (!dmpReady)
//     	return;

//     // // wait for MPU interrupt or extra packet(s) available
//     // while(1)
//     // {
//     //     fifoCount = mpu.getFIFOCount();
//     //     if((flagMPUInterrupt == true) || (fifoCount > packetSize))
//     //         break;
//     // }

//     while (!flagMPUInterrupt && fifoCount < packetSize)
//     {
//         fifoCount = mpu.getFIFOCount();
//     }

//     // reset interrupt flag and get INT_STATUS byte
//     flagMPUInterrupt = false;

//     mpuIntStatus = mpu.getIntStatus();

//     // check for overflow (this should never happen unless our code is too inefficient)
//     if ((mpuIntStatus & 0x10) || (fifoCount == 1024))
//     {
//         // reset so we can continue cleanly
//         mpu.resetFIFO();
//         Serial.println(F("FIFO overflow!"));

//     	// otherwise, check for DMP data ready interrupt (this should happen frequently)
//     }
//     else if (mpuIntStatus & 0x02) 
//     {
//         // wait for correct available data length, should be a VERY short wait
//         while (fifoCount < packetSize) 
//         	fifoCount = mpu.getFIFOCount();

//         // read a packet from FIFO
//         mpu.getFIFOBytes(fifoBuffer, packetSize);
        
//         // track FIFO count here in case there is > 1 packet available
//         // (this lets us immediately read more without waiting for an interrupt)
//         fifoCount -= packetSize;

//         #ifdef OUTPUT_READABLE_QUATERNION
//             // display quaternion values in easy matrix form: w x y z
//             mpu.dmpGetQuaternion(&q, fifoBuffer);
//             Serial.print("quat  \t");
//             Serial.print(q.w);
//             Serial.print("  \t");
//             Serial.print(q.x);
//             Serial.print("  \t");
//             Serial.print(q.y);
//             Serial.print("  \t");
//             Serial.println(q.z);
//         #endif

//         #ifdef OUTPUT_READABLE_EULER
//             // display Euler angles in degrees
//             mpu.dmpGetQuaternion(&q, fifoBuffer);
//             mpu.dmpGetEuler(euler, &q);
//             Serial.print("euler  \t");
//             Serial.print(euler[0] * 180/M_PI);
//             Serial.print("  \t");
//             Serial.print(euler[1] * 180/M_PI);
//             Serial.print("  \t");
//             Serial.println(euler[2] * 180/M_PI);
//         #endif

//         #ifdef OUTPUT_READABLE_YAWPITCHROLL
//             // display Euler angles in degrees
//             mpu.dmpGetQuaternion(&q, fifoBuffer);
//             mpu.dmpGetGravity(&gravity, &q);
//             mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//             Serial.print("ypr  \t");
//             Serial.print(ypr[0] * 180/M_PI);
//             Serial.print("  \t");
//             Serial.print(ypr[1] * 180/M_PI);
//             Serial.print("  \t");
//             Serial.println(ypr[2] * 180/M_PI);
//         #endif

//         #ifdef OUTPUT_READABLE_REALACCEL
//             // display real acceleration, adjusted to remove gravity
//             mpu.dmpGetQuaternion(&q, fifoBuffer);
//             mpu.dmpGetAccel(&aa, fifoBuffer);
//             mpu.dmpGetGravity(&gravity, &q);
//             mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//             Serial.print("areal  \t");
//             Serial.print(aaReal.x);
//             Serial.print("  \t");
//             Serial.print(aaReal.y);
//             Serial.print("  \t");
//             Serial.println(aaReal.z);
//         #endif

//         #ifdef OUTPUT_READABLE_WORLDACCEL
//             // display initial world-frame acceleration, adjusted to remove gravity
//             // and rotated based on known orientation from quaternion
//             mpu.dmpGetQuaternion(&q, fifoBuffer);
//             mpu.dmpGetAccel(&aa, fifoBuffer);
//             mpu.dmpGetGravity(&gravity, &q);
//             mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//             mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
//             Serial.print("aworld  \t");
//             Serial.print(aaWorld.x);
//             Serial.print("  \t");
//             Serial.print(aaWorld.y);
//             Serial.print("  \t");
//             Serial.println(aaWorld.z);
//         #endif
    
//         #ifdef OUTPUT_TEAPOT
//             // display quaternion values in InvenSense Teapot demo format:
//             teapotPacket[2] = fifoBuffer[0];
//             teapotPacket[3] = fifoBuffer[1];
//             teapotPacket[4] = fifoBuffer[4];
//             teapotPacket[5] = fifoBuffer[5];
//             teapotPacket[6] = fifoBuffer[8];
//             teapotPacket[7] = fifoBuffer[9];
//             teapotPacket[8] = fifoBuffer[12];
//             teapotPacket[9] = fifoBuffer[13];
//             Serial.write(teapotPacket, 14);
//             teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
//         #endif

//         // Blink LED to indicate activity
//         if(digitalRead(LED_GREEN) == LOW)
//             digitalWrite(LED_GREEN, HIGH);
//         else
//             digitalWrite(LED_GREEN, LOW);
//     }
// }


///////////////////////////////////////////////////////////////////////////////////////////
//
//                        KIEM TRA MPU6050
//
///////////////////////////////////////////////////////////////////////////////////////////
// #include "Energia.h"
// #include "Wire.h"
// #include "quadcopter.h"
// #include "motor.h"
// #include "ReceiverRF.h"
// #include "MPU6050_6Axis_MotionApps20.h"

// MPU6050 mpu;

// #define OUTPUT_READABLE_ACCELGYRO
// //#define OUTPUT_BINARY_ACCELGYRO


// #define LED_RED     9           // LED Red
// #define LED_GREEN   8           // LED Green
// #define LED_YELLOW  10          // LED Yellow

// #define MPU_INT_PIN 42          // PD5 - INT

// int16_t ax, ay, az;
// int16_t gx, gy, gz;

// // ================================================================
// // ===                      INITIAL SETUP                       ===
// // ================================================================

// void setup()
// {
//     init_Clock();
//     init_ReceiverRF();
//     init_PWM();

//     delay(2);
//     Serial.begin(19200);
//     Wire.begin();

//     delay(2);
//     Serial.println("Initializing I2C devices...");
//     mpu.initialize();

//     // verify connection
//     Serial.println("Testing device connections...");
//     Serial.println(mpu.testConnection() ? "Connection successful" : "Connection failed");

//     pinMode(LED_RED, OUTPUT);
//     pinMode(LED_GREEN, OUTPUT);
//     pinMode(LED_YELLOW, OUTPUT);
// }


// // ================================================================
// // ===                    MAIN PROGRAM LOOP                     ===
// // ================================================================

// void loop()
// {
//     // read raw accel/gyro measurements from device
//     mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

//     // these methods (and a few others) are also available
//     //accelgyro.getAcceleration(&ax, &ay, &az);
//     //accelgyro.getRotation(&gx, &gy, &gz);

//     #ifdef OUTPUT_READABLE_ACCELGYRO

//         Serial.print("Accel:  \t");
//         Serial.print(ax); Serial.print("  \t");
//         Serial.print(ay); Serial.print("  \t");
//         Serial.print(az); Serial.print("  \t");

//         Serial.print("Gyro:  \t");
//         Serial.print(gx); Serial.print("  \t");
//         Serial.print(gy); Serial.print("  \t");
//         Serial.println(gz);
//     #endif

//     #ifdef OUTPUT_BINARY_ACCELGYRO
//         Serial.print("Accel:  \t");
//         Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF)); Serial.print(" ");
//         Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF)); Serial.print(" ");
//         Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF)); Serial.print(" ");

//         Serial.print("  \tGyro:  \t");
//         Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF)); Serial.print(" ");
//         Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF)); Serial.print(" ");
//         Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF)); Serial.print(" ");

//         Serial.println(" ");
//     #endif

//     // Blink LED to indicate activity
//     if(digitalRead(LED_GREEN) == LOW)
//         digitalWrite(LED_GREEN, HIGH);
//     else
//         digitalWrite(LED_GREEN, LOW);
// }



///////////////////////////////////////////////////////////////////////////////////////////
//
//                        KIEM TRA RF - PWM
//
///////////////////////////////////////////////////////////////////////////////////////////
#include "Energia.h"
#include "Wire.h"
#include "quadcopter.h"
#include "motor.h"
#include "ReceiverRF.h"
#include "MPU6050_6Axis_MotionApps20.h"


#define LED_RED     	9           // LED Red
#define LED_GREEN   	8           // LED Green
#define LED_YELLOW  	10          // LED Yellow
#define MPU_INT_PIN 	42          // PD5 - INT

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
    init_Clock();
    init_ReceiverRF();
    init_PWM();

    delay(2);
    Serial.begin(19200);
    Wire.begin();
   
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);

    // Enable global interrupt
    IntMasterEnable();

    delay(40);
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{
	uint8_t duty;

    // for (int i = 1; i <= 6; ++i)
    // {
    // 	duty = ReceiverRF_Read(i);
    //     Serial.print("CH"); Serial.print(i); Serial.print("= "); Serial.print(duty); Serial.print("%   \t");
    // }
    // Serial.println(" ");

	duty = ReceiverRF_Read(3);

    motor_UpdateDuty(1, duty-10);
    motor_UpdateDuty(2, duty);
    motor_UpdateDuty(3, duty);
    motor_UpdateDuty(4, duty-6);

    delay(2);

    // Blink LED
    if(digitalRead(LED_GREEN) == LOW)
        digitalWrite(LED_GREEN, HIGH);
    else
        digitalWrite(LED_GREEN, LOW);
}

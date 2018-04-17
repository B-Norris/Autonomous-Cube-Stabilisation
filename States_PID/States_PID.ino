#include <PID_v1.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

// PID //
// Define pins used
#define PIN_OUTPUT 5
#define PIN_LED 3
#define PIN_BUTTON 2

//scaling to account for degrees & scaling of output
//& tuning controller response
double d = 0.075;

// Define variables
double Setpoint, Input, Output;

// Set tuning parameters and initialise PID with scaling
double Kp = 55 * d, Ki = 110 * d, Kd = 4.4 * d ;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// defining the sampling time
#define SAMPLE_TIME 10

// I2C Setup
// IMU //
#define LSM9DS1_1 0x6A // Would be 0x6A if SDO_AG is LOW
#define LSM9DS1_2 0x6B

// DAC //
#define MCP4725_ADDR 0x60

// Create an object from the imu class
LSM9DS1 imu1;
LSM9DS1 imu2;

// Required variables
// state variable
int state = 0;

//used as counter
int i;

//used for timing
unsigned long last, last2;

//angle variables
float ang = 0;
float ang_A, ang_G;
float avgx, avgy;
float accelx, accely;

// IMU Gyro bias values
double bias1 = 0;
double biasp1 = 0;
double bias2 = 0;
double biasp2 = 0;

//dac output voltage which will be converted to 0v for the controller
uint16_t offvoltage = 2039;

void setup() {
  //start serial monitor
  Serial.begin(115200);

  //set initial dac voltage
  setpoint(offvoltage);
  
  // PID //
  // Initialize linked variables
  Input = analogRead(ang);
  Setpoint = -1.25;  //if spinning anticlockwise at rest then increase setpoint, clockwise at rest then decrease setpoint
  myPID.SetSampleTime(SAMPLE_TIME);  //Sample time set in milliseconds
  myPID.SetOutputLimits(-10, 10);  //limit output values

  // Turn the PID on
  myPID.SetMode(AUTOMATIC);

  // IMU //
  // Setup communication
  imu1.settings.device.commInterface = IMU_MODE_I2C;
  imu1.settings.device.agAddress = LSM9DS1_1;
  imu2.settings.device.commInterface = IMU_MODE_I2C;
  imu2.settings.device.agAddress = LSM9DS1_2;

  // Test ability to connect to the IMU
  if (!imu1.begin())
  { Serial.println("Failed to communicate with LSM9DS1 1.");
    while (1);
  }
  // uncomment the second imu if you want to use it
  //if (!imu2.begin())
  //{ Serial.println("Failed to communicate with LSM9DS1 2.");
  //while (1);
  //}

  // Setup Interrupt Pin
  pinMode (PIN_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), pin_ISR, LOW);
}

void loop() {
  // Read Gyro and Accel data when available
  if ( imu1.gyroAvailable() )
  {
    imu1.readGyro();
  }
  if ( imu2.gyroAvailable() )
  {
    imu2.readGyro();
  }
  if ( imu1.accelAvailable() )
  {
    imu1.readAccel();
  }
  if ( imu2.accelAvailable() )
  {
    imu2.readAccel();
  }

  //When turned on wait for interupt to start calibration.
  if (state == 0)
  {
    Serial.println("Keep Stationary and Press Button to Start");
    setpoint(offvoltage);
    delay(500);
  }

  // Calibrate //
  if (state == 1)
  {
    Serial.println("Calibration Initialised. Wait for LED");
    // Initial angle calibration
    imu1.readAccel();
    avgx = -imu1.calcAccel(imu1.ax);
    avgy = imu1.calcAccel(imu1.ay);
    last = 0;
    i = 2;

    //average the accelerometers for 100 samples for initial angle
    //also read in 100 values of gyro to calculate bias at rest
    while (i < 102)
    {
      if ( imu1.accelAvailable() && (millis() - last) > SAMPLE_TIME)
      { imu1.readAccel();
        imu1.readGyro();
        accelx = -imu1.calcAccel(imu1.ax);
        accely = imu1.calcAccel(imu1.ay);
        avgx = (avgx * (i - 1) + accelx) / i;
        avgy = (avgy * (i - 1) + accely) / i;

        // Gyro bias calibration
        biasp1 += -1.35 * imu1.calcGyro(imu1.gz);
        biasp2 += -1.35 * imu2.calcGyro(imu2.gz);

        i++;
        last = millis();
        float test = atan(avgy / avgx);
      }
    }
    bias1 = biasp1 / 100;
    bias2 = biasp2 / 100;
    biasp1=0;
    biasp2=0;
    
    // Initial angle from calibration
    ang = atan(avgy / avgx);
    state = 2;
  }

  // Calibration complete //
  else if (state == 2)
  {
    Serial.println("Calibration complete");
    digitalWrite(PIN_LED, HIGH);
    delay(500);
  }

  // Control //
  else if (state == 3)
  {
    //run angle filtering, calculate & set pid output every sample time
    if ((millis() - last2) > SAMPLE_TIME)
    {
   
      digitalWrite(PIN_LED, LOW);
      //Angle estimation
      ang_G = ang + ((-1.35 * (imu1.calcGyro(imu1.gz)) - bias1) * 0.01);
      ang_A = 57.2957795 * atan(imu1.calcAccel(imu1.ay) / -imu1.calcAccel(imu1.ax));
      ang = (ang_G * 0.98) + (ang_A * 0.02);
//      Serial.print(ang);
//      Serial.println(" ");

      //Set input as the filtered angle and calculate output of pid
      Input = ang;
      myPID.Compute();
      Serial.print(Output);
      Serial.print(" ");
      int Voltage = map(Output, -10, 10, offvoltage-(offvoltage*2.0/3.0), offvoltage+(offvoltage*2.0/3.0));
      setpoint(Voltage);
      Serial.println(ang);
      last2 = millis();

      //Optional outputs
//      Serial.print(" ");
//      Serial.println(last2-last);
//      Serial.print(Input);
//      Serial.print("    ");
//      Serial.print(Output);
//      Serial.print("    ");
//      Serial.println(Voltage);
//      Serial.print(ang_G);
//      Serial.print("    ");
//      Serial.print(ang_A);
//      Serial.print("    ");
    }

  }

  // Stop //
  else if (state == 4)
  {
    Serial.println("System Stopped");
    digitalWrite(PIN_LED, LOW);
    // Analog write half way => stop motor
    //analogWrite(PIN_OUTPUT, 2047);
    setpoint(offvoltage);
    state = 0;
    delay(1000);
  }
}

//Interrupt Service Routine
void pin_ISR()  {
  // Debounce button
  static unsigned long last_interrupt = 0;
  unsigned long interrupt_time = millis();
  // If interrupt is faster than 500ms, assume bounce and ignore
  if (interrupt_time - last_interrupt > 500)
  {
    state ++;
  }
  last_interrupt = interrupt_time;
}

//Set the DAC output
void setpoint(uint16_t val)
{
  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write(64);                     // cmd to update the DAC
  Wire.write(val >> 4);        // the 8 most significant bits...
  Wire.write((val & 15) << 4); // the 4 least significant bits...
  Wire.endTransmission();
}

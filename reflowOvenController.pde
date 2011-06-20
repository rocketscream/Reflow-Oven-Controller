/*******************************************************************************
* Title: Reflow Oven Controller
* Version: 1.00
* Date: 20-06-2011
* Company: Rocket Scream Electronics
* Website: www.rocketscream.com
* 
* Brief
* =====
* This is an example firmware for our Arduino compatible reflow oven controller. 
* The reflow curve used in this firmware is meant for lead-free profile 
* (it's even easier for leaded process!). Please check our wiki 
* (www.rocketscream.com/wiki) for more information on using this piece of code 
* together with the reflow oven controller.
*
* Temperature (Degree Celcius)                 Magic Happens Here!
* 245-|                                               x  x  
*     |                                            x        x
*     |                                         x              x
*     |                                      x                    x
* 200-|                                   x                          x
*     |                              x    |                          |   x   
*     |                         x         |                          |       x
*     |                    x              |                          |
* 150-|               x                   |                          |
*     |             x |                   |                          |
*     |           x   |                   |                          | 
*     |         x     |                   |                          | 
*     |       x       |                   |                          | 
*     |     x         |                   |                          |
*     |   x           |                   |                          |
* 30 -| x             |                   |                          |
*     |<  60 - 90 s  >|<    90 - 120 s   >|<       90 - 120 s       >|
*     | Preheat Stage |   Soaking Stage   |       Reflow Stage       | Cool
*  0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
*                                                                Time (Seconds)
*
* This firmware owed very much on the works of other talented individuals as
* follows:
* ==========================================
* Brett Beauregard (www.brettbeauregard.com)
* ==========================================
* Author of Arduino PID library. On top of providing industry standard PID 
* implementation, he gave a lot of help in making this reflow oven controller 
* possible using his awesome library.
*
* ==========================================
* Limor Fried of Adafruit (www.adafruit.com)
* ==========================================
* Author of Arduino MAX6675 library. Adafruit has been the source of tonnes of
* tutorials, examples, and libraries for everyone to learn.
*
* Disclaimer
* ==========
* Dealing with high voltage is a very dangerous act! Please make sure you know
* what you are dealing with and have proper knowledge before hand. Your use of 
* any information or materials on this reflow oven controller is entirely at 
* your own risk, for which we shall not be liable. 
*
* Licences
* ========
* This reflow oven controller hardware and firmware are released under the 
* Creative Commons Share Alike v3.0 license
* http://creativecommons.org/licenses/by-sa/3.0/ 
* You are free to take this piece of code, use it and modify it. 
* All we ask is attribution including the supporting libraries used in this 
* firmware. 
*
* Revision  Description
* ========  ===========
* 1.00      Initial public release.
*******************************************************************************/
// ***** INCLUDES *****
#include <LiquidCrystal.h>
#include <max6675.h>
#include <PID_v1.h>

// ***** TYPE DEFINITIONS *****
typedef enum REFLOW_STATE{
    REFLOW_STATE_IDLE,
    REFLOW_STATE_PREHEAT,
    REFLOW_STATE_SOAK,
    REFLOW_STATE_REFLOW,
    REFLOW_STATE_COOL,
    REFLOW_STATE_COMPLETE,
    REFLOW_STATE_ERROR
} reflowState_t;

typedef enum REFLOW_STATUS{
    REFLOW_STATUS_OFF,
    REFLOW_STATUS_ON
} reflowStatus_t;

typedef enum DEBOUNCE_STATE{
    DEBOUNCE_STATE_IDLE,
    DEBOUNCE_STATE_CHECK,
    DEBOUNCE_STATE_RELEASE
} debounceState_t;

// ***** CONSTANTS *****
#define TEMPERATURE_ROOM 50
#define TEMPERATURE_SOAK_MIN 150
#define TEMPERATURE_SOAK_MAX 200
#define TEMPERATURE_REFLOW_MAX 250
#define TEMPERATURE_COOL_MIN 100
#define SENSOR_SAMPLING_TIME 1000
#define SOAK_TEMPERATURE_STEP 5
#define SOAK_MICRO_PERIOD 9000
#define DEBOUNCE_PERIOD_MIN 50
#define THERMOCOUPLE_DISCONNECTED 10000

// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT 300
#define PID_KI_PREHEAT 0.05
#define PID_KD_PREHEAT 400
// ***** SOAKING STAGE *****
#define PID_KP_SOAK 300
#define PID_KI_SOAK 0.05
#define PID_KD_SOAK 250
// ***** REFLOW STAGE *****
#define PID_KP_REFLOW 300
#define PID_KI_REFLOW 0.05
#define PID_KD_REFLOW 350
#define PID_SAMPLE_TIME 1000

// ***** LCD MESSAGES *****
const char* lcdMessagesReflowStatus[] = {
    "Ready",
    "Pre-heat",
    "Soak",
    "Reflow",
    "Cool",
    "Complete",
    "Error"
};

// ***** DEGREE SYMBOL FOR LCD *****
unsigned char degree[8]  = {140,146,146,140,128,128,128,128};

// ***** PIN ASSIGNMENT *****
int ssr = 5;
int thermocoupleSO = A5;
int thermocoupleCS = A4;
int thermocoupleCLK = A3;
int lcdRs = 7;
int lcdE = 8;
int lcdD4 = 9;
int lcdD5 = 10;
int lcdD6 = 11;
int lcdD7 = 12;
int ledRed = A1;
int ledGreen = A0;
int buzzer = 6;
int button1 = 2;
int button2 = 3;

// ***** PID CONTROL VARIABLES *****
double setpoint;
double input;
double output;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
int windowSize;
unsigned long windowStartTime;
unsigned long nextCheck;
unsigned long nextRead;
unsigned long timerSoak;
unsigned long buzzerPeriod;
// Reflow oven controller state machine state variable
reflowState_t reflowState;
// Reflow oven controller status
reflowStatus_t reflowStatus;
// Button debounce state machine state variable
debounceState_t debounceState;
// Button debounce timer
long lastDebounceTime;
// Button press status
boolean buttonPressStatus;
// Seconds timer
int timerSeconds;

// Specify PID control interface
PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
// Specify LCD interface
LiquidCrystal lcd(lcdRs, lcdE, lcdD4, lcdD5, lcdD6, lcdD7);
// Specify MAX6675 thermocouple interface
MAX6675 thermocouple(thermocoupleCLK, thermocoupleCS, thermocoupleSO);

void setup()
{
    // SSR pin initialization to ensure reflow oven is off
    digitalWrite(ssr, LOW);
    pinMode(ssr, OUTPUT);

    // Buzzer pin initialization to ensure annoying buzzer is off
    digitalWrite(buzzer, LOW);
    pinMode(buzzer, OUTPUT);
    
    // LED pins initialization and turn on upon start-up (active low)
    digitalWrite(ledRed, LOW);
    digitalWrite(ledGreen, LOW);
    pinMode(ledRed, OUTPUT);
    pinMode(ledGreen, OUTPUT);
	// Push button pins initialization
    pinMode(button1, INPUT);
    pinMode(button2, INPUT);
    
    // Start-up splash
    digitalWrite(buzzer, HIGH);
    lcd.begin(8, 2);
    lcd.createChar(0, degree);
    lcd.clear();
    lcd.print("Reflow");
    lcd.setCursor(0, 1);
    lcd.print("Oven V1");
    digitalWrite(buzzer, LOW);
    delay(2500);
    lcd.clear();
    
    // Serial communication at 57600 bps
    Serial.begin(57600);
    
    // Turn off LED (active low)
    digitalWrite(ledRed, HIGH);
    digitalWrite(ledGreen, HIGH);
    
    // Set window size
    windowSize = 2000;
    // Initialize time keeping variable
    nextCheck = millis();
    // Initialize thermocouple reading varible
    nextRead = millis();
}

void loop()
{
    // Current time
    unsigned long now;
    
    // Time to read thermocouple?
    if (millis() > nextRead)
    {
        // Read thermocouple next sampling period
        nextRead += SENSOR_SAMPLING_TIME;
        // Read current temperature
        input = thermocouple.readCelsius();
        
        // If thermocouple is not connected
        if (input == THERMOCOUPLE_DISCONNECTED)
        {
            // Illegal operation without thermocouple
            reflowState = REFLOW_STATE_ERROR;
            reflowStatus = REFLOW_STATUS_OFF;
        }
    }
    
    if (millis() > nextCheck)
    {
        // Check input in the next seconds
        nextCheck += 1000;
        // If reflow process is on going
        if (reflowStatus == REFLOW_STATUS_ON)
        {
            // Toggle red LED as system heart beat
            digitalWrite(ledRed, !(digitalRead(ledRed)));
            // Increase seconds timer for reflow curve analysis
            timerSeconds++;
            // Send temperature and time stamp to serial 
            Serial.print(timerSeconds);
            Serial.print(" ");
            Serial.print(setpoint);
            Serial.print(" ");
            Serial.print(input);
            Serial.print(" ");
            Serial.println(output);
        }
        else
        {
            // Turn off red LED
            digitalWrite(ledRed, HIGH);
        }
        
         // Clear LCD
        lcd.clear();
        // Print current system state
        lcd.print(lcdMessagesReflowStatus[reflowState]);
        // Move the cursor to the 2 line
        lcd.setCursor(0, 1);
		
        // If currently in error state
        if (reflowState == REFLOW_STATE_ERROR)
        {
            // No thermocouple wire connected
            lcd.print("No TC!");
        }
        else
        {
            // Print current temperature
            lcd.print(input);
            // Print degree Celsius symbol
            lcd.print(0, BYTE);
            lcd.print("C ");
        }
    }
    
    // Reflow oven controller state machine
    switch (reflowState)
    {
        case REFLOW_STATE_IDLE:
            // If button is pressed to start reflow process
            if (buttonPressStatus)
            {
                // Ensure current temperature is comparable to room temperature
				// TO DO: To add indication that temperature is still high for
				// reflow process to start
                if (input <= TEMPERATURE_ROOM)
                {
                    // Send header for CSV file
                    Serial.println("Time Setpoint Input Output");
                    // Intialize seconds timer for serial debug information
                    timerSeconds = 0;
                    // Initialize PID control window starting time
                    windowStartTime = millis();
                    // Ramp up to minimum soaking temperature
                    setpoint = TEMPERATURE_SOAK_MIN;
                    // Tell the PID to range between 0 and the full window size
                    reflowOvenPID.SetOutputLimits(0, windowSize);
                    reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
                    // Turn the PID on
                    reflowOvenPID.SetMode(AUTOMATIC);
                    // Proceed to preheat stage
                    reflowState = REFLOW_STATE_PREHEAT;
                }
            }
            break;

        case REFLOW_STATE_PREHEAT:
            reflowStatus = REFLOW_STATUS_ON;
            // If minimum soak temperature is achieve       
            if (input >= TEMPERATURE_SOAK_MIN)
            {
                // Chop soaking period into smaller sub-period
                timerSoak = millis() + SOAK_MICRO_PERIOD;
                // Set less agressive PID parameters for soaking ramp
                reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
                // Ramp up to first section of soaking temperature
                setpoint = TEMPERATURE_SOAK_MIN + SOAK_TEMPERATURE_STEP;   
                // Proceed to soaking state
                reflowState = REFLOW_STATE_SOAK; 
            }
            break;
        
        case REFLOW_STATE_SOAK:     
            // If micro soak temperature is achieved       
            if (millis() > timerSoak)
            {
                timerSoak = millis() + SOAK_MICRO_PERIOD;
                // Increment micro setpoint
                setpoint += SOAK_TEMPERATURE_STEP;
                if (setpoint > TEMPERATURE_SOAK_MAX)
                {
                    // Set agressive PID parameters for reflow ramp
                    reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
                    // Ramp up to first section of soaking temperature
                    setpoint = TEMPERATURE_REFLOW_MAX;   
                    // Proceed to reflowing state
                    reflowState = REFLOW_STATE_REFLOW; 
                }
            }
            break; 
         
        case REFLOW_STATE_REFLOW:
            // We need to avoid hovering at peak temperature for too long
            // Crude method that works like a charm and safe for the components
            if (input >= (TEMPERATURE_REFLOW_MAX - 5))
            {
                // Set PID parameters for cooling ramp
                reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
                // Ramp down to minimum cooling temperature
                setpoint = TEMPERATURE_COOL_MIN;   
                // Proceed to cooling state
                reflowState = REFLOW_STATE_COOL; 
            }
            break;   
            
        case REFLOW_STATE_COOL:
            // If minimum cool temperature is achieve       
            if (input <= TEMPERATURE_COOL_MIN)
            {
                // Retrieve current time for buzzer usage
                buzzerPeriod = millis() + 1000;
                // Turn on buzzer and green LED to indicate completion
                digitalWrite(ledGreen, LOW);
                digitalWrite(buzzer, HIGH);
                // Turn off reflow process
                reflowStatus = REFLOW_STATUS_OFF;                
                // Proceed to reflow Completion state
                reflowState = REFLOW_STATE_COMPLETE; 
            }         
            break;    
            
        case REFLOW_STATE_COMPLETE:
            if (millis() > buzzerPeriod)
            {
                // Turn off buzzer and green LED
                digitalWrite(buzzer, LOW);
                digitalWrite(ledGreen, HIGH);
                // Reflow process ended
                reflowState = REFLOW_STATE_IDLE; 
            }
            break;
        
        case REFLOW_STATE_ERROR:
            // If thermocouple is still not connected
            if (input == THERMOCOUPLE_DISCONNECTED)
            {
                // Wait until thermocouple wire is connected
                reflowState = REFLOW_STATE_ERROR; 
            }
            else
            {
                // Clear to perform reflow process
                reflowState = REFLOW_STATE_IDLE; 
            }
            break;    
    }    
    
    // If button is pressed
    if (buttonPressStatus == true)
    {
        // If currently reflow process is on going
        if (reflowStatus == REFLOW_STATUS_ON)
        {
            // Button press is for cancelling
            // Turn off reflow process
            reflowStatus = REFLOW_STATUS_OFF;
            // Reinitialize state machine
            reflowState = REFLOW_STATE_IDLE;
        }
    } 
    
    // Simple button debounce state machine (for button #1 only)
	// TO DO: To be replaced with interrupt version in next revision
    switch (debounceState)
    {
        case DEBOUNCE_STATE_IDLE:
            // No valid button press
            buttonPressStatus = false;
            // If button #1 is pressed
            if (digitalRead(button1) == LOW)
            {
                // Intialize debounce counter
                lastDebounceTime = millis();
                // Proceed to check validity of button press
                debounceState = DEBOUNCE_STATE_CHECK;
            }
            break;
            
        case DEBOUNCE_STATE_CHECK:
            // If button #1 is still pressed
            if (digitalRead(button1) == LOW)
            {
                // If minimum debounce period is completed
                if ((millis() - lastDebounceTime) > DEBOUNCE_PERIOD_MIN)
                {
                    // Proceed to wait for button release
                    debounceState = DEBOUNCE_STATE_RELEASE;
                }
            }
            // False trigger
            else
            {
                // Reinitialize button debounce state machine
                debounceState = DEBOUNCE_STATE_IDLE; 
            }
            break;
            
        case DEBOUNCE_STATE_RELEASE:
            if (digitalRead(button1) == HIGH)
            {
                // Valid button press
                buttonPressStatus = true;
                // Reinitialize button debounce state machine
                debounceState = DEBOUNCE_STATE_IDLE; 
            }
            break;
    }
    
	// PID computation and SSR control
    if (reflowStatus == REFLOW_STATUS_ON)
    {
        //unsigned long now;
        now = millis();
        
        reflowOvenPID.Compute();

        if((now - windowStartTime) > windowSize)
        { 
            // Time to shift the Relay Window
            windowStartTime += windowSize;
        }
        if(output > (now - windowStartTime)) digitalWrite(ssr, HIGH);
        else digitalWrite(ssr, LOW);   
    }
    // Reflow oven process is off, ensure oven is off
    else 
    {
        digitalWrite(ssr, LOW);
    }
}

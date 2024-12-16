#include <wiringPi.h>
#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

// Encoder pins for main rotor
#define ENCODER_PIN_A_MAIN 3
#define ENCODER_PIN_B_MAIN 5

// Encoder pins for tail rotor
#define ENCODER_PIN_A_TAIL 2
#define ENCODER_PIN_B_TAIL 6

// ESC pins
#define ESC_PIN_MAIN 9
#define ESC_PIN_TAIL 10

// PID parameters for main rotor
float kp_Main = 5.0, ki_Main = 0.5, kd_Main = 0.5;
float integral_Main = 0.0, previousError_Main = 0.0;

// PID parameters for tail rotor
float kp_Tail = 0.1, ki_Tail = 1.0, kd_Tail = 0.4;
float integral_Tail = 0.0, previousError_Tail = 0.0;

// Sliding Mode Control (SMC) parameters
float lambda = 10.0, eta = 8.0, epsilon = 0.4;

// Target states
float targetTheta_Main = 45.0, targetThetaDot_Main = 0.0;
float targetTheta_Tail = 40.0, targetThetaDot_Tail = 0.0;

// Maximum allowable angle
const float maxAngle = 150.0;

// Encoder variables
volatile int encoderCountsMain = 0, encoderCountsTail = 0;

// State variables for main rotor
float theta_Main = 0.0, thetaDot_Main = 0.0, previousTheta_Main = 0.0;

// State variables for tail rotor
float theta_Tail = 0.0, thetaDot_Tail = 0.0, previousTheta_Tail = 0.0;

// ESC parameters
int maxThrottle = 2000, minThrottle = 1000;
int throttleMain = 1200, throttleTail = 1200;

// Function prototypes
void updateEncoderMain();
void updateEncoderTail();
void updateRotorControl(
    volatile int *encoderCounts, float *theta, float *thetaDot, float *previousTheta,
    float targetTheta, float targetThetaDot,
    float *integral, float *previousError,
    int *throttle, int escPin);
float saturation(float s);
void setupESC(int escPin);

int main() {
    // Initialize wiringPi and pigpio
    if (wiringPiSetupGpio() == -1) {
        printf("Failed to initialize wiringPi.\n");
        return 1;
    }

    if (gpioInitialise() < 0) {
        printf("Failed to initialize pigpio.\n");
        return 1;
    }

    // Set up encoder pins
    pinMode(ENCODER_PIN_A_MAIN, INPUT);
    pinMode(ENCODER_PIN_B_MAIN, INPUT);
    pinMode(ENCODER_PIN_A_TAIL, INPUT);
    pinMode(ENCODER_PIN_B_TAIL, INPUT);

    wiringPiISR(ENCODER_PIN_A_MAIN, INT_EDGE_BOTH, updateEncoderMain);
    wiringPiISR(ENCODER_PIN_A_TAIL, INT_EDGE_BOTH, updateEncoderTail);

    // Set up ESC pins
    setupESC(ESC_PIN_MAIN);
    setupESC(ESC_PIN_TAIL);

    printf("Starting main loop...\n");
    while (1) {
        // Main rotor control
        updateRotorControl(
            &encoderCountsMain, &theta_Main, &thetaDot_Main, &previousTheta_Main,
            targetTheta_Main, targetThetaDot_Main,
            &integral_Main, &previousError_Main,
            &throttleMain, ESC_PIN_MAIN
        );

        // Tail rotor control
        updateRotorControl(
            &encoderCountsTail, &theta_Tail, &thetaDot_Tail, &previousTheta_Tail,
            targetTheta_Tail, targetThetaDot_Tail,
            &integral_Tail, &previousError_Tail,
            &throttleTail, ESC_PIN_TAIL
        );

        usleep(10000); // 10 ms delay
    }

    // Cleanup
    gpioTerminate();
    return 0;
}

void setupESC(int escPin) {
    gpioSetPWMfrequency(escPin, 50); // Set PWM frequency to 50 Hz
    gpioSetPWMrange(escPin, 20000);  // Set PWM range to match throttle values
    gpioPWM(escPin, minThrottle);   // Initialize ESC to minimum throttle
    sleep(2); // Wait for ESC to initialize
}

void updateEncoderMain() {
    int stateA = digitalRead(ENCODER_PIN_A_MAIN);
    int stateB = digitalRead(ENCODER_PIN_B_MAIN);
    static int lastStateA = 0;

    if (stateA != lastStateA) {
        if (stateA == stateB) {
            encoderCountsMain--;
        } else {
            encoderCountsMain++;
        }
    }
    lastStateA = stateA;
}

void updateEncoderTail() {
    int stateA = digitalRead(ENCODER_PIN_A_TAIL);
    int stateB = digitalRead(ENCODER_PIN_B_TAIL);
    static int lastStateA = 0;

    if (stateA != lastStateA) {
        if (stateA == stateB) {
            encoderCountsTail++;
        } else {
            encoderCountsTail--;
        }
    }
    lastStateA = stateA;
}

void updateRotorControl(
    volatile int *encoderCounts, float *theta, float *thetaDot, float *previousTheta,
    float targetTheta, float targetThetaDot,
    float *integral, float *previousError,
    int *throttle, int escPin) {

    // Limit the target angle
    targetTheta = fmax(0, fmin(targetTheta, maxAngle));

    // Calculate current angle
    *theta = (*encoderCounts % 1200) * (360.0 / 1200);
    if (*theta >= 250 && *theta <= 359.9) *theta = 0;

    // Calculate angular velocity
    *thetaDot = (*theta - *previousTheta) / 0.01;
    *previousTheta = *theta;

    // PID calculation
    float errorTheta = targetTheta - *theta;
    *integral += errorTheta;
    *integral = fmax(-550, fmin(*integral, 550));
    float derivativeTheta = (errorTheta - *previousError) / 0.01;
    *previousError = errorTheta;
    float pidOutput = (kp_Main * errorTheta) + (ki_Main * (*integral)) + (kd_Main * derivativeTheta);

    // Sliding Mode Control (SMC)
    float slidingSurface = lambda * (*theta - targetTheta) + (*thetaDot - targetThetaDot);
    float smcOutput = -eta * saturation(slidingSurface);

    // Combine PID and SMC outputs
    float controlOutput = pidOutput + smcOutput;

    // Map control output to throttle
    *throttle = minThrottle + controlOutput;
    *throttle = fmax(1080, fmin(*throttle, 1280));

    // Send throttle to ESC
    gpioPWM(escPin, *throttle);

    // Print debug information
    printf("Theta: %.2f | ThetaDot: %.2f | TargetTheta: %.2f | Throttle: %d\n",
           *theta, *thetaDot, targetTheta, *throttle);
}

float saturation(float s) {
    if (fabs(s) <= epsilon) {
        return s / epsilon;
    } else {
        return (s > 0) ? 1.0 : -1.0;
    }
}

/* 
 * File:   ahrs_dcm_ml_main.c
 * Author: Aaron Hunter
 * Brief: 
 * Created on July 7, 2022
 * Modified on <month> <day>, <year>, <hour> <pm/am>
 */

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/
#include <stdio.h>
#include <math.h>
#include <pigpio.h> // i2c library
#include <time.h> // has nanosleep() function and clock_gettime()
#include <unistd.h> // has usleep() function
#include "../IMU/BNO055.h" // The header file for this source file. 

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/
#define SUCCESS 0
#define ERROR -1

#define TRUE 1
#define FALSE 0

#define MSZ 3
#define QSZ 4

#define DT 0.02 

#define I2C_MODE 1  // use I2C-1 pins
/** BNO055 Address A: when ADR (COM3) pin is tied to ground (default) **/
#define BNO055_ADDRESS_A (0x28)

/*******************************************************************************
 * Variables                                                                   *
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES                                                         *
 ******************************************************************************/
/**
 * @function m_v_mult()
 * Multiplies a matrix with a vector
 * @param m A matrix to be multiplied with a vector
 * @param v A vector to be multiplied with a matrix
 * @param v_out The product of the matrix and vector
 * @return SUCCESS or ERROR
 */
void m_v_mult(double m[MSZ][MSZ], double v[MSZ], double v_out[MSZ]);
/**
 * @function lin_alg_v_v_add()
 * Add a vector value to a vector
 * @param v1 Vector to add to another vector
 * @param v2 Vector to have a vector added to it
 * @param v_out Vector as sum of two vectors
 */
void v_v_add(double v1[MSZ], double v2[MSZ], double v_out[MSZ]);
/**
 * @function extract_angles();
 * Extract Euler angles from the DCM
 * @param dcm The Direction Cosine Matrix, a rotation matrix
 * @param psi A pointer to return the Yaw angle in radians from -pi to pi
 * @param theta A pointer to return the Pitch angle in radians from -pi to pi
 * @param phi A pointer to return the Roll angle in radians from -pi to pi
 * @return SUCCESS or FAIL
 */
void extract_angles(double dcm[MSZ][MSZ], double euler[MSZ]);

/*******************************************************************************
 * FUNCTIONS                                                                   *
 ******************************************************************************/

/**
 * @function m_v_mult()
 * Multiplies a matrix with a vector
 * @param m A matrix to be multiplied with a vector
 * @param v A vector to be multiplied with a matrix
 * @param v_out The product of the matrix and vector
 * @return SUCCESS or ERROR
 */
void m_v_mult(double m[MSZ][MSZ], double v[MSZ], double v_out[MSZ]) {
    int row;
    int col;

    for (row = 0; row < MSZ; row++) {
        v_out[row] = 0;
        for (col = 0; col < MSZ; col++) {
            v_out[row] += m[row][col] * v[col];
        }
    }
}

/**
 * @function lin_alg_v_v_add()
 * Add a vector value to a vector
 * @param v1 Vector to add to another vector
 * @param v2 Vector to have a vector added to it
 * @param v_out Vector as sum of two vectors
 */
void v_v_add(double v1[MSZ], double v2[MSZ], double v_out[MSZ]) {
    int row;

    for (row = 0; row < MSZ; row++) {
        v_out[row] = v1[row] + v2[row];
    }
}

/**
 * @function extract_angles();
 * Extract Euler angles from the DCM
 * @param dcm The Direction Cosine Matrix, a rotation matrix
 * @param psi A pointer to return the Yaw angle in radians from -pi to pi
 * @param theta A pointer to return the Pitch angle in radians from -pi to pi
 * @param phi A pointer to return the Roll angle in radians from -pi to pi
 * @return SUCCESS or FAIL
 */
void extract_angles(double dcm[MSZ][MSZ], double euler[MSZ]) {
    const double pi_2 = M_PI / 2;
    euler[0] = atan2(dcm[1][0], dcm[0][0]); /* Yaw */
    if (dcm[2][0] > 1.0) {
        euler[1] = -pi_2;
    } else if (dcm[2][0] < -1.0) {
        euler[1] = pi_2;
    } else {
        euler[1] = -asin(dcm[2][0]); /* Pitch */
    }

    euler[2] = atan2(dcm[2][1], dcm[2][2]); /* Roll */

}

int main(void) {
    int i2c_handle;
    char init_result;
    uint8_t index = 0;
    int row;
    int col;
 
    struct timespec current_time;
    struct timespec start_time; 
    struct timespec update_start; // start time of update step
    struct timespec update_end;  // ending time of update step 
    long int timeout_start_time; 
    long int elapsed_time = 0; //period timer
    long int update_time = 0; //update timer
    long int sec2nsec = 1000000000;
    long int period = 20000000; // period in nsec
    long int timeout = 30 + 1; // timeout in sec
    char timer_active = 1;

    /*Filter gains */
    double kp_a = 2.5; //Accelerometer proportional gain 
    double ki_a = 0.05; // Accelerometer integral gain
    double kp_m = 2.5; //Magnetometer proportional gain 
    double ki_m = 0.05; //Magnetometer integral gain

    /* Matrices and arrays */
    const double dt = DT; // integration interval
    const double gyro_scale = 250.0 / (double) ((1 << 15) - 1);
    const double deg2rad = M_PI / 180.0;
    const double rad2deg = 180.0 / M_PI;
    /*Calibration matrices and offsets from tumble test */
    double A_acc[MSZ][MSZ] = {
        {0.00100495454726146, -0.0000216880122750632, 0.0000133999325710038},
        {-0.0000162926752704191, 0.000985380021967908, 0.00000666633241783684},
        {0.0000125438947528585, 0.00000252521314262081, 0.000989136500881465}
    };
    double b_acc[MSZ] = {0.00605446075713124, 0.0531371565285396, 0.0243733439531166};

    double A_mag[MSZ][MSZ] = {
        {0.00135496706593374, 0.0000116105133187656, -0.00000832065758415854},
        {0.00000781613708114319, 0.00137913779319635, 0.00000538838213067807},
        {-0.0000126033738192070, 0.00000739989830409341, 0.00140611428756101}
    };

    double b_mag[MSZ] = {0.480254990338941, -0.286077906618463, 0.528755499855987};

    /* gravity inertial vector */
    double a_i[MSZ] = {0, 0, 1.0};
    // Earth's magnetic field inertial vector, normalized 
    // North 22,680.8 nT	East 5,217.6 nT	Down 41,324.7 nT, value from NOAA
    // converted into ENU format and normalized:
    double m_i[MSZ] = {0.110011998753301, 0.478219898291142, -0.871322609031072};

    /* Euler angles */
    double euler[MSZ] = {0, 0, 0};

    /* attitude DCMs */
    double r_minus[MSZ][MSZ] = {{1, 0, 0},
                                {0, 1, 0},
                                {0, 0, 1}
                                 };
    double r_plus[MSZ][MSZ] = {{1, 0, 0},
                                {0, 1, 0},
                                {0, 0, 1}
                                 };

    /* gyro bias vectors */
    double b_minus[MSZ] = {0, 0, 0};
    double b_plus[MSZ] = {0, 0, 0};

    /* Raw Data arrays */
    int16_t acc_raw[MSZ];
    int16_t mag_raw[MSZ];
    int16_t gyro_raw[MSZ];
    double acc_raw_double[MSZ];
    double mag_raw_double[MSZ];
    int8_t temperature = 0;
    /* Calibrated data arrays */
    double gyro_cal[MSZ];
    double acc_tmp[MSZ]; // intermediate array
    double acc_cal[MSZ];
    double mag_cal[MSZ];
    double mag_tmp[MSZ]; // intermediate array

    /*init */
    printf("AHRS quaternion double evaluation application, compiled " __DATE__ " " __TIME__ "\r\n");
    printf("Sensor data will be streamed after initialization.\r\n");
        /*Initialize the GPIO subsystem*/
    if(gpioInitialise() <0 ){
        printf("GPIO failed init!\r\n");
        return(0);
    } else {
        printf("GPIO initialized.\r\n");
    }
    /*Initialize 12c system*/
    i2c_handle = i2cOpen(I2C_MODE, BNO055_ADDRESS_A,0);
    if(i2c_handle <0 ) {
        printf("Failed to open i2c bus, error code %d.\r\n", i2c_handle);
    } else {
        printf("i2c bus opened with handle %d.\r\n", i2c_handle);
    }
    /*Initialize the BNO055 IMU*/
    init_result = BNO055_Init(i2c_handle);
    /* get start time*/
    clock_gettime(CLOCK_REALTIME, &start_time);
    timeout_start_time = start_time.tv_sec;
    while(timer_active){
        /* get current time*/
        clock_gettime(CLOCK_REALTIME, &current_time);
            /*if elapsed time >= measurement period, start new measurement*/
        elapsed_time = (current_time.tv_sec - start_time.tv_sec)*sec2nsec +
            (current_time.tv_nsec - start_time.tv_nsec);
        if(elapsed_time >= period){
            /*reset start time*/
            clock_gettime(CLOCK_REALTIME, &start_time); // reset period counter
            /* read sensors*/
            BNO055_ReadAccel(i2c_handle, acc_raw);
            BNO055_ReadMag(i2c_handle, mag_raw);
            BNO055_ReadGyro(i2c_handle, gyro_raw);
            /* convert uint16_t to double for compatibility with linear algebra library */
            for(index = 0; index < MSZ; index++){
                acc_raw_double[index] = (double)acc_raw[index];
                mag_raw_double[index] = (double)mag_raw[index];
                gyro_cal[index] = (double) gyro_raw[index] * gyro_scale * deg2rad; // scale and convert gyro data into rad/s
            }
            /* calibrate sensors */
            m_v_mult(A_acc, acc_raw_double, acc_tmp); // scale and calibrate acc             
            v_v_add(acc_tmp, b_acc, acc_cal); // offset accelerometer data              
            m_v_mult(A_mag, mag_raw_double, mag_tmp); // scale magnetometer data                 
            v_v_add(mag_tmp, b_mag, mag_cal); // offset magnetometer data

            clock_gettime(CLOCK_REALTIME,&update_start);
            ahrs_m_update(r_minus, b_minus, gyro_cal, mag_cal, acc_cal, m_i, a_i, dt,
                    kp_a, ki_a, kp_m, ki_m, r_plus, b_plus);
            clock_gettime(CLOCK_REALTIME, &update_end);
            extract_angles(r_plus,euler);
            /* update b_minus and q_minus */
            for (row = 0; row < MSZ; row++) {
                for (col = 0; col < MSZ; col++) {
                    r_minus[row][col] = r_plus[row][col];
                    b_minus[col] = b_plus[col];
                }
            }
            /* print out data */
            update_time =  (update_end.tv_sec - update_start.tv_sec)*sec2nsec +
                (update_end.tv_nsec - update_start.tv_nsec);
            printf("%+3.1f, %3.1f, %3.1f, ", euler[0] * rad2deg, euler[1] * rad2deg, euler[2] * rad2deg);
            printf("%1.3e, %1.3e, %1.3e, ", b_plus[0], b_plus[1], b_plus[2]);
            printf("%ld\r\n", update_time);
        } /*end if*/
        /* check for timeout*/
        if(current_time.tv_sec - timeout_start_time >= timeout) timer_active = 0;
    } /*end while*/
    i2cClose(i2c_handle);
    gpioTerminate();
    return(0);
} /*end main*/



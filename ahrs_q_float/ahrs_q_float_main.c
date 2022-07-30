/* 
 * File:   ahrs_q_float_main.c
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


void cross(float u[MSZ], float v[MSZ], float w_out[MSZ]);
void q_mult(float q[QSZ], float p[QSZ], float r[QSZ]);
void quat2euler(float q[QSZ], float euler[MSZ]) ;
void q_rot_v_q(float v_i[MSZ], float q[QSZ], float v_b[MSZ]);
float q_norm(float q[QSZ]);
void v_scale(float s, float v[MSZ]);
void v_v_add(float v1[MSZ], float v2[MSZ], float v_out[MSZ]);
void v_v_sub(float v1[MSZ], float v2[MSZ], float v_out[MSZ]);
void v_copy(float m_in[MSZ], float m_out[MSZ]);
void m_v_mult(float m[MSZ][MSZ], float v[MSZ], float v_out[MSZ]);
void ahrs_update(float q_minus[QSZ], float q_plus[QSZ], float bias_minus[MSZ],
        float bias_plus[MSZ], float gyros[MSZ], float mags[MSZ], float accels[MSZ],
        float mag_i[MSZ], float acc_i[MSZ], float dt);

/*******************************************************************************
 * FUNCTIONS                                                                   *
 ******************************************************************************/
/**
 * @function cross()
 * @param u A vector
 * @param v A vector
 * @return The cross product (sometimes called the outter product) of u and v
 */
void cross(float u[MSZ], float v[MSZ], float w_out[MSZ]) {
    w_out[0] = u[1] * v[2] - u[2] * v[1];
    w_out[1] = u[2] * v[0] - u[0] * v[2];
    w_out[2] = u[0] * v[1] - u[1] * v[0];
}

/**
 * @function q_mult()
 * Multiply two quaternions together
 * @param p A quaternion
 * @param q A quaternion
 * @param r The resulting quaternion
 */
void q_mult(float q[QSZ], float p[QSZ], float r[QSZ]) {
    r[0] = p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3];
    r[1] = p[1] * q[0] + p[0] * q[1] + p[3] * q[2] - p[2] * q[3];
    r[2] = p[2] * q[0] - p[3] * q[1] + p[0] * q[2] + p[1] * q[3];
    r[3] = p[3] * q[0] + p[2] * q[1] - p[1] * q[2] + p[0] * q[3];
}
/**
 * @function quat2euler()
 * @param q A quaternion
 * @param euler a vector of euler angles in [psi, theta, roll] order
 */
void quat2euler(float q[QSZ], float euler[MSZ]) {
    float q00 = q[0] * q[0];
    float q11 = q[1] * q[1];
    float q22 = q[2] * q[2];
    float q33 = q[3] * q[3];

    // psi
    euler[0] = atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), ((q00 + q11 - q22 - q33)));
    // theta
    euler[1] = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
    // phi
    euler[2] = atan2(2.0 * (q[2] * q[3] + q[0] * q[1]), q00 - q11 - q22 + q33);
}

/**
 * @function q_rot_v_q()
 * Rotate a vector from the inertial frame to the body frame
 * @param v_i, a 3space vector in the inertial frame
 * @param q an attitude quaternion
 * sets v_b to the rotated inertial vector in the body frame
 */
void q_rot_v_q(float v_i[MSZ], float q[QSZ], float v_b[MSZ]) {
    float q_i[QSZ];
    float q_temp[QSZ];
    float q_conj[QSZ];
    float q_b[QSZ]; // container for inertial vector in body frame as pure quaternion

    /* calculate conjugate of q */
    q_conj[0] = q[0];
    q_conj[1] = -q[1];
    q_conj[2] = -q[2];
    q_conj[3] = -q[3];
    /* convert v_i to a pure quaternion --> q_i */
    q_i[0] = 0;
    q_i[1] = v_i[0];
    q_i[2] = v_i[1];
    q_i[3] = v_i[2];
    /* first quaternion product q_i by q --> q_temp */
    q_mult(q_i, q, q_temp);
    /* second quaternion product q_conj by q_temp -->q_b */
    q_mult(q_conj, q_temp, q_b);
    /* set v_b to imaginary part of q_b */
    v_b[0] = q_b[1];
    v_b[1] = q_b[2];
    v_b[2] = q_b[3];
}

/**
 * @function q_norm()
 * @param q A quaternion
 * @return The magnitude of the quaternion, q
 */
float q_norm(float q[QSZ]) {
    return ((float) sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]));
}

/**
 * @function v_scale()
 * Scales vector
 * @param s Scalar to scale vector with
 * @param v Vector to be scaled
 */
void v_scale(float s, float v[MSZ]) {
    int row;

    for (row = 0; row < MSZ; row++) {
        v[row] *= s;
    }
}

/**
 * @function v_v_add()
 * Add a vector value to a vector
 * @param v1 Vector to add to another vector
 * @param v2 Vector to have a vector added to it
 * @param v_out Vector as sum of two vectors
 */
void v_v_add(float v1[MSZ], float v2[MSZ], float v_out[MSZ]) {
    int row;

    for (row = 0; row < MSZ; row++) {
        v_out[row] = v1[row] + v2[row];
    }
}
/**
 * @function v_v_sub()
 * Add a vector value to a vector
 * @param v1 Vector to subtract to another vector
 * @param v2 Vector to have a vector subtracted to it
 * @param v_out Vector as difference of two vectors
 */
void v_v_sub(float v1[MSZ], float v2[MSZ], float v_out[MSZ]) {
    int row;

    for (row = 0; row < MSZ; row++) {
        v_out[row] = v1[row] - v2[row];
    }
}

void v_copy(float m_in[MSZ], float m_out[MSZ]) {
    int row;
    for (row = 0; row < MSZ; row++) {
        m_out[row] = m_in[row];
    }
}

/**
 * @function m_v_mult()
 * Multiplies a matrix with a vector
 * @param m A matrix to be multiplied with a vector
 * @param v A vector to be multiplied with a matrix
 * @param v_out The product of the matrix and vector
 * @return SUCCESS or ERROR
 */
void m_v_mult(float m[MSZ][MSZ], float v[MSZ], float v_out[MSZ]) {
    int row;
    int col;

    for (row = 0; row < MSZ; row++) {
        v_out[row] = 0;
        for (col = 0; col < MSZ; col++) {
            v_out[row] += m[row][col] * v[col];
        }
    }
}

void ahrs_update(float q_minus[QSZ], float q_plus[QSZ], float bias_minus[MSZ],
    float bias_plus[MSZ], float gyros[MSZ], float mags[MSZ], float accels[MSZ],
    float mag_i[MSZ], float acc_i[MSZ], float dt) {
    float kp_a = 2.5; //accelerometer proportional gain
    float ki_a = 0.05; // accelerometer integral gain
    float kp_m = 2.5; // magnetometer proportional gain
    float ki_m = 0.05; //magnetometer integral gain

    float acc_b[MSZ]; //estimated gravity vector in body frame
    float mag_b[MSZ]; //estimated magnetic field vector in body frame

    float gyro_cal[MSZ]; // gyros with bias correction
    float gyro_wfb[MSZ]; // gyro 'rate' after feedback
    float w_meas_ap[MSZ]; // accelerometer proportion correction rate
    float w_meas_mp[MSZ]; // magnetometer proportional correction rate
    float w_meas_ai[MSZ]; // accelerometer integral correction rate
    float w_meas_mi[MSZ]; // magnetometer integral correction rate

    float gyro_q_wfb[QSZ]; // temporary quaternion to hold feedback term
    float q_dot[QSZ]; // quaternion derivative
    float b_dot[MSZ]; // bias vector derivative
    float q_n;


    /*Accelerometer attitude calculations */
    q_rot_v_q(acc_i, q_minus, acc_b); //estimate gravity vector in body frame 
    cross(accels, acc_b, w_meas_ap); // calculate the accelerometer rate term
    v_copy(w_meas_ap, w_meas_ai); // make a copy for the integral term
    v_scale(kp_a, w_meas_ap); // calculate the accelerometer proportional feedback term 
    v_scale(ki_a, w_meas_ai); // calculate the accelerometer integral feedback term 

    /*Magnetometer attitude calculations*/
    q_rot_v_q(mag_i, q_minus, mag_b); //estimate magnetic field vector in body frame
    cross(mags, mag_b, w_meas_mp); // calculate the magnetometer rate term
    v_copy(w_meas_mp, w_meas_mi); //make a copy for the integral term
    v_scale(kp_m, w_meas_mp); // calculate the magnetometer proportional feedback term
    v_scale(ki_m, w_meas_mi); // calculate the magnetometer integral feedback term

    /*Gyro attitude contributions */
    v_v_sub(gyros, bias_minus, gyro_cal); //correct the gyros with the b_minus vector

    /* calculate total rate term gyro_wfb */
    v_v_add(w_meas_ap, w_meas_mp, gyro_wfb);
    v_v_add(gyro_cal, gyro_wfb, gyro_wfb);

    /* convert feedback term to a pure quaternion */
    gyro_q_wfb[0] = 0;
    gyro_q_wfb[1] = gyro_wfb[0];
    gyro_q_wfb[2] = gyro_wfb[1];
    gyro_q_wfb[3] = gyro_wfb[2];

    /* compute the quaternion derivative q_dot */
    q_mult(q_minus, gyro_q_wfb, q_dot);

    /* integrate term by term */
    q_plus[0] = q_minus[0] + 0.5 * q_dot[0] * dt;
    q_plus[1] = q_minus[1] + 0.5 * q_dot[1] * dt;
    q_plus[2] = q_minus[2] + 0.5 * q_dot[2] * dt;
    q_plus[3] = q_minus[3] + 0.5 * q_dot[3] * dt;

    // normalize the quaternion for stability
    q_n = q_norm(q_plus);
    q_plus[0] = q_plus[0] / q_n;
    q_plus[1] = q_plus[1] / q_n;
    q_plus[2] = q_plus[2] / q_n;
    q_plus[3] = q_plus[3] / q_n;

    // compute the integral of the bias term by term
    bias_plus[0] = bias_minus[0] - (w_meas_ai[0] + w_meas_mi[0]) * dt;
    bias_plus[1] = bias_minus[1] - (w_meas_ai[1] + w_meas_mi[1]) * dt;
    bias_plus[2] = bias_minus[2] - (w_meas_ai[2] + w_meas_mi[2]) * dt;
}

int main(void) {
    int i2c_handle;
    char init_result;
    uint8_t index = 0;
 
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

    /* Matrix and Quaternion arrays */
    const float dt = DT; // integration interval
    const float gyro_scale = 250.0 / (float) ((1 << 15) - 1);
    const float deg2rad = M_PI / 180.0;
    const float rad2deg = 180.0 / M_PI;
    /*Calibration matrices and offsets from tumble test */
    float A_acc[MSZ][MSZ] = {
        {0.00100495454726146, -0.0000216880122750632, 0.0000133999325710038},
        {-0.0000162926752704191, 0.000985380021967908, 0.00000666633241783684},
        {0.0000125438947528585, 0.00000252521314262081, 0.000989136500881465}
    };
    float b_acc[MSZ] = {0.00605446075713124, 0.0531371565285396, 0.0243733439531166};

    float A_mag[MSZ][MSZ] = {
        {0.00135496706593374, 0.0000116105133187656, -0.00000832065758415854},
        {0.00000781613708114319, 0.00137913779319635, 0.00000538838213067807},
        {-0.0000126033738192070, 0.00000739989830409341, 0.00140611428756101}
    };

    float b_mag[MSZ] = {0.480254990338941, -0.286077906618463, 0.528755499855987};

    // gravity inertial vector
    float a_i[MSZ] = {0, 0, 1.0};
    // Earth's magnetic field inertial vector, normalized 
    // North 22,680.8 nT	East 5,217.6 nT	Down 41,324.7 nT, value from NOAA
    // converted into ENU format and normalized:
    float m_i[MSZ] = {0.110011998753301, 0.478219898291142, -0.871322609031072};

    // Euler angles
    float euler[MSZ] = {0, 0, 0};

    // attitude quaternions
    float q_minus[QSZ] = {1, 0, 0, 0};
    float q_plus[QSZ] = {1, 0, 0, 0};
    // gyro bias vector
    float b_minus[MSZ] = {0, 0, 0};
    float b_plus[MSZ] = {0, 0, 0};

    // gryo, accelerometer, magnetometer raw and calibrated vectors
    /* Raw Data arrays */
    int16_t acc_raw[MSZ];
    int16_t mag_raw[MSZ];
    int16_t gyro_raw[MSZ];
    float acc_raw_float[MSZ];
    float mag_raw_float[MSZ];
    int8_t temperature = 0;
    /* Calibrated data arrays */
    float gyro_cal[MSZ];
    float acc_tmp[MSZ]; // intermediate array
    float acc_cal[MSZ];
    float mag_cal[MSZ];
    float mag_tmp[MSZ]; // intermediate array

    /*init */
    printf("AHRS quaternion float evaluation application, compiled " __DATE__ " " __TIME__ "\r\n");
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
            /* convert uint16_t to float for compatibility with linear algebra library */
            for(index = 0; index < MSZ; index++){
                acc_raw_float[index] = (float)acc_raw[index];
                mag_raw_float[index] = (float)mag_raw[index];
                gyro_cal[index] = (float) gyro_raw[index] * gyro_scale * deg2rad; // scale and convert gyro data into rad/s
            }
            /* calibrate sensors */
            m_v_mult(A_acc, acc_raw_float, acc_tmp); // scale and calibrate acc             
            v_v_add(acc_tmp, b_acc, acc_cal); // offset accelerometer data              
            m_v_mult(A_mag, mag_raw_float, mag_tmp); // scale magnetometer data                 
            v_v_add(mag_tmp, b_mag, mag_cal); // offset magnetometer data

            // clock_gettime(CLOCK_REALTIME,&update_start);
            clock_gettime(CLOCK_MONOTONIC, &update_start);
            ahrs_update(q_minus, q_plus, b_minus, b_plus, gyro_cal, mag_cal, acc_cal, m_i, a_i, dt);
            clock_gettime(CLOCK_MONOTONIC, &update_end);
            // clock_gettime(CLOCK_REALTIME, &update_end);
            quat2euler(q_plus, euler); // convert quaternion to euler angles
            /* update b_minus and q_minus */
            b_minus[0] = b_plus[0];
            b_minus[1] = b_plus[1];
            b_minus[2] = b_plus[2];
            q_minus[0] = q_plus[0];
            q_minus[1] = q_plus[1];
            q_minus[2] = q_plus[2];
            q_minus[3] = q_plus[3];
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



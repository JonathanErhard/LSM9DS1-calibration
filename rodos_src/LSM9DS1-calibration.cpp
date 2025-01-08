#include "rodos.h"
#include <cfloat>

// mostly copied from my float-sat repository (sensors app)

#include "hal/hal_i2c.h"
#include "hal/hal_adc.h"

RODOS::HAL_I2C imu(RODOS::I2C_IDX::I2C_IDX2); // PB10 & PB11

#define AccADDR 0x6B // also includes gyro
#define MagADDR 0x1E

// Buffers for data to be read into
uint8_t DATA_L[1];
uint8_t DATA_H[1];

/*
 *  Buffers for all processed values
 */

// GYRO
float GYR_CALIB_VALS[3] = {1.26502215f, -0.03441086f, 0.77206468f};
float GYR_REAL_VALS[3] = {0.0f, 0.0f, 0.0f};
int16_t GYR_RAW_VALS[3] = {0, 0, 0};

// ACCELEROMETER
float ACC_CALIB_VALS[3] = {-0.10875375f, -0.01491124f, 0.03084271f};
float ACC_REAL_VALS[3] = {0.0f, 0.0f, 0.0f};
int16_t ACC_RAW_VALS[3] = {0, 0, 0};

// MAGNETOMETER {x[min,max], y[min,max], z[min,max]}
float MAG_BOUNDRIES[3][2] = {{0.0f, 1.0f}, {0.0f, 1.0f}, {0.0f, 1.0f}}; //
float MAG_REAL_VALS[3] = {0.0f, 0.0f, 0.0f};
int16_t MAG_RAW_VALS[3] = {0, 0, 0};

/*
 *  REGISTER ADDRESSES
 */

// GYRO
uint8_t GYR_X_L[1] = {0x18};
uint8_t GYR_X_H[1] = {0x19};
uint8_t GYR_Y_L[1] = {0x1A};
uint8_t GYR_Y_H[1] = {0x1B};
uint8_t GYR_Z_L[1] = {0x1C};
uint8_t GYR_Z_H[1] = {0x1D};

// ACCELLEROMETER
uint8_t ACC_X_L[1] = {0x28};
uint8_t ACC_X_H[1] = {0x29};
uint8_t ACC_Y_L[1] = {0x2A};
uint8_t ACC_Y_H[1] = {0x2B};
uint8_t ACC_Z_L[1] = {0x2C};
uint8_t ACC_Z_H[1] = {0x2D};

// MAGNETOMETER
uint8_t MAG_X_L[1] = {0x28};
uint8_t MAG_X_H[1] = {0x29};
uint8_t MAG_Y_L[1] = {0x2A};
uint8_t MAG_Y_H[1] = {0x2B};
uint8_t MAG_Z_L[1] = {0x2C};
uint8_t MAG_Z_H[1] = {0x2D};

#define ACC_NUM_ITERATIONS 100   // num measurements to initialize acc in each axis
#define GYR_NUM_ITERATIONS 100   // num measurements to initialize gyro (has to stand still, corriolis not considered)
#define MAG_NUM_ITERATIONS 10000 // num measurements to find min and max magnetometer output

#define DELAY 10 // delay between measurements in ms

class CalibThread : public StaticThread<>
{
protected:
    int iteration;
    // write IMU registers into RAW_VALS buffers. seperate method for calibration purposes
    void readRawIMU()
    {
        // accellerometer
        imu.writeRead(AccADDR, ACC_X_L, 1, DATA_L, 1);
        imu.writeRead(AccADDR, ACC_X_H, 1, DATA_H, 1);
        ACC_RAW_VALS[0] = DATA_L[0] + int16_t((DATA_H[0] << 8));

        imu.writeRead(AccADDR, ACC_Y_L, 1, DATA_L, 1);
        imu.writeRead(AccADDR, ACC_Y_H, 1, DATA_H, 1);
        ACC_RAW_VALS[1] = DATA_L[0] + int16_t((DATA_H[0] << 8));

        imu.writeRead(AccADDR, ACC_Z_L, 1, DATA_L, 1);
        imu.writeRead(AccADDR, ACC_Z_H, 1, DATA_H, 1);
        ACC_RAW_VALS[2] = DATA_L[0] + int16_t((DATA_H[0] << 8));
        // gyroscope
        imu.writeRead(AccADDR, GYR_X_L, 1, DATA_L, 1);
        imu.writeRead(AccADDR, GYR_X_H, 1, DATA_H, 1);
        GYR_RAW_VALS[0] = DATA_L[0] + int16_t((DATA_H[0] << 8));

        imu.writeRead(AccADDR, GYR_Y_L, 1, DATA_L, 1);
        imu.writeRead(AccADDR, GYR_Y_H, 1, DATA_H, 1);
        GYR_RAW_VALS[1] = DATA_L[0] + int16_t((DATA_H[0] << 8));

        imu.writeRead(AccADDR, GYR_Z_L, 1, DATA_L, 1);
        imu.writeRead(AccADDR, GYR_Z_H, 1, DATA_H, 1);
        GYR_RAW_VALS[2] = DATA_L[0] + int16_t((DATA_H[0] << 8));
        // magnetometer
        imu.writeRead(MagADDR, MAG_X_L, 1, DATA_L, 1);
        imu.writeRead(MagADDR, MAG_X_H, 1, DATA_H, 1);
        MAG_RAW_VALS[0] = DATA_L[0] + int16_t((DATA_H[0] << 8));

        imu.writeRead(MagADDR, MAG_Y_L, 1, DATA_L, 1);
        imu.writeRead(MagADDR, MAG_Y_H, 1, DATA_H, 1);
        MAG_RAW_VALS[1] = DATA_L[0] + int16_t((DATA_H[0] << 8));

        imu.writeRead(MagADDR, MAG_Z_L, 1, DATA_L, 1);
        imu.writeRead(MagADDR, MAG_Z_H, 1, DATA_H, 1);
        MAG_RAW_VALS[2] = DATA_L[0] + int16_t((DATA_H[0] << 8));
    }

    void readIMU()
    {
        readRawIMU();

        // calculate acceleration
        for (int j = 0; j < 3; j++)
            ACC_REAL_VALS[j] = (0.061f / 1000) * ACC_RAW_VALS[j] - ACC_CALIB_VALS[j];

        // calculate angular velocity
        for (int j = 0; j < 3; j++)
            GYR_REAL_VALS[j] = (70.0f / 1000) * GYR_RAW_VALS[j] - GYR_CALIB_VALS[j];

        // calculate magnetic vector
        for (int j = 0; j < 3; j++)
            MAG_REAL_VALS[j] = ((MAG_RAW_VALS[j] - MAG_BOUNDRIES[j][0]) / (MAG_BOUNDRIES[j][1] - MAG_BOUNDRIES[j][0])) * 2 - 1; // normalize raw data to be within [0-1] (unlesse boundry is breached)
    }

    // horrendous code, sorry
    template <typename T>
    void print_bits(T num)
    {
        int size = sizeof(T);
        for (int i = 0; i++ < 8 * size; num <<= 1)
            PRINTF("%d", !!(num & 0x01 << 8 * size));
    }

    void printReal()
    {
        /*PRINTF("#########-REAL-###########\nit:%i\nacc: %i, %i, %i\ngyr: %i, %i, %i\nmag: %i, %i, %i\n\n",
               iteration,
               ACC_REAL_VALS[0], ACC_REAL_VALS[1], ACC_REAL_VALS[2],
               GYR_REAL_VALS[0], GYR_REAL_VALS[1], GYR_REAL_VALS[2],
               MAG_REAL_VALS[0], MAG_REAL_VALS[1], MAG_REAL_VALS[2]);*/
    }

    void printRawBits()
    {
        PRINTF("##########-RAW_BITS-###########\nit:");
        print_bits(iteration);
        PRINTF("\nacc:");
        print_bits(ACC_RAW_VALS[0]);
        PRINTF(", ");
        print_bits(ACC_RAW_VALS[1]);
        PRINTF(", ");
        print_bits(ACC_RAW_VALS[2]);
        PRINTF("\ngyr:");
        print_bits(GYR_RAW_VALS[0]);
        PRINTF(", ");
        print_bits(GYR_RAW_VALS[1]);
        PRINTF(", ");
        print_bits(GYR_RAW_VALS[2]);
        PRINTF("\nmag:");
        print_bits(MAG_RAW_VALS[0]);
        PRINTF(", ");
        print_bits(MAG_RAW_VALS[1]);
        PRINTF(", ");
        print_bits(MAG_RAW_VALS[2]);
        PRINTF("\n");
    }

    void printRaw()
    {
        /*PRINTF("##########-RAW-###########\nit:%i\nacc: %i, %i, %i\ngyr: %i, %i, %i\nmag: %i, %i, %i\n\n",
               iteration,
               ACC_RAW_VALS[0], ACC_RAW_VALS[1], ACC_RAW_VALS[2],
               GYR_RAW_VALS[0], GYR_RAW_VALS[1], GYR_RAW_VALS[2],
               MAG_RAW_VALS[0], MAG_RAW_VALS[1], MAG_RAW_VALS[2]);*/
    }

public:
    CalibThread() : StaticThread("Hello World", 1000) {}

    void init()
    {
        imu.init(400000);

        // init all the registered required for I2C communications
        uint8_t INIT_REG_ACC[2] = {0x20, 0b10000011};
        imu.write(AccADDR, INIT_REG_ACC, 2);

        uint8_t INIT_REG_GYR[2] = {0x10, 0b10011000};
        imu.write(AccADDR, INIT_REG_GYR, 2);

        uint8_t INIT_REG_MAG_1[2] = {0x20, 0b00011100};
        imu.write(MagADDR, INIT_REG_MAG_1, 2);

        uint8_t INIT_REG_MAG_2[2] = {0x21, 0b01100000};
        imu.write(MagADDR, INIT_REG_MAG_2, 2);

        uint8_t INIT_REG_MAG_3[2] = {0x22, 0b00000000};
        imu.write(MagADDR, INIT_REG_MAG_3, 2);
    }

    void calibrate_gyr()
    {
        PRINTF("calibrating gyroscope in 5 sec! don't move pls :)\n");
        AT(NOW() + 5 * SECONDS);
        PRINTF("calibrating...");

        iteration = 0;
        float measurement_buffer[3] = {0.0f, 0.0f, 0.0f};
        while (++iteration < GYR_NUM_ITERATIONS)
        {
            readRawIMU();
            for (int i = 0; i < 3; i++)
                measurement_buffer[i] += (70.0f / 1000) * GYR_RAW_VALS[i]; // cant save raw coz it would overflow ): float might be too inaccurate idk
            AT(NOW() + DELAY * MILLISECONDS);
        }
        for (int i = 0; i < 3; i++)
            GYR_CALIB_VALS[i] = measurement_buffer[i] / GYR_NUM_ITERATIONS;
        PRINTF("gyroscope calibration done!\n");
    }

    void calibrate_acc()
    {
        PRINTF("calibrating acc!");

        iteration = 0;
        float measurement_buffer[3] = {0.0f, 0.0f, 0.0f};
        for (int axis = 0; axis < 3; axis++)
        {
            switch (axis)
            {
            case 0:
                PRINTF("calibrating x in 5 sec!");
                break;
            case 1:
                PRINTF("calibrating y in 5 sec!");
                break;
            case 2:
                PRINTF("calibrating z in 5 sec!");
                break;
            }
            AT(NOW() + 5 * SECONDS);
            PRINTF("calibrating...");
            while (++iteration < ACC_NUM_ITERATIONS)
            {

                readRawIMU();
                measurement_buffer[axis] += (0.061f / 1000) * ACC_RAW_VALS[axis];
                AT(NOW() + DELAY * MILLISECONDS);
            }
            ACC_CALIB_VALS[axis] = measurement_buffer[axis] / ACC_NUM_ITERATIONS;
            PRINTF("axis %d done!", axis);
        }
        PRINTF("gyroscope calibration done!");
    }

    void calibrate_mag()
    {
        PRINTF("calibrating magnetometer in 5 sec! try to hit every orientation :) calibration will take %d s\n", MAG_NUM_ITERATIONS / SECONDS);
        AT(NOW() + 5 * SECONDS);
        PRINTF("calibrating...");
        iteration = 0;
        for (int i = 0; i < 3; i++)
        {
            MAG_BOUNDRIES[i][0] = -FLT_MAX;
            MAG_BOUNDRIES[i][1] = FLT_MAX;
        }
        while (++iteration < MAG_NUM_ITERATIONS)
        {
            readRawIMU();
            for (int i = 0; i < 3; i++)
            {
                MAG_BOUNDRIES[i][0] = min((float)MAG_RAW_VALS[0], MAG_BOUNDRIES[i][0]);
                MAG_BOUNDRIES[i][0] = max((float)MAG_RAW_VALS[0], MAG_BOUNDRIES[i][0]);
            }
            AT(NOW() + DELAY * MILLISECONDS);
            if (iteration % max((SECONDS / DELAY), (long long int)1) == 0)
                PRINTF("%d seconds left!", (MAG_NUM_ITERATIONS - iteration) / SECONDS);
        }
    }

    void
    run()
    {
        calibrate_gyr();
        calibrate_acc();
        calibrate_mag();
        PRINTF("CALIBRATION VALUE:");
        PRINTF("gyr = {%f,%f,%f}", GYR_CALIB_VALS[0], GYR_CALIB_VALS[1], GYR_CALIB_VALS[2]);
        PRINTF("acc = {%f,%f,%f}", ACC_CALIB_VALS[0], ACC_CALIB_VALS[1], ACC_CALIB_VALS[2]);
        PRINTF("mag = {{%f,%f},{%f,%f},{%f,%f}}", MAG_BOUNDRIES[0][0], MAG_BOUNDRIES[0][1], MAG_BOUNDRIES[1][0], MAG_BOUNDRIES[1][1], MAG_BOUNDRIES[2][0], MAG_BOUNDRIES[2][1]);
    }
} calibThread;
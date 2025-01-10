#include "rodos.h"
#include <cfloat> // for max/min float

#include "../drivers/lsm9ds1/include/lsm9ds1.h" //drivers not included by default

// IMU::IMU();

#define ACC_NUM_ITERATIONS 100  // num measurements to initialize acc in each axis
#define GYR_NUM_ITERATIONS 100  // num measurements to initialize gyro (has to stand still, corriolis not considered)
#define MAG_NUM_ITERATIONS 1000 // num measurements to find min and max magnetometer output

#define DELAY 10 // delay between measurements in ms

class CalibThread : public StaticThread<>
{
protected:
    int16_t GYR_CALIB_VALS[3];
    int16_t ACC_CALIB_VALS[3];
    int16_t MAG_BOUNDRIES[3][2];

    IMU::LMS9DS1 imu;

public:
    CalibThread() : StaticThread("LSM9DS1_CALIB_THREAD", 10), imu() {}

    void init()
    {
        imu.init(400000);
    }

    void calibrate_gyr()
    {
        PRINTF("calibrating gyroscope in 5 sec! don't move pls :)\n");
        AT(NOW() + 5 * SECONDS);
        PRINTF("calibrating...\n");

        int iteration = 0;
        int16_t measurement_buffer[3] = {0, 0, 0};
        while (++iteration < GYR_NUM_ITERATIONS)
        {
            imu.read_raw();
            for (int i = 0; i < 3; i++)
                measurement_buffer[i] += imu.GYR_RAW_VALS[i]; // cant save raw coz it would overflow ): float might be too inaccurate idk
            AT(NOW() + DELAY * MILLISECONDS);
        }
        for (int i = 0; i < 3; i++)
            GYR_CALIB_VALS[i] = measurement_buffer[i] / GYR_NUM_ITERATIONS;
        PRINTF("gyroscope calibration done!\n");
    }

    void calibrate_acc()
    {
        PRINTF("calibrating acc!\n");

        int iteration = 0;
        int16_t measurement_accumulator[3] = {0, 0, 0};
        for (int axis = 0; axis < 3; axis++)
        {
            switch (axis)
            {
            case 0:
                PRINTF("calibrating x in 5 sec!\n");
                break;
            case 1:
                PRINTF("calibrating y in 5 sec!\n");
                break;
            case 2:
                PRINTF("calibrating z in 5 sec!\n");
                break;
            }
            AT(NOW() + 5 * SECONDS);
            PRINTF("calibrating...\n");
            while (++iteration < ACC_NUM_ITERATIONS)
            {

                imu.read_raw();
                measurement_accumulator[axis] += imu.ACC_RAW_VALS[axis];
                AT(NOW() + DELAY * MILLISECONDS);
            }
            ACC_CALIB_VALS[axis] = measurement_accumulator[axis] / ACC_NUM_ITERATIONS;
            PRINTF("axis %d done!\n", axis);
        }
        PRINTF("gyroscope calibration done!\n");
    }

    void calibrate_mag()
    {
        PRINTF("calibrating magnetometer in 5 sec! try to hit every orientation :) calibration will take %lld s\n", (DELAY * MAG_NUM_ITERATIONS) / MILLISECONDS);
        AT(NOW() + 5 * SECONDS);
        PRINTF("calibrating...\n");
        int iteration = 0;
        for (int i = 0; i < 3; i++)
        {
            MAG_BOUNDRIES[i][0] = INT16_MAX;
            MAG_BOUNDRIES[i][1] = INT16_MIN;
        }
        while (++iteration < MAG_NUM_ITERATIONS)
        {
            imu.read_raw();
            imu.print_raw();
            for (int i = 0; i < 3; i++)
            {
                MAG_BOUNDRIES[i][0] = min(imu.MAG_RAW_VALS[i], MAG_BOUNDRIES[i][0]);
                MAG_BOUNDRIES[i][1] = max(imu.MAG_RAW_VALS[i], MAG_BOUNDRIES[i][1]);
            }
            AT(NOW() + DELAY * MILLISECONDS);
            if (iteration % max((SECONDS / DELAY), (long long int)1) == 0)
                PRINTF("%lld seconds left!\n", (MAG_NUM_ITERATIONS - iteration) / SECONDS);
        }
    }

    void
    run()
    {
        calibrate_gyr();
        calibrate_acc();
        calibrate_mag();
        PRINTF("CALIBRATION VALUE:\n");
        PRINTF("gyr = {%d,%d,%d}\n", GYR_CALIB_VALS[0], GYR_CALIB_VALS[1], GYR_CALIB_VALS[2]);
        PRINTF("acc = {%d,%d,%d}\n", ACC_CALIB_VALS[0], ACC_CALIB_VALS[1], ACC_CALIB_VALS[2]);
        PRINTF("mag = {{%d,%d},{%d,%d},{%d,%d}}\n", MAG_BOUNDRIES[0][0], MAG_BOUNDRIES[0][1], MAG_BOUNDRIES[1][0], MAG_BOUNDRIES[1][1], MAG_BOUNDRIES[2][0], MAG_BOUNDRIES[2][1]);
    }
} calibThread;
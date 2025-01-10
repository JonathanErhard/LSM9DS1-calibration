#include "rodos.h"
#include <cfloat>
#include "../drivers/lsm9ds1/include/lsm9ds1.h"

#define DELAY 10 // delay between measurements in ms

class IMUtestThread : public StaticThread<>
{
protected:
    int iteration;
    IMU::LMS9DS1 imu;

public:
    IMUtestThread() : StaticThread("Hello World", 1000), imu()
    {
    }

    void init()
    {
        imu.init(400000);
    }

    void
    run()
    {
        while (1)
        {
            imu.read_adj();
            // you can use the other print methods if that is more interesting to you c:
            imu.print_real();
            AT(NOW() + DELAY * MILLISECONDS);
        }
    }
} calibThread;
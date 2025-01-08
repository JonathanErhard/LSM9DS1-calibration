
# LSM9DS1-Calibration

Fork created by Jonathan Erhard.
Original creator: Konstantin Winkel

rodos-src/LSM9DS1-calibration.cpp is the file created in this Fork, the rest is (mostly) part of the original repository.

# HowTo setup

use

```bash
./scripts/setup.sh discovery
```

to clone and compile rodos. After cloning open "rodos/src/bare-metal/stm32f4/platform-parameters/discovery/platform-parameters.h"
and comment line 31 and uncomment line 33 to enable uart via the programming wire.
Call

```bash
./scripts/setup.sh discovery
```

again to recompile rodos!

# HowTo calibrate

After that you can calibrate the IMU by using

```bash
./scripts/setup.sh discovery
```

to compile rodos-src/LSM9DS1-calibration.cpp and flash the board (make sure the programming wire is connected.)
Make sure LSM9DS1-calibration.cpp is in CompileList.txt c: Note: the command does not recompile rodos!
the rest of the calibration is trivial if you do what the print statements tell you to do.

# HowTo test

set program to test_IMU.cpp using CompileList.txt (just write test_IMU.cpp into the file) and call

```bash
./scripts/setup.sh discovery
```

to compile and flash the test-file. Note: the command does not recompile rodos!

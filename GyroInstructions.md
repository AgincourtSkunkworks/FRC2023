# How to (Properly) Use the Gyroscope
So that future me, or whoever does this next year isn't absolutely clueless.

## Table of Contents
[Documentation](#documentation)
[Mounting](#mounting)
[Calibration](#calibration)
[Libraries](#libraries)
[\[Code\] Setup](#setup)
[Method Reference](#method-reference)

## Documentation
The manufacturer provides a long and in-depth [guide](https://pdocs.kauailabs.com/navx-mxp/wp-content/uploads/2020/09/navx2-mxp_robotics_navigation_sensor_user_guide-8.pdf) on pretty much everything for the Gyro. This information is a summary of personal tests and information from that guide, however you're free to read the full guide if you need something not mentioned here, or want more information (but good luck to your sanity).

## Mounting
Preferably, the Gyroscope should be mounted to the MXP port on the RoboRIO (this is the long port in the center of the RoboRIO, there is a connector on the back of the Gyroscope that fits this). However, in situations where the Gyroscope needs to be mounted elsewhere (which should be rare), mounting via SPI is then preferred (port on the top right of the RoboRIO, line of pins labeled 'SPI' on the Gyroscope). If mounting via SPI is not possible, the last resort should be USB mounting (USB port on the RoboRIO, to the Mini USB port on the Gyroscope).

## Calibration
The board almost always **needs to be calibrated manually**, otherwise the readings will be extremely inaccurate. This should be done anytime the Gyro is remounted, just in case. The calibration is done to ensure that the Gyroscope properly determines what mounting orientation it is in, so that the readings are proper and accurate. Theoretically if the mounting position is the same when moving the Gyroscope, a calibration isn't needed, but it's safer to do one anyhow.

[Full Calibration Guide](https://pdocs.kauailabs.com/navx-mxp/installation/omnimount/)

**Ensure that the robot is not moved during the calibration sequence**

1. Install the Gyroscope to the desired location. Ensure that one axis perpendicular to the ground (cannot be diagonal or slanted).
2. Power on the robot and wait until the Gyroscope is on
3. Press and hold the 'CAL' button on the Gyroscope for 5 seconds.
4. Release the 'CAL' button. The orange light should flash (or in our case turn on, either is fine) for 1 second before turning off.
5. Once the light has turned off, press the 'RESET' button.
6. The board is automatically calibrating, do not touch the robot or the board during this process. The orange LED should be slowly flashing during this process. It may take anywhere between 2-20 seconds.
    - If the orange LED begins flashing quickly at any point, there is an error in calibration. Either the Gyroscope is not mounted with one axis perpendicular to the ground, or the Gyroscope was moved during calibration. Power cycle (may or may not be optional), then reperform these steps to try again.
7. Calibration should be complete, values should be proper and calibration is no longer necessary until moved.

## Code
### Libraries
The Gyroscope needs a library to function, full installation instructions are available [here](https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/java), this is a short overview of the Online Installation Method.

1. Open the project in VSCode
2. Open Command Pallette (Ctrl + P)
3. Type in `>WPILib: Manage Vendor Libraries`, and open that option
4. Select the `Install new ibrary (online)` option
5. Enter the following link: `https://dev.studica.com/releases/2023/NavX.json`
    - This link may change, it may be a good idea to check the full official installation instructions to see if it was updated.
6. Once the installation finishes, your library should be installed

### Setup
Depending on the method of connection (MXP, SPI, USB), the initialization code may need to be adjusted. This section is presuming that you are using MXP, you will need to do additional research for other methods of connection.

The following imports are needed (one for the Gyro library, other for the port):
```java
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
```

Initialize the AHRS object (object used to obtain readings) by using the following line of code:
```java
AHRS ahrs = new AHRS(SPI.Port.kMXP);
```

This object can then be used to obtain readings, using the methods shown below.

### Method Reference
All of these need to be prefixed with the AHRS object. If the variable it is stored in is named `ahrs`, then it would be `ahrs.isConnected()` to call `isConnected()`, for example.

This list does not include every possible reference. [This list](https://img.theodorehua.dev/ePlgpGHY) includes a few more, but not all. If you need all of them, use `F12` (View Definition) while your cursor is on the AHRS object initialization, which will open the Source file for it. You can look through all the methods there.

- **`isConnected()`**: Boolean value of whether the Gyroscope is connected (and subsequently whether readings can be obtained)
- **`zeroYaw()`**: Set the current angle the robot is facing as the zero-point for YAW. (when you face this angle, `getYaw()` will read 0)
- **`getYaw()`** or **`getAngle()`**: Reading for YAW (rotation around the Z axis, which is perpendicular to the earth) in degrees, from -180 to 180. If past 180, it will circle back to -180, and vice-versa. Turning left should normally be decreasing.
- **`getPitch()`**: Reading for pitch (rotation around the X axis \[up/down\]) in degrees, from -180 to 180. If past 180, it will circle back to -180, and vice-versa. Down decreases, up increases.
- **`getCompassHeading()`**: Supposedly the reading for compass heading, from 0 to 360, read using a mangnetometer. This reading **did not work** in our testing, it may be interference, lack of calibration, wrong usage, or it just doesn't work.

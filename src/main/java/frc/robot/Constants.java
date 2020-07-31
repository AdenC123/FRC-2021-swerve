/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class MotorControllers {
        public static final int
            RF_DRIVE = 1,
            RF_SPIN = 2,
            RR_DRIVE = 3,
            RR_SPIN = 4,
            LR_DRIVE = 5,
            LR_SPIN = 6,
            LF_DRIVE = 7,
            LF_SPIN = 8;
    }

    public static final class AnalogPorts {
        public static final int
            RF = 0,
            RR = 1,
            LR = 2,
            LF = 3;
    }

    public static final class CalibrationOffset {
        public static final double
            RF = 0.911,
            RR = 0.982,
            LR = 0.771,
            LF = 0.853;
    }

    public static double MAX_DRIVE_VELOCITY = 5500;

    // length and width from center of the wheels, in cm (unit doesnt matter)
    public static double DRIVE_LENGTH = 57.4;
    public static double DRIVE_WIDTH = 57.7;

    // Spin motor PID values for testing only. Once found, should be set permanently using Spark client app.
    public static double SPIN_kP = 0.25;
    public static double SPIN_kI = 0;

    public static double DRIVE_kP = 0;
    public static double DRIVE_kI = 0;
    public static double DRIVE_kFF = 0.000170;
    public static double DRIVE_kMIN = -1;
    public static double DRIVE_kMAX = 1;
}

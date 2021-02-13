/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import org.a05annex.util.Utl;
import org.json.simple.JSONObject;
import org.json.simple.parser.ContainerFactory;
import org.json.simple.parser.ParseException;

import java.io.FileWriter;
import java.io.IOException;

import static org.a05annex.util.JsonSupport.*;
import static org.a05annex.util.Utl.TWO_PI;

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
            RF = 0.916,
            RR = 0.980,
            LR = 0.774,
            LF = 0.853;
    }

    // This is the maximum velocity read by the encoders being used to control the drive speed. The actual
    // maximum is closer to 5700, but we have scaled that down a bit to account for friction, drag, lower
    // battery voltage, etc.
    public static final double MAX_DRIVE_VELOCITY = 5000;

    // length and width from center of the wheels, in m
    public static final double DRIVE_LENGTH = 0.590;
    public static final double DRIVE_WIDTH = 0.590;
    public static final double DRIVE_DIAGONAL = Utl.length(DRIVE_LENGTH, DRIVE_WIDTH);
    public static final double DRIVE_RADIUS = DRIVE_DIAGONAL / 2.0;
    // 18 motor revolutions to 1 spin of the drive wheel
    public static final double RADIANS_TO_SPIN_ENCODER = 18.0 / TWO_PI;

    // maximum linear speed and rotational speed. What we are really interested in knowing is how the robot
    // can travel or turn in one command cycle at full speed because for path following we need to know how
    // big to make the increments along the path, and need a pretty good estimate of where the robot is for
    // making course corrections.
    public static final double MAX_METERS_PER_SEC = 3.1951;
    public static final double MAX_RADIANS_PER_SEC = MAX_METERS_PER_SEC / DRIVE_RADIUS;

    // PID values for the spin spark motor controller PID loop
    public static double SPIN_kP = 0.25;
    public static double SPIN_kI = 0.0;

    // PID values for the drive spark motor controller speed PID loop
    public static double DRIVE_kP = 0.00003;
    public static double DRIVE_kI = 0.000002;
    public static double DRIVE_kFF = 0.000174;
    public static double DRIVE_IZONE = 200.0;

    // PID values for the drive spark motor controller position PID loop
    public static double DRIVE_POS_kP = 0.13;
    public static double DRIVE_POS_kI = 0.0;

    // drive encoder tics per radian of robot rotation when rotation is controlled by position rather than speed.
    public static double DRIVE_POS_TICS_PER_RADIAN = 10.385;

    // driver enumerator
    public enum DRIVERS {
        NOLAN("Nolan", Filesystem.getDeployDirectory().toString() + "/drivers/nolan.json"),
        KALVIN("Kalvin", Filesystem.getDeployDirectory().toString() + "/drivers/kalvin.json");

        private static final String USE_CONTROLLER = "USE_CONTROLLER";
        private static final String XBOX_CONTROLLER = "XBOX";
        private static final String JOYSTICK_CONTROLLER = "JOYSTICK";

        private static final String DRIVE_DEADBAND = "DRIVE_DEADBAND";
        private static final String DRIVE_SPEED_SENSITIVITY = "DRIVE_SPEED_SENSITIVITY";
        private static final String DRIVE_SPEED_GAIN = "DRIVE_SPEED_GAIN";
        private static final String TWIST_DEADBAND = "TWIST_DEADBAND";
        private static final String TWIST_SENSITIVITY = "TWIST_SENSITIVITY";
        private static final String TWIST_GAIN = "TWIST_GAIN";

        String m_driverName;
        String m_driverFile;
        DRIVERS(String driverName, String driverFile) {
            m_driverName = driverName;
            m_driverFile = driverFile;
        }

        public void load() {
            try {
                JSONObject dict = readJsonFileAsJSONObject(m_driverFile);
                if (null != dict) {
                    // Read in the driver data
                    Constants.USE_CONTROLLER = parseString(dict, USE_CONTROLLER, Constants.USE_CONTROLLER);
                    Constants.DRIVE_DEADBAND = parseDouble(dict, DRIVE_DEADBAND, Constants.DRIVE_DEADBAND);
                    Constants.DRIVE_SPEED_SENSITIVITY = parseDouble(dict, DRIVE_SPEED_SENSITIVITY, Constants.DRIVE_SPEED_SENSITIVITY);
                    Constants.DRIVE_SPEED_GAIN = parseDouble(dict, DRIVE_SPEED_GAIN, Constants.DRIVE_SPEED_GAIN);
                    Constants.TWIST_DEADBAND = parseDouble(dict, TWIST_DEADBAND, Constants.TWIST_DEADBAND);
                    Constants.TWIST_SENSITIVITY = parseDouble(dict, TWIST_SENSITIVITY, Constants.TWIST_SENSITIVITY);
                    Constants.TWIST_GAIN = parseDouble(dict, TWIST_GAIN, Constants.TWIST_GAIN);
                }

            } catch (IOException | ParseException | ClassCastException | NullPointerException e) {
                e.printStackTrace();
            }
        }

        public static void save() {
            JSONObject dict = new JSONObject();
            dict.put(USE_CONTROLLER, Constants.USE_CONTROLLER);
            dict.put(DRIVE_DEADBAND, Constants.DRIVE_DEADBAND);
            dict.put(DRIVE_SPEED_SENSITIVITY, Constants.DRIVE_SPEED_SENSITIVITY);
            dict.put(DRIVE_SPEED_GAIN, Constants.DRIVE_SPEED_GAIN);
            dict.put(TWIST_DEADBAND, Constants.TWIST_DEADBAND);
            dict.put(TWIST_SENSITIVITY, Constants.TWIST_SENSITIVITY);
            dict.put(TWIST_GAIN, Constants.TWIST_GAIN);
            try (FileWriter file = new FileWriter(Constants.currentDriver.m_driverFile)) {
                file.write(dict.toJSONString());
                file.flush();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        public static void setDriverAtID(int ID) {
            switch (ID) {
                case 0:
                    Constants.currentDriver = NOLAN;
                    break;
                case 1:
                    Constants.currentDriver = KALVIN;
                    break;
                default:
                    Constants.currentDriver = KALVIN;
            }
            Constants.currentDriver.load();
        }
    }

    // used in DriveCommand
    public static DRIVERS currentDriver = DRIVERS.KALVIN;

    public static String USE_CONTROLLER = DRIVERS.JOYSTICK_CONTROLLER;

    public static double DRIVE_DEADBAND = 0.1;
    public static double DRIVE_SPEED_SENSITIVITY = 2.0;
    public static double DRIVE_SPEED_GAIN = 0.5;

    public static double TWIST_DEADBAND = 0.1;
    public static double TWIST_SENSITIVITY = 2.0;
    public static double TWIST_GAIN = 1.0;

    // small number for zero check
    public static final double SMALL = 0.000001;

    // temp variable used in DriveDistance
    public static double DRIVE_SPEED = 0.0;

    // PID values for rotation to target using a PID on current heading and target direction to keep ths robot
    // oriented to the target while driving.
    public static double TARGET_kP = 0.5;
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DriveModule;
import frc.robot.NavX;
import frc.robot.Utl;

public class DriveSubsystem extends SubsystemBase {

    // create drive modules
    private final DriveModule m_rf;
    private final DriveModule m_rr;
    private final DriveModule m_lf;
    private final DriveModule m_lr;

    // create NavX - the drive subsystem owns the NavX and is responsible for the heading update
    // cycle.
    private final NavX m_navx = NavX.getInstance();

    // the drive geometry
    private final double LENGTH_OVER_DIAGONAL =
            Constants.DRIVE_LENGTH / Constants.DRIVE_DIAGONAL;
    private final double WIDTH_OVER_DIAGONAL =
            Constants.DRIVE_WIDTH / Constants.DRIVE_DIAGONAL;

    // keep track of last angles
    private double m_RF_lastRadians = 0.0;
    private double m_RR_lastRadians = 0.0;
    private double m_LF_lastRadians = 0.0;
    private double m_LR_lastRadians = 0.0;

    // keep track of the last chassis speeds for odometry
    private double m_thisChassisForward = 0.0;
    private double m_thisChassisStrafe = 0.0;
    private double m_thisChassisRotation = 0.0;
    private long m_lastTime = System.currentTimeMillis();
    private double m_lastHeading = 0.0;
    private double m_lastChassisForward = 0.0;
    private double m_lastChassisStrafe = 0.0;
    private double m_lastChassisRotation = 0.0;

    private double m_fieldX = 0.0;
    private double m_fieldY = 0.0;
    private double m_fieldHeading = 0.0;


    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem() {
        // Initialization Here
        m_rf = DriveModule.factory(Constants.MotorControllers.RF_DRIVE, Constants.MotorControllers.RF_SPIN,
                Constants.AnalogPorts.RF, Constants.CalibrationOffset.RF);

        m_rr = DriveModule.factory(Constants.MotorControllers.RR_DRIVE, Constants.MotorControllers.RR_SPIN,
                Constants.AnalogPorts.RR, Constants.CalibrationOffset.RR);

        m_lf = DriveModule.factory(Constants.MotorControllers.LF_DRIVE, Constants.MotorControllers.LF_SPIN,
                Constants.AnalogPorts.LF, Constants.CalibrationOffset.LF);

        m_lr = DriveModule.factory(Constants.MotorControllers.LR_DRIVE, Constants.MotorControllers.LR_SPIN,
                Constants.AnalogPorts.LR, Constants.CalibrationOffset.LR);
    }

    public DriveModule getRFModule() {
        return m_rf;
    }

    public DriveModule getRRModule() {
        return m_rr;
    }

    public DriveModule getLFModule() {
        return m_lf;
    }

    public DriveModule getLRModule() {
        return m_lr;
    }

    public void resetDrivePID() {
        m_rf.setDrivePID();
        m_rr.setDrivePID();
        m_lf.setDrivePID();
        m_lr.setDrivePID();
    }

    /**
     * Swerve drive with a forward, strafe and rotate.
     *
     * @param forward  Drive forward. From -1 to 1.
     * @param strafe   Strafe right. From -1 to 1.
     * @param rotation Clockwise rotation. From -1 to 1.
     */
    private void swerveDriveComponents(double forward, double strafe,
                                       double rotation) {

        // calculate a, b, c and d variables
        double a = strafe - (rotation * LENGTH_OVER_DIAGONAL);
        double b = strafe + (rotation * LENGTH_OVER_DIAGONAL);
        double c = forward - (rotation * WIDTH_OVER_DIAGONAL);
        double d = forward + (rotation * WIDTH_OVER_DIAGONAL);

        // calculate wheel speeds
        double rfSpeed = Utl.length(b, c);
        double lfSpeed = Utl.length(b, d);
        double lrSpeed = Utl.length(a, d);
        double rrSpeed = Utl.length(a, c);

        // normalize speeds
        double max = Utl.max(rfSpeed, lfSpeed, lrSpeed, rrSpeed);
        if (max > 1.0) {
            rfSpeed /= max;
            lfSpeed /= max;
            lrSpeed /= max;
            rrSpeed /= max;
            forward /= max;
            strafe /= max;
            rotation /= max;
        }

        // if speed is small or 0, (i.e. essentially stopped), use the last angle because its next motion
        // will probably be very close to its current last motion - i.e. the next direction will probably
        // be very close to the last direction.
        m_RF_lastRadians = (rfSpeed < Constants.SMALL) ? m_RF_lastRadians : Math.atan2(b, c);
        m_LF_lastRadians = (lfSpeed < Constants.SMALL) ? m_LF_lastRadians : Math.atan2(b, d);
        m_LR_lastRadians = (lrSpeed < Constants.SMALL) ? m_LR_lastRadians : Math.atan2(a, d);
        m_RR_lastRadians = (rrSpeed < Constants.SMALL) ? m_RR_lastRadians : Math.atan2(a, c);

        // run wheels at speeds and angles
        m_rf.setRadiansAndSpeed(m_RF_lastRadians, rfSpeed);
        m_lf.setRadiansAndSpeed(m_LF_lastRadians, lfSpeed);
        m_lr.setRadiansAndSpeed(m_LR_lastRadians, lrSpeed);
        m_rr.setRadiansAndSpeed(m_RR_lastRadians, rrSpeed);

        // save the values we set for use in odometry calculations
        m_thisChassisForward = forward;
        m_thisChassisStrafe = strafe;
        m_thisChassisRotation = rotation;
    }

    /**
     * Swerve drive with a robot-relative direction (angle in radians), a speed and a rotation.
     *
     * @param chassisDirection (double) The robot chassis relative direction in radians from -Math.PI to
     *                         Math.PI where 0.0 is towards the front of the robot, and positive is clockwise.
     * @param speed            (double) Speed from 0.0 to 1.0.
     * @param rotation         (double) Clockwise rotation speed from -1.0 to 1.0.
     */
    public void swerveDrive(double chassisDirection, double speed, double rotation) {
        swerveDriveComponents(Math.cos(chassisDirection) * speed,
                Math.sin(chassisDirection) * speed, rotation);
    }

    /**
     * Swerve drive with a field-relative direction (angle in radians), a speed and a rotation.
     *
     * @param fieldDirection (double) The direction in radians from -Math.PI to Math.PI where 0.0 is away from the
     *                       driver, and positive is clockwise.
     * @param speed          (double) Speed from 0.0 to 1.0.
     * @param rotation       (double) Clockwise rotation speed from -1.0 to 1.0.
     */
    public void swerveDriveFieldRelative(double fieldDirection, double speed, double rotation) {
        double chassisDirection = fieldDirection - m_navx.getHeading();
        swerveDrive(chassisDirection, speed, rotation);
    }

    /**
     * Swerve drive with a robot-relative direction (angle in degrees), a speed and a rotation.
     *
     * @param direction (double) The direction from -180.0 to 180.0 degrees where 0.0 is towards the
     *                  front of the robot, and positive is clockwise.
     * @param speed     Speed from 0.0 to 1.0.
     * @param rotation  Clockwise rotation speed from -1.0 to 1.0.
     */
    public void swerveDriveDegrees(double direction, double speed, double rotation) {
        swerveDrive(Math.toRadians(direction), speed, rotation);
    }

    /**
     * Set the field position of the robot. This is typically called at the beginning of the autonomous
     * command as the command that is run should know where the robot has been placed on the field. It
     * could also be called during play if machine vision, sensors, or some other method is available
     * o locate the robot on the field.
     *
     * @param fieldX  (double) The X location of the robot on the field.
     * @param fieldY  (double) The Y location of the robot on the field.
     * @param heading (double) The heading of the robot on the field.
     */
    public void setFieldPosition(double fieldX, double fieldY, double heading) {
        m_fieldX = fieldX;
        m_fieldY = fieldY;
        m_fieldHeading = heading;
        m_navx.initializeHeadingAndNav(heading);
        m_lastTime = System.currentTimeMillis();
    }

    public double getFieldX() {
        return m_fieldX;
    }

    public double getFieldY() {
        return m_fieldY;
    }

    public double getFieldHeading() {
        return m_fieldHeading;
    }

    //TODO
    public void setHeading(double targetHeading) {
        m_RF_lastRadians = Math.atan2(Constants.DRIVE_LENGTH, -Constants.DRIVE_WIDTH);
        m_LF_lastRadians = Math.atan2(Constants.DRIVE_LENGTH, Constants.DRIVE_WIDTH);
        m_LR_lastRadians = Math.atan2(-Constants.DRIVE_LENGTH, Constants.DRIVE_WIDTH);
        m_RR_lastRadians = Math.atan2(-Constants.DRIVE_LENGTH, -Constants.DRIVE_WIDTH);
        
        double deltaTics = (targetHeading - m_navx.getHeading()) * Constants.DRIVE_POS_TICS_PER_RADIAN;

        m_rf.setRadiansAndDistance(m_RF_lastRadians, deltaTics);
        m_lf.setRadiansAndDistance(m_LF_lastRadians, deltaTics);
        m_lr.setRadiansAndDistance(m_LR_lastRadians, deltaTics);
        m_rr.setRadiansAndDistance(m_RR_lastRadians, deltaTics);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Update the NavX heading
        m_navx.recomputeHeading(false);
        // Update the odometry for the drive. OK, the scam here is that there was a previous heading set
        // in the last command cycle when we were setting the new direction/speed/rotation for the
        // chassis, and the heading we are at now. For odometry, assume the average of the last heading and current
        // heading approximates the path of the robot and that the last speed set happened pretty
        // instantaneously. In that case, we can make a pretty good guess how the robot moved on the field.

        // Get the average speed and heading for this interval
        double currentHeading = m_navx.getHeading();
        double aveHeading = (m_lastHeading + currentHeading) * 0.5;
        double aveForward = (m_lastChassisForward + m_thisChassisForward) * 0.5;
        double aveStrafe = (m_lastChassisStrafe + m_thisChassisStrafe) * 0.5;

        // the the maximum distance we could travel in this interval at max speed
        long now = System.currentTimeMillis();
        double maxDistanceInInterval = Constants.MAX_METERS_PER_SEC * (double) (now - m_lastTime) / 1000.0;

        // compute the distance in field X and Y and update the field position
        double sinHeading = Math.sin(aveHeading);
        double cosHeading = Math.cos(aveHeading);
        m_fieldX += ((aveForward * sinHeading) + (aveStrafe * cosHeading)) * maxDistanceInInterval;
        m_fieldY += ((aveForward * cosHeading) - (aveStrafe * sinHeading)) * maxDistanceInInterval;

        // save the current state as the last state
        m_lastHeading = m_fieldHeading = currentHeading;
        m_lastChassisForward = m_thisChassisForward;
        m_lastChassisStrafe = m_thisChassisStrafe;
        m_lastChassisRotation = m_thisChassisRotation;
        m_lastTime = now;
    }
}

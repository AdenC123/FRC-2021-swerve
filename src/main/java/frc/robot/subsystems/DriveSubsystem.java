/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

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

    // the drive geometry
    private final double lengthOverDiagonal =
            Constants.DRIVE_LENGTH / Utl.length(Constants.DRIVE_LENGTH, Constants.DRIVE_WIDTH);
    private final double widthOverDiagonal =
            Constants.DRIVE_WIDTH / Utl.length(Constants.DRIVE_LENGTH, Constants.DRIVE_WIDTH);

    // keep track of last angles
    private double m_lastRFAngle = 0.0;
    private double m_lastRRAngle = 0.0;
    private double m_lastLFAngle = 0.0;
    private double m_lastLRAngle = 0.0;

    // create NavX
    private final NavX m_navx = NavX.getInstance();

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem() {
        // Initialization Here
        m_rf = new DriveModule(Constants.MotorControllers.RF_DRIVE, Constants.MotorControllers.RF_SPIN,
                Constants.AnalogPorts.RF, Constants.CalibrationOffset.RF);

        m_rr = new DriveModule(Constants.MotorControllers.RR_DRIVE, Constants.MotorControllers.RR_SPIN,
                Constants.AnalogPorts.RR, Constants.CalibrationOffset.RR);

        m_lf = new DriveModule(Constants.MotorControllers.LF_DRIVE, Constants.MotorControllers.LF_SPIN,
                Constants.AnalogPorts.LF, Constants.CalibrationOffset.LF);

        m_lr = new DriveModule(Constants.MotorControllers.LR_DRIVE, Constants.MotorControllers.LR_SPIN,
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

    /**
     * Swerve drive with a forward, strafe and rotate.
     *
     * @param fwd Drive forward. From -1 to 1.
     * @param str Strafe right. From -1 to 1.
     * @param rcw Clockwise rotation. From -1 to 1.
     */
    public void swerveDriveComponents(double fwd, double str, double rcw) {

        // calculate a, b, c and d variables
        double a = str - (rcw * lengthOverDiagonal);
        double b = str + (rcw * lengthOverDiagonal);
        double c = fwd - (rcw * widthOverDiagonal);
        double d = fwd + (rcw * widthOverDiagonal);

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
        }

        // if speed is small or 0, don't change angle
        m_lastRFAngle = (rfSpeed < Constants.SMALL) ? m_lastRFAngle : Math.toDegrees(Math.atan2(b, c));
        m_lastLFAngle = (lfSpeed < Constants.SMALL) ? m_lastLFAngle : Math.toDegrees(Math.atan2(b, d));
        m_lastLRAngle = (lrSpeed < Constants.SMALL) ? m_lastLRAngle : Math.toDegrees(Math.atan2(a, d));
        m_lastRRAngle = (rrSpeed < Constants.SMALL) ? m_lastRRAngle : Math.toDegrees(Math.atan2(a, c));


        // run wheels at speeds and angles
        m_rf.setDegreesAndSpeed(m_lastRFAngle, rfSpeed);
        m_lf.setDegreesAndSpeed(m_lastLFAngle, lfSpeed);
        m_lr.setDegreesAndSpeed(m_lastLRAngle, lrSpeed);
        m_rr.setDegreesAndSpeed(m_lastRRAngle, rrSpeed);
    }

    /**
     * Swerve drive with a robot-relative direction (angle in radians), a speed and a rotation.
     *
     * @param direction (double) The direction from -Math.PI to Math.PI radians where 0.0 is towards the
     *                  front of the robot, and positive is clockwise.
     * @param speed     Speed from 0.0 to 1.0.
     * @param rotation  Clockwise rotation speed from -1.0 to 1.0.
     */
    public void swerveDrive(double direction, double speed, double rotation) {
        swerveDriveComponents(Math.cos(direction) * speed, Math.sin(direction) * speed, rotation);
    }

    public void swerveDriveFieldRelative(double direction, double speed, double rotation) {
        double fwd = Math.cos(direction) * speed;
        double str = Math.sin(direction) * speed;
        double heading = Math.toRadians(m_navx.getHeadingInfo().heading);
        double temp = (fwd * Math.cos(heading)) + (str * Math.sin(heading));
        str = (-fwd * Math.sin(heading)) + (str * Math.cos(heading));
        fwd = temp;
        swerveDriveComponents(fwd, str, rotation);
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

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Update the NavX heading
        m_navx.recomputeHeading(false);
    }
}

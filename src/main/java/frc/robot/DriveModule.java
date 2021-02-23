/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj.AnalogPotentiometer;

import javax.annotation.Nonnull;

import static org.a05annex.util.Utl.*;


/**
 * Class that contains and controls the motors and encoders for a single swerve drive module.
 */
public class DriveModule {

    // This is the physical hardware wired to the roborio
    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_spinMotor;
    private final AnalogPotentiometer m_analogEncoder;

    // The are components of the physical hardware
    private final CANEncoder m_driveEncoder;
    private final CANPIDController m_drivePID;
    private final CANEncoder m_spinEncoder;
    private final CANPIDController m_spinPID;

    // This is the initial 0.0 degree position calibration
    private final double m_calibrationOffset;
    /**
     * A multiplier for the speed that is either 1.0 (forward) or -1.0 (backwards) because the shortest
     * spin to the desired direction may be the backwards direction of the wheel, which requires the speed
     * to be reversed.
     */
    private double m_speedMultiplier = 1.0;
    /**
     * The last angle the wheel was set to, in radians. this may be either the front or the back of
     * the wheel - see {@link #m_speedMultiplier) documentation for determining whether this is the
     * orientation of the front or the back. This will be in the range -pi to pi.
     */
    private double m_lastRadians = 0.0;
    /**
     * The last spin encoder value that was set. Note, we always set the next spin by using a change angle.
     * This means the encoder setting can be anywhere from -infinity to +infinity.
     */
    private double m_lastEncoder = 0.0;
    /**
     * The last speed value that was set for this module, in the range 0.0 to 1.0.
     */
    private double m_lastSpeed = 0.0;
    private boolean m_driveBySpeed = true;

    /**
     * * The factory that creates the DriveModule given the
     *
     * @param driveCAN          CAN address for the motor that drives the wheel forward.
     * @param spinCAN           CAN address for the motor that spins the wheel around.
     * @param analogPort        Roborio analog port for the analog potentiometer, from 0 to 3. The analog
     *                          potentiometer tracks the absolute spin motor.
     * @param calibrationOffset The value of the analog potentiometer that will point the module forward.
     * @return (not null) Returns the initialized drive module.
     */
    public static DriveModule factory(int driveCAN, int spinCAN, int analogPort, double calibrationOffset) {
        // basic code representations for physical hardware
        CANSparkMax driveMotor = new CANSparkMax(driveCAN, MotorType.kBrushless);
        CANSparkMax spinMotor = new CANSparkMax(spinCAN, MotorType.kBrushless);
        AnalogPotentiometer analogEncoder = new AnalogPotentiometer(analogPort);
        // derived representations of components embedded in the physical hardware
        CANEncoder driveEncoder = driveMotor.getEncoder();
        CANPIDController drivePID = driveMotor.getPIDController();
        CANEncoder spinEncoder = spinMotor.getEncoder();
        CANPIDController spinPID = spinMotor.getPIDController();
        return new DriveModule(driveMotor, driveEncoder, drivePID,
                spinMotor, spinEncoder, spinPID,
                analogEncoder, calibrationOffset);
    }

    /**
     * Instantiate a DriveModule. All of instanced robot hardware control classes are passed in so this
     * module can be tested using the JUnit test framework.
     *
     * @param driveMotor (CANSparkMax, not null) The drive motor controller.
     * @param driveEncoder (CANEncoder, not null) The drive motor encoder.
     * @param drivePID (CANPIDController, not null) The drive motor PID controller.
     * @param spinMotor (CANSparkMax, not null) The spin motor controller.
     * @param spinEncoder (CANEncoder, not null) The spin motor encoder.
     * @param spinPID (CANPIDController, not null) The spin motor PID controller.
     * @param analogEncoder (AnalogPotentiometer, not null) The spin analog position encoder which provides
     *                      the absolute spin position of the module.
     * @param calibrationOffset The value of the analog potentiometer that will point the module forward.
     */
    public DriveModule(@Nonnull CANSparkMax driveMotor, CANEncoder driveEncoder, CANPIDController drivePID,
                       CANSparkMax spinMotor, CANEncoder spinEncoder, CANPIDController spinPID,
                       AnalogPotentiometer analogEncoder, double calibrationOffset) {

        m_driveMotor = driveMotor;
        m_driveEncoder = driveEncoder;
        m_drivePID = drivePID;
        m_spinMotor = spinMotor;
        m_spinEncoder = spinEncoder;
        m_spinPID = spinPID;
        m_analogEncoder = analogEncoder;

        // reset motor controllers to factory default
        m_driveMotor.restoreFactoryDefaults();
        m_spinMotor.restoreFactoryDefaults();

        // invert the spin so positive is a clockwise spin
        m_spinMotor.setInverted(true);

        // update PID controllers for spin and drive motors and initialize them
        initPID(m_drivePID, Constants.DRIVE_kFF, Constants.DRIVE_kP, Constants.DRIVE_kI, Constants.DRIVE_IZONE);
        initPID(m_spinPID, 0.0, Constants.SPIN_kP, Constants.SPIN_kI, 0.0);

        // calibrate
        m_calibrationOffset = calibrationOffset;
        calibrate(); // reset spin encoder to forward
        m_spinPID.setReference(0.0, ControlType.kPosition);
        m_lastRadians = 0.0;
        m_lastEncoder = 0.0;
    }

    /**
     * Updates the spin CANPIDController object using values in constants file. Used only when tuning the PID
     * constants for best control.
     */
    public void setSpinPID() {
        m_spinPID.setP(Constants.SPIN_kP);
        m_spinPID.setI(Constants.SPIN_kI);
    }

    /**
     * Updates the drive CANPIDController object using values in constants file. Used only when tuning the PID
     * * constants for best control.
     */
    public void setDrivePID() {
        m_drivePID.setP(Constants.DRIVE_kP);
        m_drivePID.setI(Constants.DRIVE_kI);
        m_drivePID.setFF(Constants.DRIVE_kFF);
        m_drivePID.setIZone(Constants.DRIVE_IZONE);
    }

    public void setDrivePosPID() {
        m_drivePID.setP(Constants.DRIVE_POS_kP);
        m_drivePID.setI(Constants.DRIVE_POS_kI);
        m_drivePID.setFF(0.0);
        m_drivePID.setIZone(0.0);
    }

    private void initPID(CANPIDController pid, double kFF, double kP, double kI, double kIZone) {
        pid.setFF(kFF);
        pid.setP(kP);
        pid.setI(kI);
        pid.setD(0.0);
        pid.setIZone(kIZone);
        pid.setOutputRange(-1.0, 1.0);
    }

    /**
     * Returns the drive motor velocity (RPM) as read from the encoder
     *
     * @return The drive motor velocity (RPM)
     */
    public double getDriveEncoderVelocity() {
        return m_driveEncoder.getVelocity();
    }

    /**
     * Returns the drive motor position as read from the encoder.
     *
     * @return The drive motor position as read from the encoder.
     */
    public double getDriveEncoderPosition() {
        return m_driveEncoder.getPosition();
    }

    /**
     * Returns the spin motor position as read from the encoder.
     *
     * @return The spin motor position as read from the encoder.
     */
    public double getSpinEncoderPosition() {
        return m_spinEncoder.getPosition();
    }

    /**
     * Returns the value of the analog encoder as a double. The value goes from 0.0 to 1.0, wrapping around
     * when the boundary between 0 and 2pi is reached. This method is provided primarily to read the
     * analog encoder to determine the calibrationOffset that should be used for initialization.
     *
     * @return The analog spin encoder position.
     */
    public double getAnalogEncoderPosition() {
        return m_analogEncoder.get();
    }

    /**
     * Returns the last speed that was set for this module in m/sec.
     *
     * @return The last speed that was set in m/sec.
     */
    public double getLastSpeed() {
        return m_lastSpeed * Constants.MAX_DRIVE_VELOCITY;
    }

    /**
     * Returns the last speed that was set for this module normalized to 0.0-1.0.
     *
     * @return the last normalized speed that was set.
     */
    public double getLastNormalizedSpeed() {
        return m_lastSpeed;
    }

    /**
     * Get the last direction set for the module.
     *
     * @return the last direction se (in radians)
     */
    public double getLastRadians() {
        return m_lastRadians;
    }

    /**
     * Set the NEO spin encoder value using the analog encoder, so that forward is an encoder
     * reading of 0 rotations.
     */
    public void calibrate() {
        // (actual - offset) * 360 / 20
        m_spinEncoder.setPosition((m_analogEncoder.get() - m_calibrationOffset) * 18.0);
    }

    /**
     * Set the direction and speed of the drive wheel in this module.
     *
     * @param targetDegrees (double) The direction from -180.0 to 180.0 degrees where 0.0 is towards the
     *                      front of the robot, and positive is clockwise.
     * @param speed         (double) The normalized speed of the wheel from 0.0 to 1.0 where 1.0 is the maximum
     *                      forward velocity.
     */
    public void setDegreesAndSpeed(double targetDegrees, double speed) {
        setRadiansAndSpeed(Math.toRadians(targetDegrees), speed);
    }

    /**
     * Set the module direction in radians. This code finds the closest forward-backward direction and sets the
     * foward-backaward multiplier for speed.
     *
     * @param targetRadians (double) The direction from -pi to pi radians where 0.0 is towards the
     *      *                      front of the robot, and positive is clockwise.
     */
    public void setRadians(double targetRadians) {
        // The real angle of the front of the wheel is 180 degrees away from the current angle if the wheel
        // is going backwards (i.e. the m_lastAngle was the last target angle
        double realLastForward = (m_speedMultiplier > 0.0) ? m_lastRadians :
                (m_lastRadians < 0.0) ? m_lastRadians + PI : m_lastRadians - PI;
        double deltaRadians = targetRadians - realLastForward;
        m_speedMultiplier = 1.0;

        // Since there is wrap-around at -180.0 and 180.0, it is easy to create cases where only a small correction
        // is required, but a very large deltaDegrees results because the spin is in the wrong direction. If the
        // angle is greater than 180 degrees in either direction, the spin is the wrong way. So the next section
        // checks that and changes the direction of the spin is the wrong way.
        if (deltaRadians > PI) {
            deltaRadians -= TWO_PI;
        } else if (deltaRadians < NEG_PI) {
            deltaRadians += TWO_PI;
        }

        // So, the next bit is looking at whether it better to spin the front of the wheel to the
        // target and drive forward, or, to spin the back of the wheel to the target direction and drive
        // backwards - if the spin is greater than 90 degrees (pi/2 radians) either direction, it is better
        // to spin the shorter angle and run backwards.
        if (deltaRadians > PI_OVER_2) {
            deltaRadians -= PI;
            m_speedMultiplier = -1.0;
        } else if (deltaRadians < NEG_PI_OVER_2) {
            deltaRadians += PI;
            m_speedMultiplier = -1.0;
        }

        // Compute and set the spin value
        m_lastRadians = targetRadians;
        m_lastEncoder += (deltaRadians * Constants.RADIANS_TO_SPIN_ENCODER);

        m_spinPID.setReference(m_lastEncoder, ControlType.kPosition);
    }

    /**
     * Set the direction and speed of the drive wheel in this module.
     *
     * @param targetRadians (double) The direction from -pi to pi radians where 0.0 is towards the
     *                      front of the robot, and positive is clockwise.
     * @param speed         (double) The normalized speed of the wheel from 0.0 to 1.0 where 1.0 is the maximum
     *                      forward velocity.
     */
    public void setRadiansAndSpeed(double targetRadians, double speed) {

        setRadians(targetRadians);

        // Compute and set the speed value
        m_lastSpeed = speed;
        speed *= Constants.MAX_DRIVE_VELOCITY * m_speedMultiplier;

        if (!m_driveBySpeed) {
            setDrivePID();
            m_driveBySpeed = true;
        }
        m_drivePID.setReference(speed, ControlType.kVelocity);
    }

    /**
     * Set the direction and distance in encoder tics that the module should move. We use this for targeting when we
     * the robot is stopped and we are trying to get very fast response and a very solid lock on the target. This is
     * far more reliable that trying to use a PID to control rotation speed to lock on target.
     *
     * @param targetRadians (double) The direction from -pi to pi radians where 0.0 is towards the
     *                      front of the robot, and positive is clockwise.
     * @param deltaTics (double) The number of tics the drive motor should mov e.
     */
    public void setRadiansAndDistance(double targetRadians, double deltaTics) {
        setRadians(targetRadians);
        double targetTics = getDriveEncoderPosition() + deltaTics * m_speedMultiplier;

        if (m_driveBySpeed) {
            m_drivePID.setReference(0, ControlType.kVelocity);
            setDrivePosPID();
            m_driveBySpeed = false;
        }
        m_drivePID.setReference(targetTics, ControlType.kPosition);
    }
}

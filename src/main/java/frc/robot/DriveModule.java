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

import static frc.robot.Utl.*;


/**
 * Class that contains the motors and encoders for a single swerve drive module.
 */
public class DriveModule {

    private static final double RADIANS_TO_TICS = 18.0 / TWO_PI;

    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_spinMotor;
    private final CANEncoder m_driveEncoder;
    private final CANEncoder m_spinEncoder;
    private final AnalogPotentiometer m_analogEncoder;
    private final double m_calibrationOffset;
    private final CANPIDController m_spinPID;
    private final CANPIDController m_drivePID;
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

    /**
     * Creates a new DriveModule. Ports should be for the same wheel.
     *
     * @param drivePort         Port for the motor that drives the wheel forward.
     * @param spinPort          Port for the motor that spins the wheel around.
     * @param analogPort        Port for the analog potentiometer, from 0 to 3. The analog potentiometer
     *                          tracks the spin motor.
     * @param calibrationOffset The value of the analog potentiometer that will point the module forward.
     */
    public DriveModule(int drivePort, int spinPort, int analogPort, double calibrationOffset) {
        m_driveMotor = new CANSparkMax(drivePort, MotorType.kBrushless);
        m_spinMotor = new CANSparkMax(spinPort, MotorType.kBrushless);

        // reset motor controllers to factory default
        m_driveMotor.restoreFactoryDefaults();
        m_spinMotor.restoreFactoryDefaults();
        m_spinMotor.setInverted(true);

        // Instantiate the motor encoders and analog encoders
        m_driveEncoder = m_driveMotor.getEncoder();
        m_spinEncoder = m_spinMotor.getEncoder();
        // m_spinEncoder.setInverted(true);
        m_analogEncoder = new AnalogPotentiometer(analogPort);


        // Create and update PID controllers for spin and drive motors and initialize them
        m_spinPID = m_spinMotor.getPIDController();
        initPID(m_spinPID, 0.0, Constants.SPIN_kP, Constants.SPIN_kI, 0.0);
        m_drivePID = m_driveMotor.getPIDController();
        initPID(m_drivePID, Constants.DRIVE_kFF, Constants.DRIVE_kP, Constants.DRIVE_kI, Constants.DRIVE_IZONE);

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

    private void initPID(CANPIDController pid, double kFF, double kP, double kI, double kIZone) {
        pid.setFF(kFF);
        pid.setP(kP);
        pid.setI(kI);
        pid.setD(0.0);
        pid.setIZone(kIZone);
        pid.setOutputRange(-1.0, 1.0);
        pid.setD(0.0);
    }
    /**
     * Returns the CANEncoder object for the drive motor.
     */
    public CANEncoder getDriveEncoder() {
        return m_driveEncoder;
    }

    /**
     * Returns the CANEncoder object for the spin motor.
     */
    public CANEncoder getSpinEncoder() {
        return m_spinEncoder;
    }

    /**
     * Returns the CANPIDController object for the spin motor.
     */
    public CANPIDController getSpinPID() {
        return m_spinPID;
    }

    /**
     * Returns the value of the analog encoder as a double. The value goes from 0.0 to 1.0, wrapping around
     * when the boundary between 0 and 2pi is reached. This method is provided primarily to read the the
     * analog encoder to determine the calibrationOffset that should be used for initialization.
     */
    public double getAnalogEncoder() {
        return m_analogEncoder.get();
    }

    public CANSparkMax getDriveMotor() {
        return m_driveMotor;
    }

    public CANSparkMax getSpinMotor() {
        return m_spinMotor;
    }

    /**
     * Set the NEO spin encoder value using the analog encoder, so that forward is 0 rotations.
     */
    public void calibrate() {
        // (offset - actual) * 360 / 20
        //m_spinEncoder.setPosition((m_calibrationOffset - m_analogEncoder.get()) * 18.0);
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
        setRadiansAndSpeed(Math.toRadians(targetDegrees),  speed);
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
        // m_lastEncoder -= (deltaRadians * RADIANS_TO_TICS);
        m_lastEncoder += (deltaRadians * RADIANS_TO_TICS);
        m_spinPID.setReference(m_lastEncoder, ControlType.kPosition);

        // Compute and set the speed value
        m_lastSpeed = speed * Constants.MAX_DRIVE_VELOCITY * m_speedMultiplier;
        m_drivePID.setReference(m_lastSpeed, ControlType.kVelocity);

        // remember the last spin direction and speed this module was set to.
        m_lastSpeed = speed;
        m_lastRadians = targetRadians;
    }
}

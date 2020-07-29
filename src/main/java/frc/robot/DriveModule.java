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

/**
 * Class that contains the motors and encoders for a single swerve drive module.
 */
public class DriveModule {

    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_spinMotor;
    private final CANEncoder m_driveEncoder;
    private final CANEncoder m_spinEncoder;
    private final AnalogPotentiometer m_analogEncoder;
    private final double m_calibrationOffset;
    private CANPIDController m_spinPID;
    private CANPIDController m_drivePID;
    private double m_mult = 1;
    private double m_lastAngle;
    private double m_lastTics;

    /**
     * Creates a new DriveModule. Ports should be for the same wheel.
     * 
     * @param drivePort Port for the motor that drives the wheel forward.
     * @param spinPort Port for the motor that spins the wheel around.
     * @param analogPort Port for the analog potentiometer, from 0 to 3. The analog potentiometer tracks the spin motor.
     * @param calibration_offset The value of the analog potentiometer that will point the module forward.
     */
    public DriveModule(int drivePort, int spinPort, int analogPort, double calibrationOffset) {
        m_driveMotor = new CANSparkMax(drivePort, MotorType.kBrushless);
        m_spinMotor = new CANSparkMax(spinPort, MotorType.kBrushless);

        // reset motors to factory default
        m_driveMotor.restoreFactoryDefaults();
        m_spinMotor.restoreFactoryDefaults();

        m_driveEncoder = m_driveMotor.getEncoder();
        m_spinEncoder = m_spinMotor.getEncoder();
        m_analogEncoder = new AnalogPotentiometer(analogPort);

        m_calibrationOffset = calibrationOffset;

        // Create and update PID contollers for spin and drive motors
        m_spinPID = m_spinMotor.getPIDController();
        setSpinPID();
        m_drivePID = m_driveMotor.getPIDController();
        setDrivePID();

        calibrate(); // reset spin encoder to forward
        m_spinPID.setReference(0, ControlType.kPosition);
        m_lastAngle = 0;
        m_lastTics = 0;
    }

    /**
     * Updates CANPIDContoller object using values in constants file.
     */
    public void setSpinPID() {
        m_spinPID.setP(Constants.SPIN_kP);
        m_spinPID.setI(Constants.SPIN_kI);
    }

    public void setDrivePID() {
        m_drivePID.setP(Constants.DRIVE_kP);
        m_drivePID.setI(Constants.DRIVE_kI);
        m_drivePID.setFF(Constants.DRIVE_kFF);
        m_drivePID.setOutputRange(Constants.DRIVE_kMIN, Constants.DRIVE_kMAX);
        m_drivePID.setD(0);
        m_drivePID.setIZone(0);
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
     * Returns the value of the analog encoder as a double.
     * The value goes from 0 to 1 when the wheel turns 360 degrees, then loops around.
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
        m_spinEncoder.setPosition((m_calibrationOffset - m_analogEncoder.get()) * 18);
    }

    /**
     * Set the PID controller of the module to a target angle.
     * @param targetAngle From -180 and 180 degrees.
     * @param speed From -1 to 1.
     */
    public void setAngleAndSpeed(double targetAngle, double speed) {
        double deltaAngle = targetAngle - m_lastAngle;
        if (deltaAngle > 180) {
            deltaAngle -= 360;
        } else if (deltaAngle < -180) {
            deltaAngle += 360;
        }
        double deltaTics = deltaAngle / 20;
        double newTics = m_lastTics - deltaTics;
        m_spinPID.setReference(newTics, ControlType.kPosition);
        m_lastTics = newTics;
        m_lastAngle = targetAngle;

        speed *= Constants.MAX_DRIVE_VELOCITY;
        m_drivePID.setReference(speed, ControlType.kVelocity);

        // TODO: implement m_mult and the 90 degree maximum wheel turn
    }
}

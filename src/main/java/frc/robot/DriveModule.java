/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
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

        // Create and update PID contoller for spin motor
        m_spinPID = m_spinMotor.getPIDController();
        setSpinPID();

        calibrate(); // reset spin encoder to forward
    }

    /**
     * Updates CANPIDContoller object using values in constants file.
     */
    public void setSpinPID() {
        m_spinPID.setP(Constants.SPIN_kP);
        m_spinPID.setI(Constants.SPIN_kI);
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

    public void setAngleAndSpeed(float angle, float speed) {
        // TODO: write me
        return;
    }
}

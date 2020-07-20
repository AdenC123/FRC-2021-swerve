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

        m_driveEncoder = new CANEncoder(m_driveMotor);
        m_spinEncoder = new CANEncoder(m_spinMotor);
        m_analogEncoder = new AnalogPotentiometer(analogPort);

        m_calibrationOffset = calibrationOffset;
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
     * Set the wheel angle to straight and reset the NEO spin encoder.
     */
    public void calibrate() {

    }

    public void setAngleAndSpeed(float angle, float speed) {
        // TODO: write me
        return;
    }
}

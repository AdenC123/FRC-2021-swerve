/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DriveModule;

public class DriveSubsystem extends SubsystemBase {

  // create drive modules
  private final DriveModule m_rf;

  private final DriveModule m_rr;

  private final DriveModule m_lf;

  private final DriveModule m_lr;
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

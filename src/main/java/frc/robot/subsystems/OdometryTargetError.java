/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.a05annex.util.Utl;

public class OdometryTargetError extends SubsystemBase implements IGetTargetError{

  private DriveSubsystem m_driveSubsystem;
  private double m_targetX = 0.0;
  private double m_targetY = 3.0;

  /**
   * Creates a new OdometryTargetError.
   */
  public OdometryTargetError(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
  }

  public double GetTargetHeadingError() {
    double fieldX = m_driveSubsystem.getFieldX();
    double fieldY = m_driveSubsystem.getFieldY();
    double fieldHeading = m_driveSubsystem.getFieldHeading();
    double angleToTargetFromZero = Math.atan2(m_targetX - fieldX, m_targetY - fieldY);
    fieldHeading %= Utl.PI;
    return angleToTargetFromZero - fieldHeading;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

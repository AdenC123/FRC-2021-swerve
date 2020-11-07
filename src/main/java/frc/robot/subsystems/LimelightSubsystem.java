/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utl;
import edu.wpi.first.networktables.NetworkTable; 
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase implements IGetTargetError{

    double m_x;
    double m_y;
    double m_a;

  /**
   * Creates a new LimelightSubsystem.
   */
  public LimelightSubsystem() {
      updateData();
  }

  private void updateData() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    m_x = tx.getDouble(0.0);
    m_y = ty.getDouble(0.0);
    m_a = ta.getDouble(0.0);
  }

  public double getX() {
      return m_x;
  }

  public double getY() {
      return m_y;
  }

  public double getA() {
      return m_a;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateData();
  }

  public double GetTargetHeadingError() {
    return m_x * (Utl.PI/180);
  }
}
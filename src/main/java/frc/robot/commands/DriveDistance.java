/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistance extends CommandBase {

  private double m_speed;
  private final DriveSubsystem m_driveSubsystem;
  private final double m_distance;
  private final double m_direction;
  private long m_targetTime;

  /**
   * Creates a new DriveDistance.
   */
  public DriveDistance(double distance, double direction, DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    m_distance = distance;
    m_direction = direction;
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_speed = Constants.DRIVE_SPEED;
    m_targetTime = (long) ((m_distance * 1000.0) / (Constants.MAX_METERS_PER_SEC * m_speed)) + System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.swerveDriveFieldRelative(m_direction, m_speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (System.currentTimeMillis() < m_targetTime) {
      return false;
    } else {
      return true;
    }
  }
}

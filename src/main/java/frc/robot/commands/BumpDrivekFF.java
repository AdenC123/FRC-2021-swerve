/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class BumpDrivekFF extends CommandBase {

  private final double m_inc;

  /**
   * Creates a new BumpDrivekFF.
   */
  public BumpDrivekFF(double inc) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_inc = inc;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Constants.DRIVE_kFF + m_inc >= 0) {
      Constants.DRIVE_kFF += m_inc;
    }
    RobotContainer.m_activeModule.setDrivePID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

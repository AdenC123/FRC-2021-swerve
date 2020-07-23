/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class SpinTest extends CommandBase {

  private final DriveSubsystem m_driveSubsystem;
  private final Joystick m_stick;

  /**
   * Angles the active module to the same angle as the joystick.
   */
  public SpinTest(Joystick stick, DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    m_stick = stick;

    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double stickY = -m_stick.getY();
    double stickX = m_stick.getX();
    if (stickY*stickY + stickX*stickX > 0.1) {
      // calculate stick angle
      double stickAngle = Math.atan2(stickX, stickY);
      stickAngle = Math.toDegrees(stickAngle);
      RobotContainer.m_activeModule.setAngleAndSpeed(stickAngle, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

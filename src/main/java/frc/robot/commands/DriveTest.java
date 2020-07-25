/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTest extends CommandBase {

  private final Joystick m_stick;
  private final DriveSubsystem m_driveSubsystem;

  /**
   * Sets the speed of the active module using setAngleAndSpeed.
   * Sets speed to the Y of the joystick.
   */
  public DriveTest(Joystick stick, DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_stick = stick;
    m_driveSubsystem = driveSubsystem;
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
    if (m_stick.getRawButton(1)) {
      RobotContainer.m_activeModule.setAngleAndSpeed(0, 0.5);
    } else if (stickY*stickY + stickX*stickX > 0.1) {
      RobotContainer.m_activeModule.setAngleAndSpeed(0, stickY);
    } else {
      RobotContainer.m_activeModule.setAngleAndSpeed(0, 0);
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

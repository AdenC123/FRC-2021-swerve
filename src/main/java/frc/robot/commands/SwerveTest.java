/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveTest extends CommandBase {

  private final XboxController m_xbox;
  private final DriveSubsystem m_driveSubsystem;

  /**
   * Command to test out the swerve modules by setting drive motors to stick y and spin motors to stick x.
   * 
   * @param xbox The xbox controller.
   * @param driveSubsystem The drive subsystem.
   */
  public SwerveTest(XboxController xbox, DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_xbox = xbox;
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
    RobotContainer.m_activeModule.getDriveMotor().set(-m_xbox.getY());
    RobotContainer.m_activeModule.getSpinMotor().set(m_xbox.getX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Setting speed to 0 is not needed because the set method of Sparks must be called constantly
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
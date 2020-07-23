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
  private double m_stickAngle;

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
    // calculate stick angle
    m_stickAngle = Math.atan2(m_stick.getY(), m_stick.getX()); // y may need to be negative?
    // convert to rotations
    double degrees = Math.toDegrees(m_stickAngle);
    double rotations = degrees / 20;
    // set wheel angle
    RobotContainer.m_activeModule.getSpinPID().setReference(rotations, ControlType.kPosition);
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

  public double getStickAngle() {
    return m_stickAngle;
  }
}

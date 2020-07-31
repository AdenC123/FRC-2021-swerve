/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {

  private final Joystick m_stick;
  private final DriveSubsystem m_driveSubsystem;

  /**
   * Drive using a single joystick, with the y being forward, the x being strafe, and the twist being rotate.
   */
  public DriveCommand(Joystick stick, DriveSubsystem driveSubsystem) {
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
    // get stick values
    double stickY = -m_stick.getY();
    double stickX = m_stick.getX();
    double stickTwist = m_stick.getTwist();
    // do deadband on speed
    double distance = Math.sqrt(stickY*stickY + stickX*stickX);
    double speed;
    if (distance < Constants.DRIVE_DEADBAND) {
      speed = 0;
    } else {
      speed = (distance - Constants.DRIVE_DEADBAND) / (1 - Constants.DRIVE_DEADBAND);
    }
    // add gain and sensitivity
    speed = Math.pow(speed, Constants.DRIVE_SPEED_SENSITIVTY) * Constants.DRIVE_SPEED_GAIN;
    // do deadband on rotation
    double rotation;
    if (Math.abs(stickTwist) < Constants.DRIVE_DEADBAND) {
      rotation = 0;
    } else if (stickTwist > 0) {
      rotation = (stickTwist - Constants.DRIVE_DEADBAND) / (1 - Constants.DRIVE_DEADBAND);
    } else {
      rotation = (stickTwist + Constants.DRIVE_DEADBAND) / (1 - Constants.DRIVE_DEADBAND);
    }
    // find direction, if the speed is 0 then it won't rotate
    double direction = Math.atan2(stickX, stickY);
    m_driveSubsystem.swerveDrive(direction, speed, rotation);
    
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

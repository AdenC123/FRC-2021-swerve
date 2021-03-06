/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IGetTargetError;
import org.a05annex.util.Utl;

public class DriveCommandXbox extends CommandBase {

  private final XboxController m_xbox;
  private final DriveSubsystem m_driveSubsystem;
  private final IGetTargetError m_getTargetError;

  /**
   * Drive using an xbox controller, with left stick Y being forward, left stick X being strafe,
   * and right stick X being rotate.
   */
  public DriveCommandXbox(XboxController xbox, DriveSubsystem driveSubsystem, IGetTargetError getTargetError) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_xbox = xbox;
    m_driveSubsystem = driveSubsystem;
    addRequirements(m_driveSubsystem);
    m_getTargetError = getTargetError;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get stick values
    double stickY = -m_xbox.getY(GenericHID.Hand.kLeft);
    double stickX = m_xbox.getX(GenericHID.Hand.kLeft);
    double stickTwist = m_xbox.getX(GenericHID.Hand.kRight);
    // do deadband on speed
    double distance = Utl.length(stickY,stickX);
    double speed;
    if (distance < Constants.DRIVE_DEADBAND) {
      speed = 0.0;
    } else {
      if (distance > 1.0) {
        distance = 1.0;
      }
      speed = (distance - Constants.DRIVE_DEADBAND) / (1.0 - Constants.DRIVE_DEADBAND);
    }
    // add gain and sensitivity
    speed = Math.pow(speed, Constants.DRIVE_SPEED_SENSITIVITY) * Constants.DRIVE_SPEED_GAIN;

    // either do rotation with right stick, or PID to target
    double rotation;
    if (m_xbox.getRawButton(5)) {
      // OK, the driver says key on the target, so we ignore the stick twist and use the
      // targeting heading to control the rotation.
      double targetHeadingError = m_getTargetError.GetTargetHeadingError();
      if (speed == 0) {
        m_driveSubsystem.setHeading(m_driveSubsystem.getFieldHeading() + targetHeadingError);
        return;
      } else {
        rotation = targetHeadingError * Constants.TARGET_kP;
      }
    } else {
      // Use the stick forward, strafe, twist as specified by the driver. NOTE: if the driver does
      // not specify a twist, then this command should be applying a rotation correction to maintain
      // the current heading.
      double rotMult = (stickTwist < 0.0) ? -1.0 : 1.0;
      stickTwist = Math.abs(stickTwist);
      // do deadband on rotation
      if (stickTwist < Constants.TWIST_DEADBAND) {
        // TODO - OK, no twist is specified - we should be trying to maintain the expected heading
        rotation = 0.0;
      } else {
        // TODO - OK, twist is specified, we should be updating the expected heading to match the
        // TODO - current robot heading.
        rotation = (stickTwist - Constants.TWIST_DEADBAND) / (1.0 - Constants.TWIST_DEADBAND);
      }
      // add sensitivity, gain and sign
      rotation = Math.pow(rotation, Constants.TWIST_SENSITIVITY) * Constants.TWIST_GAIN * rotMult;
    }
    // find direction, if the speed is 0 then it won't rotate
    double direction = Math.atan2(stickX, stickY);
    m_driveSubsystem.swerveDriveFieldRelative(direction, speed, rotation);
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

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {

  private final XboxController m_xbox;
  private final DriveSubsystem m_driveSubsystem;

  /**
   * Creates a new DriveCommand.
   */
  public DriveCommand(XboxController xbox, DriveSubsystem driveSubsystem) {
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
    // find forward, strafe right, and rotate clockwise from joystick inputs
    double fwd = -m_xbox.getY(Hand.kLeft);
    double str = m_xbox.getX(Hand.kLeft);
    double rcw = m_xbox.getX(Hand.kRight);

    // length, width and diagonal
    double length = Constants.DRIVE_LENGTH;
    double width = Constants.DRIVE_WIDTH;
    double diagonal = Math.sqrt(length*length + width*width);

    // calculate a, b, c and d variables
    double a = str - rcw * (length/diagonal);
    double b = str + rcw * (length/diagonal);
    double c = fwd - rcw * (width/diagonal);
    double d = fwd + rcw * (width/diagonal);

    // calculate wheel speeds and angles
    double frSpeed = Math.sqrt(b*b + c*c);
    double flSpeed = Math.sqrt(b*b + d*d);
    double rlSpeed = Math.sqrt(a*a + d*d);
    double rrSpeed = Math.sqrt(a*a + c*c);

    double frAngle = Math.atan2(b, c) * (180/Math.PI);
    double flAngle = Math.atan2(b, d) * (180/Math.PI);
    double rlAngle = Math.atan2(a, d) * (180/Math.PI);
    double rrAngle = Math.atan2(a, c) * (180/Math.PI);

    // normalize speeds TODO: make this not look like shit
    double max=frSpeed; if(flSpeed>max)max=flSpeed; if(rlSpeed>max)max=rlSpeed; if(rrSpeed>max)max=rrSpeed; // oh god why
    if (max > 1) {frSpeed /= max; flSpeed /= max; rlSpeed /= max; rrSpeed /= max;}

    // run wheels at speeds and angles TODO

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

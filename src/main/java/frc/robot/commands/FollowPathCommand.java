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
import org.a05annex.util.geo2d.KochanekBartelsSpline;
import org.a05annex.util.geo2d.KochanekBartelsSpline.PathFollower;
import org.a05annex.util.geo2d.KochanekBartelsSpline.PathPoint;

public class FollowPathCommand extends CommandBase {

  private DriveSubsystem m_driveSubsystem;
  private final KochanekBartelsSpline m_spline = new KochanekBartelsSpline();
  private PathFollower m_pathFollower;
  private boolean m_isFinished = false;
  private long m_startTime;

  /**
   * Creates a new FollowPathCommand.
   */
  public FollowPathCommand(String pathName, DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    addRequirements(m_driveSubsystem);

    m_spline.loadPath(pathName);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
    m_pathFollower = m_spline.getPathFollower();
    m_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = (System.currentTimeMillis() - m_startTime) / 1000.0;
    PathPoint point = m_pathFollower.getPointAt(currentTime);
    if (point == null) {
      m_isFinished = true;
      m_driveSubsystem.swerveDriveComponents(0, 0, 0);
    } else {
      // TODO - The expected heading is included in the PathPoint. The path point is the instantaneous
      // TODO - speed and position that we want to be at NOW. If the heading is incorrect, then the
      // TODO - direction the forward and strafe is incorrect and we will be at the wrong place on
      // TODO - the field. So we need a PID correction of heading incorporated here.
      double forward = point.speedForward / Constants.MAX_METERS_PER_SEC;
      double strafe = point.speedStrafe / Constants.MAX_METERS_PER_SEC;
      double rotation = point.speedRotation / Constants.MAX_RADIANS_PER_SEC;
      m_driveSubsystem.swerveDriveComponents(forward, strafe, rotation);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}

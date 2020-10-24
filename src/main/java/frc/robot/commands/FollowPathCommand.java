/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc6831.lib2d.KochanekBartelsSpline;
import frc6831.lib2d.KochanekBartelsSpline.PathFollower;
import frc6831.lib2d.KochanekBartelsSpline.PathPoint;

public class FollowPathCommand extends CommandBase {

  private DriveSubsystem m_driveSubsystem;
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

    KochanekBartelsSpline spline = new KochanekBartelsSpline();
    spline.loadPath(pathName);
    m_pathFollower = spline.getPathFollower();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    long currentTime = (System.currentTimeMillis() - m_startTime) / 1000;
    PathPoint point = m_pathFollower.getPointAt(currentTime);
    if (point == null) {
      m_isFinished = true;
      m_driveSubsystem.swerveDriveComponents(0, 0, 0);
    } else {
      m_driveSubsystem.swerveDriveComponents(point.speedForward, point.speedStrafe, point.speedRotation);
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

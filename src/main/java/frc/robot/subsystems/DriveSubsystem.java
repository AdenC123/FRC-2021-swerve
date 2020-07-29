/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DriveModule;

public class DriveSubsystem extends SubsystemBase {

  // create drive modules
  private final DriveModule m_rf;

  private final DriveModule m_rr;

  private final DriveModule m_lf;

  private final DriveModule m_lr;
  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    // Initialization Here
    m_rf = new DriveModule(Constants.MotorControllers.RF_DRIVE, Constants.MotorControllers.RF_SPIN,
    Constants.AnalogPorts.RF, Constants.CalibrationOffset.RF);

    m_rr = new DriveModule(Constants.MotorControllers.RR_DRIVE, Constants.MotorControllers.RR_SPIN,
    Constants.AnalogPorts.RR, Constants.CalibrationOffset.RR);

    m_lf = new DriveModule(Constants.MotorControllers.LF_DRIVE, Constants.MotorControllers.LF_SPIN,
    Constants.AnalogPorts.LF, Constants.CalibrationOffset.LF);

    m_lr = new DriveModule(Constants.MotorControllers.LR_DRIVE, Constants.MotorControllers.LR_SPIN,
    Constants.AnalogPorts.LR, Constants.CalibrationOffset.LR);
  }

  public DriveModule getRFModule() {
    return m_rf;
  }

  public DriveModule getRRModule() {
    return m_rr;
  }

  public DriveModule getLFModule() {
    return m_lf;
  }

  public DriveModule getLRModule() {
    return m_lr;
  }

  /**
   * Drives the robot using all 4 modules.
   * @param fwd Drive forward. From -1 to 1.
   * @param str Strafe right. From -1 to 1.
   * @param rcw Clockwise rotation. From -1 to 1.
   */
  public void swerveDrive(double fwd, double str, double rcw) {
    
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
    double rfSpeed = Math.sqrt(b*b + c*c);
    double lfSpeed = Math.sqrt(b*b + d*d);
    double lrSpeed = Math.sqrt(a*a + d*d);
    double rrSpeed = Math.sqrt(a*a + c*c);

    double rfAngle = Math.atan2(b, c) * (180/Math.PI);
    double lfAngle = Math.atan2(b, d) * (180/Math.PI);
    double lrAngle = Math.atan2(a, d) * (180/Math.PI);
    double rrAngle = Math.atan2(a, c) * (180/Math.PI);

    // normalize speeds TODO: make this not look like shit
    double max=rfSpeed; if(lfSpeed>max)max=lfSpeed; if(lrSpeed>max)max=lrSpeed; if(rrSpeed>max)max=rrSpeed;
    if (max > 1) {rfSpeed /= max; lfSpeed /= max; lrSpeed /= max; rrSpeed /= max;}

    // run wheels at speeds and angles
    m_rf.setAngleAndSpeed(rfAngle, rfSpeed);
    m_lf.setAngleAndSpeed(lfAngle, lfSpeed);
    m_lr.setAngleAndSpeed(lrAngle, lrSpeed);
    m_rr.setAngleAndSpeed(rrAngle, rrSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

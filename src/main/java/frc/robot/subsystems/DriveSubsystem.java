/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  // create motors
  private CANSparkMax m_rf_drive = new CANSparkMax(Constants.MotorControllers.RF_DRIVE, MotorType.kBrushless);
  private CANSparkMax m_rf_spin = new CANSparkMax(Constants.MotorControllers.RF_SPIN, MotorType.kBrushless);
  private CANSparkMax m_rr_drive = new CANSparkMax(Constants.MotorControllers.RR_DRIVE, MotorType.kBrushless);
  private CANSparkMax m_rr_spin = new CANSparkMax(Constants.MotorControllers.RR_SPIN, MotorType.kBrushless);
  private CANSparkMax m_lr_drive = new CANSparkMax(Constants.MotorControllers.LR_DRIVE, MotorType.kBrushless);
  private CANSparkMax m_lr_spin = new CANSparkMax(Constants.MotorControllers.LR_SPIN, MotorType.kBrushless);
  private CANSparkMax m_lf_drive = new CANSparkMax(Constants.MotorControllers.LF_DRIVE, MotorType.kBrushless);
  private CANSparkMax m_lf_spin = new CANSparkMax(Constants.MotorControllers.LF_SPIN, MotorType.kBrushless);

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {

    // restore all motors to factory default
    m_rf_drive.restoreFactoryDefaults();
    m_rf_spin.restoreFactoryDefaults();
    m_rr_drive.restoreFactoryDefaults();
    m_rr_spin.restoreFactoryDefaults();
    m_lr_drive.restoreFactoryDefaults();
    m_lr_spin.restoreFactoryDefaults();
    m_lf_drive.restoreFactoryDefaults();
    m_lf_spin.restoreFactoryDefaults();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Spin a module to a degree value.
   * 
   * @param module (String) 'RF', 'RR', 'LR', or 'LF'. TODO: find a better way to do this
   * @param degrees (double) Degree value to spin to, between -180 and 180 degrees with 0 being straight forward.
   */
  public void setAngle(String module, double degrees) {
    // TODO: this will require the analog encoders and PID
    // run at full power (or gain value) towards correct angle
  }

  /**
   * Set a module to a speed.
   * 
   * @param module (String) 'RF', 'RR', 'LR', or 'LF'. TODO: find a better way to do this
   * @param speed (double) Speed to run at, from -1 to 1.
   */
  public void setSpeed(String module, double speed) {

    // really hacky way to set the speed of a module
    if (module == "RF") {
      m_rf_drive.set(speed);
    } else if (module == "RR") {
      m_rr_drive.set(speed);
    } else if (module == "LR") {
      m_lr_drive.set(speed);
    } else if (module == "LF") {
      m_lf_drive.set(speed);
    } else {
      System.out.println("incorrect module name");
    }
  }

  /**
   * Set speeds of all drive motors, in the order of RF, LF, LR and RR.
   * Speed is from -1 to 1.
   */
  public void setSpeeds(double rfSpeed, double lfSpeed, double lrSpeed, double rrSpeed) {
    m_rf_drive.set(rfSpeed);
    m_lf_drive.set(lfSpeed);
    m_lr_drive.set(lrSpeed);
    m_rr_drive.set(rrSpeed);
  }

  /**
   * Set angles of all drive motors, in the order of RF, LF, LR and RR.
   * Angle is between -180 and 180 degrees.
   */
  public void setAngles(double rfAngle, double lfAngle, double lrAngle, double rrAngle) {
    // TODO: this is a derivative of setAngle, which isn't made yet
  }

  /**
   * Used in SwerveTest. Sets all the drive motors to a speed.
   * 
   * @param speed Speed from -1 to 1.
   */
  public void setAllDriveSpeed(double speed) {
    m_rf_drive.set(speed);
    m_rr_drive.set(speed);
    m_lf_drive.set(speed);
    m_lr_drive.set(speed);
  }

  /**
   * Used in SwerveTest. Sets all the spin motors to a speed.
   * 
   * @param speed Speed from -1 to 1.
   */
  public void setAllSpinSpeed(double speed) {
    m_rf_spin.set(speed);
    m_rr_spin.set(speed);
    m_lf_spin.set(speed);
    m_lr_spin.set(speed);
  }
}

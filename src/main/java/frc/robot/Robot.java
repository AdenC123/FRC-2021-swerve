/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  // Telemetry variables
  private double m_lastAnalogEncoder = -1.0;
  private double m_lastSpinEncoder = -1.0;
  private double m_lastDriveEncoder = -1.0;
  private double m_lastSpinVelocity = -1.0;
  private double m_lastDriveVelocity = -1.0;
  private double m_lastSpinkP = -1.0;
  private double m_lastSpinkI = -1.0;
  private double m_lastDrivekP = -1.0;
  private double m_lastDrivekI = -1.0;
  private double m_lastDrivekFF = -1.0;

  /**
   * Update telemetry feedback for a real number value. If the value has not changed, no update is sent
   *
   * @param port      (int) The port 0 - 9 to write to.
   * @param key       (String) The key for the telemetry.
   * @param var       (double) The number to be reported.
   * @param lastValue (double) The last value reported.
   * @return (double) Returns {@code var}
   */
  @SuppressWarnings("unused")
  private double dashboardTelemetry(int port, String key, double var, double lastValue) {
    if (var != lastValue) {
      SmartDashboard.putString(String.format("DB/String %d", port), String.format("%s: %10.6f", key, var));
    }
    return var;
  }

  /**
   * Update telemetry feedback for an integer value. If the value has not changed, no update is sent
   *
   * @param port      (int) The port 0 - 9 to write to.
   * @param key       (String) The key for the telemetry.
   * @param var       (int) The integer to be reported.
   * @param lastValue (int) The last value reported.
   * @return (int) Returns {@code var}
   */
  @SuppressWarnings("unused")
  private int dashboardTelemetry(int port, String key, int var, int lastValue) {
    if (var != lastValue) {
      SmartDashboard.putString(String.format("DB/String %d", port), String.format("%s: %d", key, var));
    }
    return var;
  }

  /**
   * Update telemetry feedback for a string value. If the value has not changed, no update is sent
   *
   * @param port      (int) The port 0 - 9 to write to.
   * @param key       (String) The key for the telemetry.
   * @param var       (String) The string to be reported.
   * @param lastValue (String) The last value reported.
   * @return (String) Returns {@code var}
   */
  @SuppressWarnings("unused")
  private String dashboardTelemetry(int port, String key, String var, String lastValue) {
    if (!var.equals(lastValue)) {
      SmartDashboard.putString(String.format("DB/String %d", port), String.format("%s: %s", key, var));
    }
    return var;
  }

  /**
   * Update telemetry feedback for a boolean value. If the value has not changed, no update is sent
   *
   * @param port      (int) The port 0 - 9 to write to.
   * @param key       (String) The key for the telemetry.
   * @param var       (boolean) The boolean to be reported.
   * @param lastValue (boolean) The last value reported.
   * @return (boolean) Returns {@code var}
   */
  @SuppressWarnings("unused")
  private boolean dashboardTelemetry(int port, String key, boolean var, boolean lastValue) {
    if (var != lastValue) {
      SmartDashboard.putString(String.format("DB/String %d", port),
          String.format("%s: %s", key, var ? "on" : "off"));
    }
    return var;
  }

  private void displayTelemetry() {
    m_lastAnalogEncoder = dashboardTelemetry(0, "Analog Enc",
      RobotContainer.m_activeModule.getAnalogEncoder(), m_lastAnalogEncoder);
    m_lastSpinEncoder = dashboardTelemetry(1, "Spin Enc",
      RobotContainer.m_activeModule.getSpinEncoder().getPosition(), m_lastSpinEncoder);
    m_lastDriveEncoder = dashboardTelemetry(2, "Drive Enc",
      RobotContainer.m_activeModule.getDriveEncoder().getPosition(), m_lastDriveEncoder);
    m_lastSpinVelocity = dashboardTelemetry(3, "Spin Vel",
      RobotContainer.m_activeModule.getSpinEncoder().getVelocity(), m_lastSpinVelocity);
    m_lastDriveVelocity = dashboardTelemetry(4, "Drive Vel",
      RobotContainer.m_activeModule.getDriveEncoder().getVelocity(), m_lastDriveVelocity);
    
    m_lastDrivekP = dashboardTelemetry(5, "Drive kP", Constants.DRIVE_kP, m_lastDrivekP);
    m_lastDrivekI = dashboardTelemetry(6, "Drive kI", Constants.DRIVE_kI, m_lastDrivekI);
    m_lastDrivekFF = dashboardTelemetry(7, "Drive kFF", Constants.DRIVE_kFF, m_lastDrivekFF);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // empty the telemetry display
    for (int i = 0; i < 10; i++) {
      SmartDashboard.putString(String.format("DB/String %d", i), " ");
    }

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    displayTelemetry();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}

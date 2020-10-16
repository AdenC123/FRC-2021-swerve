/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.DriveCommandXbox;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OdometryTargetError;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final DriveSubsystem m_driveSubsystem;
  //private final DriveCommand m_driveCommand;
  private final DriveCommandXbox m_driveCommandXbox;
  private final OdometryTargetError m_odometryTargetError;

  // controllers
  private final XboxController m_xbox = new XboxController(0);
  private final Joystick m_stick = new Joystick(1);

  // buttons
  private final POVButton m_xboxDpadUp = new POVButton(m_xbox, 0);
  private final POVButton m_xboxDpadLeft = new POVButton(m_xbox, 270);
  private final POVButton m_xboxDpadDown = new POVButton(m_xbox, 180);
  private final POVButton m_xboxDpadRight = new POVButton(m_xbox, 90);
  private final JoystickButton m_xboxA = new JoystickButton(m_xbox, 1);
  private final JoystickButton m_xboxB = new JoystickButton(m_xbox, 2);
  private final JoystickButton m_xboxX = new JoystickButton(m_xbox, 3);
  private final JoystickButton m_xboxY = new JoystickButton(m_xbox, 4);
  private final JoystickButton m_xboxLeftBumper = new JoystickButton(m_xbox, 5); // used for pointing at target
  private final JoystickButton m_xboxRightBumper = new JoystickButton(m_xbox, 6);

  private final JoystickButton m_button3 = new JoystickButton(m_stick, 3);
  private final JoystickButton m_button4 = new JoystickButton(m_stick, 4);
  private final JoystickButton m_button5 = new JoystickButton(m_stick, 5);
  private final JoystickButton m_button6 = new JoystickButton(m_stick, 6);
  private final JoystickButton m_button7 = new JoystickButton(this.m_stick, 7);
  private final JoystickButton m_button8 = new JoystickButton(this.m_stick, 8);
  private final JoystickButton m_button9 = new JoystickButton(this.m_stick, 9);
  private final JoystickButton m_button10 = new JoystickButton(this.m_stick, 10);
  private final JoystickButton m_button11 = new JoystickButton(this.m_stick, 11);
  private final JoystickButton m_button12 = new JoystickButton(this.m_stick, 12);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // subsystems
    m_driveSubsystem = new DriveSubsystem();
    m_odometryTargetError = new OdometryTargetError(m_driveSubsystem);

    // commands
    //m_driveCommand = new DriveCommand(m_stick, m_driveSubsystem);
    m_driveCommandXbox = new DriveCommandXbox(m_xbox, m_driveSubsystem, m_odometryTargetError);

    // set default commands
    //m_driveSubsystem.setDefaultCommand(m_driveCommand);
    m_driveSubsystem.setDefaultCommand(m_driveCommandXbox);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  public XboxController getXbox() {
    return m_xbox;
  }

  public Joystick getStick() {
    return m_stick;
  }

  public DriveSubsystem getDriveSubsystem() {
    return m_driveSubsystem;
  }
}

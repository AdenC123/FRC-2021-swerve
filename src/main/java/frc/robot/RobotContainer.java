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
import frc.robot.commands.SwerveTest;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.BumpDrivekFF;
import frc.robot.commands.BumpDrivekI;
import frc.robot.commands.BumpDrivekP;
import frc.robot.commands.BumpSpeed;
import frc.robot.commands.BumpSpinkI;
import frc.robot.commands.BumpSpinkP;
import frc.robot.commands.DriveTest;
import frc.robot.commands.SetActiveModule;
import frc.robot.commands.SetSpeed;
import frc.robot.commands.SpinTest;
import frc.robot.commands.SpinToDegrees;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final DriveSubsystem m_driveSubsystem;
  //private final SwerveTest m_swerveTest;
  //private final SpinTest m_spinTest;
  private final DriveTest m_driveTest;
  private final SetSpeed m_setSpeed;

  public static DriveModule m_activeModule;
  public static double m_speed = 0.0;

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
    m_activeModule = m_driveSubsystem.getLFModule();

    // commands
    //m_swerveTest = new SwerveTest(m_xbox, m_driveSubsystem);
    //m_spinTest = new SpinTest(m_stick, m_driveSubsystem);
    m_driveTest = new DriveTest(m_stick, m_driveSubsystem);
    m_setSpeed = new SetSpeed(m_driveSubsystem);

    // set default commands
    m_driveSubsystem.setDefaultCommand(m_driveTest);

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

    // set active module with dpad
    m_xboxDpadRight.whenPressed(new SetActiveModule(m_driveSubsystem.getRRModule()));
    m_xboxDpadUp.whenPressed(new SetActiveModule(m_driveSubsystem.getRFModule()));
    m_xboxDpadLeft.whenPressed(new SetActiveModule(m_driveSubsystem.getLFModule()));
    m_xboxDpadDown.whenPressed(new SetActiveModule(m_driveSubsystem.getLRModule()));

    // adjust kP and kI with top buttons
    /*
    m_button3.whenPressed(new BumpSpinkP(-0.01));
    m_button5.whenPressed(new BumpSpinkP(0.01));
    m_button4.whenPressed(new BumpSpinkI(-0.001));
    m_button6.whenPressed(new BumpSpinkI(0.001));
    */
    m_button7.whenPressed(new BumpDrivekP(-0.00001));
    m_button8.whenPressed(new BumpDrivekP(0.00001));
    m_button9.whenPressed(new BumpDrivekI(-0.000001));
    m_button10.whenPressed(new BumpDrivekI(0.000001));
    m_button11.whenPressed(new BumpDrivekFF(-0.000001));
    m_button12.whenPressed(new BumpDrivekFF(0.000001));
    
    // change speed with top buttons
    m_button3.whenPressed(new BumpSpeed(-0.1));
    m_button5.whenPressed(new BumpSpeed(0.1));

    // set degrees with xbox buttons
    /*
    m_xboxY.whenPressed(new SpinToDegrees(0, m_driveSubsystem));
    m_xboxB.whenPressed(new SpinToDegrees(90, m_driveSubsystem));
    m_xboxX.whenPressed(new SpinToDegrees(-90, m_driveSubsystem));
    m_xboxA.whenPressed(new SpinToDegrees(-180, m_driveSubsystem));
    */
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
}

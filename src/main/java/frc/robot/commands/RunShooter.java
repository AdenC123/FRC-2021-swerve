package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;


public class RunShooter extends CommandBase {

    private ShooterSubsystem m_shooterSubsystem;
    private double m_power;

    /**
     * Runs both shooter motors at the given power. When interrupted, set both power to 0.
     * @param shooterSubsystem The shooter subsystem.
     * @param power (double) The power to run at, between -1.0 and 1.0.
     */
    public RunShooter(ShooterSubsystem shooterSubsystem, double power) {
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        m_shooterSubsystem = shooterSubsystem;
        m_power = power;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_shooterSubsystem.setUpperShooter(m_power);
        m_shooterSubsystem.setLowerShooter(m_power);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.setUpperShooter(0.0);
        m_shooterSubsystem.setLowerShooter(0.0);
    }
}

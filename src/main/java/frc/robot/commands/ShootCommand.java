package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterPneumaticSubsystem;


public class ShootCommand extends CommandBase {

    private final ShooterPneumaticSubsystem m_shooterPneumaticSubsystem;
    private int ticksElapsed;

    public ShootCommand(ShooterPneumaticSubsystem shooterPneumaticSubsystem) {
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        m_shooterPneumaticSubsystem = shooterPneumaticSubsystem;
        addRequirements(m_shooterPneumaticSubsystem);
    }

    @Override
    public void initialize() {
        m_shooterPneumaticSubsystem.liftShooter();
        ticksElapsed = 0;
    }

    @Override
    public void execute() {
        ticksElapsed++;
    }

    @Override
    public boolean isFinished() {
        return ticksElapsed >= Constants.SHOOT_TICKS;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterPneumaticSubsystem.dropShooter();
    }
}

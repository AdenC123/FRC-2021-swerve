package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterPneumaticSubsystem extends SubsystemBase {
    /**
     * The Singleton instance of this ShooterPneumaticSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     * <p>
     * The INSTANCE field is volatile to ensure that multiple threads
     * offer it correctly when it is being initialized by ensuring changes
     * made in one thread are immediately reflected in other threads.
     */
    private volatile static ShooterPneumaticSubsystem INSTANCE;

    /**
     * Returns the Singleton instance of this ShooterPneumaticSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code ShooterPneumaticSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static ShooterPneumaticSubsystem getInstance() {
        // Use "Double Checked Locking" to ensure thread safety and best performance
        // Fast (non-synchronized) check to reduce overhead of acquiring a lock when it's not needed
        if (INSTANCE == null) {
            // Lock to make thread safe
            synchronized (ShooterPneumaticSubsystem.class) {
                // check nullness again as multiple threads can reach and
                // pass the initial non-synchronized null check above
                if (INSTANCE == null) {
                    INSTANCE = new ShooterPneumaticSubsystem();
                }
            }
        }
        return INSTANCE;
    }

    private final Solenoid m_shooterSolenoid = new Solenoid(Constants.Pneumatics.SHOOTER);

    /**
     * Creates a new instance of this ShooterPneumaticSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private ShooterPneumaticSubsystem() {
        m_shooterSolenoid.set(false);
    }

    public void liftShooter() {
        m_shooterSolenoid.set(true);
    }

    public void dropShooter() {
        m_shooterSolenoid.set(false);
    }
}


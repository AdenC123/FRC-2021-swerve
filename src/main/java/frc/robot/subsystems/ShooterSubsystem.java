package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    /**
     * The Singleton instance of this ShooterSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     * <p>
     * The INSTANCE field is volatile to ensure that multiple threads
     * offer it correctly when it is being initialized by ensuring changes
     * made in one thread are immediately reflected in other threads.
     */
    private volatile static ShooterSubsystem INSTANCE;

    /**
     * Returns the Singleton instance of this ShooterSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code ShooterSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static ShooterSubsystem getInstance() {
        // Use "Double Checked Locking" to ensure thread safety and best performance
        // Fast (non-synchronized) check to reduce overhead of acquiring a lock when it's not needed
        if (INSTANCE == null) {
            // Lock to make thread safe
            synchronized (ShooterSubsystem.class) {
                // check nullness again as multiple threads can reach and
                // pass the initial non-synchronized null check above
                if (INSTANCE == null) {
                    INSTANCE = new ShooterSubsystem();
                }
            }
        }
        return INSTANCE;
    }

    private final TalonSRX m_lowerShooter = new TalonSRX(Constants.MotorControllers.SHOOTER_LOWER);
    private final TalonSRX m_upperShooter = new TalonSRX(Constants.MotorControllers.SHOOTER_UPPER);
    private final Solenoid m_shooterSolenoid = new Solenoid(Constants.Pneumatics.SHOOTER);

    /**
     * Creates a new instance of this ShooterSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private ShooterSubsystem() {
        m_shooterSolenoid.set(false);
        m_lowerShooter.configFactoryDefault();
        m_lowerShooter.setNeutralMode(NeutralMode.Coast);
        m_upperShooter.configFactoryDefault();
        m_upperShooter.setNeutralMode(NeutralMode.Coast);
    }

    public void setLowerShooter(double power) {
        m_lowerShooter.set(ControlMode.PercentOutput, power);
    }

    public void setUpperShooter(double power) {
        m_upperShooter.set(ControlMode.PercentOutput, power);
    }

    public void liftShooter() {
        m_shooterSolenoid.set(true);
    }

    public void dropShooter() {
        m_shooterSolenoid.set(false);
    }

    public void shoot() {
        setLowerShooter(1.0);
        setUpperShooter(1.0);
    }

    public void stop() {
        setLowerShooter(0.0);
        setUpperShooter(0.0);
    }
}


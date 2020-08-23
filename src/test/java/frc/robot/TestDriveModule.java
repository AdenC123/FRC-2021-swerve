package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import frc.robot.DriveModule;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.platform.runner.JUnitPlatform;
import org.junit.runner.RunWith;

import static org.junit.jupiter.api.Assertions.fail;

import static org.mockito.Mockito.*;

@RunWith(JUnitPlatform.class)
public class TestDriveModule {

    private class InitializedDriveModule {
        // basic code representations for physical hardware
        final CANSparkMax driveMotor = mock(CANSparkMax.class);
        final CANSparkMax spinMotor = mock(CANSparkMax.class);
        final AnalogPotentiometer analogEncoder = mock(AnalogPotentiometer.class);
        // derived representations of components embedded in the physical hardware
        final CANEncoder driveEncoder = mock(CANEncoder.class);
        final CANPIDController drivePID = mock(CANPIDController.class);
        final CANEncoder spinEncoder = mock(CANEncoder.class);
        final CANPIDController spinPID = mock(CANPIDController.class);
        final DriveModule driveModule;

        public InitializedDriveModule() {
            when(analogEncoder.get()).thenReturn(0.375);
            driveModule = new DriveModule(driveMotor, driveEncoder, drivePID,
                    spinMotor, spinEncoder, spinPID,
                    analogEncoder, 0.125);
            // OK, this is the example in the technical documentation, which should have
            // set the spin encoder position to 4.5. Id should also have setup the PID
            // constants for the spin and drive PIDs - verify all of the calls to the PID
            // so all counters are reset.
            verifyPid(drivePID, Constants.DRIVE_kFF, Constants.DRIVE_kP, Constants.DRIVE_kI, Constants.DRIVE_IZONE);
            verifyPid(spinPID, 0.0, Constants.SPIN_kP, Constants.SPIN_kI, 0.0);
            verify(spinEncoder, times(1)).setPosition(4.5);
            verify(spinPID, times(1)).setReference(0.0, ControlType.kPosition);
        }
    }

    private void verifyPid(CANPIDController pid, double kFF, double kP, double kI, double kIZone) {
        verify(pid, times(1)).setFF(kFF);
        verify(pid, times(1)).setP(kP);
        verify(pid, times(1)).setI(kI);
        verify(pid, times(1)).setD(0.0);
        verify(pid, times(1)).setIZone(kIZone);
        verify(pid, times(1)).setOutputRange(-1.0, 1.0);
    }

    @Test
    @DisplayName("Test Calibration")
    void test_calibration() {
        new InitializedDriveModule();
    }

    @Test
    @DisplayName("Test setRadiansAndSpeed(Math.toRadians(10.0),1.0)")
    void test_set_10_1() {
        // Should spin positively
        InitializedDriveModule dm = new InitializedDriveModule();
        dm.driveModule.setRadiansAndSpeed(Math.toRadians(10.0), 1.0);
        verify(dm.spinPID, times(1)).setReference(Math.toRadians(10.0) * Constants.RADIANS_TO_SPIN_ENCODER,
                ControlType.kPosition);
        verify(dm.drivePID, times(1)).setReference(1.0 * Constants.MAX_DRIVE_VELOCITY,
                ControlType.kVelocity);
    }

    @Test
    @DisplayName("Test setRadiansAndSpeed(Math.toRadians(80.0),1.0)")
    void test_set_80_1() {
        // Should spin positively
        InitializedDriveModule dm = new InitializedDriveModule();
        dm.driveModule.setRadiansAndSpeed(Math.toRadians(80.0), 1.0);
        verify(dm.spinPID, times(1)).setReference(Math.toRadians(80.0) * Constants.RADIANS_TO_SPIN_ENCODER,
                ControlType.kPosition);
        verify(dm.drivePID, times(1)).setReference(1.0 * Constants.MAX_DRIVE_VELOCITY,
                ControlType.kVelocity);
    }

    @Test
    @DisplayName("Test setRadiansAndSpeed(Math.toRadians(100.0),1.0)")
    void test_set_100_1() {
        // Should spin negatively and go backwards
        InitializedDriveModule dm = new InitializedDriveModule();
        dm.driveModule.setRadiansAndSpeed(Math.toRadians(100.0), 1.0);
        verify(dm.spinPID, times(1)).setReference(Math.toRadians(-80.0) * Constants.RADIANS_TO_SPIN_ENCODER,
                ControlType.kPosition);
        verify(dm.drivePID, times(1)).setReference(-(1.0 * Constants.MAX_DRIVE_VELOCITY),
                ControlType.kVelocity);
    }

    @Test
    @DisplayName("Test setRadiansAndSpeed(Math.toRadians(-10.0),1.0)")
    void test_set_neg_10_1() {
        // Should spin negatively
        InitializedDriveModule dm = new InitializedDriveModule();
        dm.driveModule.setRadiansAndSpeed(Math.toRadians(-10.0), 1.0);
        verify(dm.spinPID, times(1)).setReference(Math.toRadians(-10.0) * Constants.RADIANS_TO_SPIN_ENCODER,
                ControlType.kPosition);
        verify(dm.drivePID, times(1)).setReference(1.0 * Constants.MAX_DRIVE_VELOCITY,
                ControlType.kVelocity);
    }

    @Test
    @DisplayName("Test setRadiansAndSpeed(Math.toRadians(-80.0),1.0)")
    void test_set_neg_80_1() {
        // Should spin negatively
        InitializedDriveModule dm = new InitializedDriveModule();
        dm.driveModule.setRadiansAndSpeed(Math.toRadians(-80.0), 1.0);
        verify(dm.spinPID, times(1)).setReference(Math.toRadians(-80.0) * Constants.RADIANS_TO_SPIN_ENCODER,
                ControlType.kPosition);
        verify(dm.drivePID, times(1)).setReference(1.0 * Constants.MAX_DRIVE_VELOCITY,
                ControlType.kVelocity);
    }

    @Test
    @DisplayName("Test setRadiansAndSpeed(Math.toRadians(-100.0),1.0)")
    void test_set_neg_100_1() {
        // Should spin positively and go backwards
        InitializedDriveModule dm = new InitializedDriveModule();
        dm.driveModule.setRadiansAndSpeed(Math.toRadians(100.0), 1.0);
        verify(dm.spinPID, times(1)).setReference(Math.toRadians(80.0) * Constants.RADIANS_TO_SPIN_ENCODER,
                ControlType.kPosition);
        verify(dm.drivePID, times(1)).setReference(-(1.0 * Constants.MAX_DRIVE_VELOCITY),
                ControlType.kVelocity);
    }
}

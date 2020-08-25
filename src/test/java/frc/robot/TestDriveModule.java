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
import org.mockito.AdditionalMatchers;
import org.mockito.ArgumentMatchers;

import static org.junit.jupiter.api.Assertions.*;

import static org.mockito.Mockito.*;

@RunWith(JUnitPlatform.class)
public class TestDriveModule {

    /**
     * This is a class that creates an initialized drive module using mocked motor controllers, motor encoders,
     * motor PID and analog encoder.
     */
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

        /**
         * Instantiate, initialize, and verify initialization called everything as expected. Reset all of the mock
         * objects that were touched in initialization before exiting this instantiation.
         */
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
            reset(spinEncoder, drivePID, spinPID);
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

    /**
     * The deal here is that we are setting a direction and speed for the module, but the actual direction
     * may be reversed 180, or the module may have gone over the 180 degree spin boundary, so the actual
     * may not be what was really set, but something that gives the same result.
     *
     * @param dm
     * @param radians
     * @param speed
     * @param actualRadians
     * @param actualSpeed
     */
    private void verifyRadiansAndSpeed(InitializedDriveModule dm, double radians, double speed,
                                       double actualRadians, double actualSpeed) {
        dm.driveModule.setRadiansAndSpeed(radians, speed);
        verify(dm.spinPID, times(1)).setReference(
                AdditionalMatchers.eq(actualRadians * Constants.RADIANS_TO_SPIN_ENCODER,.00001),
                ArgumentMatchers.eq(ControlType.kPosition));
        verify(dm.drivePID, times(1)).setReference(actualSpeed * Constants.MAX_DRIVE_VELOCITY,
                ControlType.kVelocity);
        // make encoder readings slightly different than what was actually set
        when(dm.spinEncoder.getPosition()).thenReturn(actualRadians + .001);
        when(dm.driveEncoder.getVelocity()).thenReturn((actualSpeed * Constants.MAX_DRIVE_VELOCITY) - 3.0);
        assertEquals(radians, dm.driveModule.getLastRadians());
        assertEquals(speed, dm.driveModule.getLastNormalizedSpeed());
        assertEquals(speed * Constants.MAX_DRIVE_VELOCITY, dm.driveModule.getLastSpeed());
        assertEquals(actualRadians + .001, dm.driveModule.getSpinEncoderPosition());
        assertEquals((actualSpeed * Constants.MAX_DRIVE_VELOCITY) - 3.0,
                dm.driveModule.getDriveEncoderPosition());

    }

    /**
     * This just does the instantiation and verifies the initial calibration. If there is a problem here
     * then all of the other tests will probably fail.
     */
    @Test
    @DisplayName("Test Calibration")
    void test_calibration() {
        new InitializedDriveModule();
    }

    /**
     * Test setting the spin PID K values
     */
    @Test
    @DisplayName("Test setSpinPID")
    void test_setSpinPID() {
        // Should spin positively
        InitializedDriveModule dm = new InitializedDriveModule();
        dm.driveModule.setSpinPID();
        verify(dm.spinPID, times(1)).setP(Constants.SPIN_kP);
        verify(dm.spinPID, times(1)).setI(Constants.SPIN_kI);
    }

    /**
     * Test setting the drive PID K values
     */
    @Test
    @DisplayName("Test setDrivePID")
    void test_setDrivePID() {
        // Should spin positively
        InitializedDriveModule dm = new InitializedDriveModule();
        dm.driveModule.setDrivePID();
        verify(dm.drivePID, times(1)).setFF(Constants.DRIVE_kFF);
        verify(dm.drivePID, times(1)).setP(Constants.DRIVE_kP);
        verify(dm.drivePID, times(1)).setI(Constants.DRIVE_kI);
    }

    /**
     * Test a basic move - 10 degrees clockwise, full speed forward.
     */
    @Test
    @DisplayName("Test setRadiansAndSpeed(Math.toRadians(10.0),1.0)")
    void test_set_10_1() {
        // Should spin positively
        InitializedDriveModule dm = new InitializedDriveModule();
        verifyRadiansAndSpeed(dm,Math.toRadians(10.0), 1.0,Math.toRadians(10.0), 1.0);
    }

    /**
     * Test a basic move specified in degrees - 10 degrees clockwise, full speed forward.
     */
    @Test
    @DisplayName("Test setDegreesAndSpeed(10.0,1.0)")
    void test_set_degrees_10_1() {
        // Should spin positively
        InitializedDriveModule dm = new InitializedDriveModule();
        dm.driveModule.setDegreesAndSpeed(10.0, 1.0);
        verify(dm.spinPID, times(1)).setReference(Math.toRadians(10.0) * Constants.RADIANS_TO_SPIN_ENCODER,
                ControlType.kPosition);
        verify(dm.drivePID, times(1)).setReference(1.0 * Constants.MAX_DRIVE_VELOCITY,
                ControlType.kVelocity);
    }

    /**
     * Test a close to, but less than, 90 degree clockwise, half speed. Spin should be clockwise and spped forward
     */
    @Test
    @DisplayName("Test setRadiansAndSpeed(Math.toRadians(80.0),0.5)")
    void test_set_80_1() {
        // Should spin positively
        InitializedDriveModule dm = new InitializedDriveModule();
        dm.driveModule.setRadiansAndSpeed(Math.toRadians(80.0), 0.5);
        verify(dm.spinPID, times(1)).setReference(Math.toRadians(80.0) * Constants.RADIANS_TO_SPIN_ENCODER,
                ControlType.kPosition);
        verify(dm.drivePID, times(1)).setReference(0.5 * Constants.MAX_DRIVE_VELOCITY,
                ControlType.kVelocity);
    }

    /**
     * Test a slightly greater than 90 degree clockwise, full speed. The closest rotation is to orient
     * the back of the wheel and go backwards - make sure this happens.
     */
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

    /**
     * Test a basic move - 10 degrees counter-clockwise, full speed.
     */
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
        dm.driveModule.setRadiansAndSpeed(Math.toRadians(-100.0), 1.0);
        verify(dm.spinPID, times(1)).setReference(Math.toRadians(80.0) * Constants.RADIANS_TO_SPIN_ENCODER,
                ControlType.kPosition);
        verify(dm.drivePID, times(1)).setReference(-(1.0 * Constants.MAX_DRIVE_VELOCITY),
                ControlType.kVelocity);
    }

    @Test
    @DisplayName("Test 180 boundary clockwise")
    void test_180_boundary_clockwise() {
        // Should spin positively and keep spinning positively at the 180 boundary - kind of a pain because we
        // need 2 less than 90 steps to get the front of the wheel there.
        InitializedDriveModule dm = new InitializedDriveModule();
        dm.driveModule.setRadiansAndSpeed(Math.toRadians(85.0), 1.0);
        dm.driveModule.setRadiansAndSpeed(Math.toRadians(170.0), 1.0);
        // This is the 180 boundary cross
        reset(dm.spinPID,dm.drivePID);
        dm.driveModule.setRadiansAndSpeed(Math.toRadians(-170.0), 1.0);
        verify(dm.spinPID, times(1)).setReference(
                AdditionalMatchers.eq(Math.toRadians(190.0) * Constants.RADIANS_TO_SPIN_ENCODER,.00001),
                ArgumentMatchers.eq(ControlType.kPosition));
        verify(dm.drivePID, times(1)).setReference(1.0 * Constants.MAX_DRIVE_VELOCITY,
                ControlType.kVelocity);
    }

    @Test
    @DisplayName("Test 180 boundary counter clockwise")
    void test_180_boundary_counter_clockwise() {
        // Should spin negatively and keep spinning negatively at the 180 boundary - kind of a pain because we
        // need 2 less than -90 steps to get the front of the wheel there.
        InitializedDriveModule dm = new InitializedDriveModule();
        dm.driveModule.setRadiansAndSpeed(Math.toRadians(-85.0), 1.0);
        dm.driveModule.setRadiansAndSpeed(Math.toRadians(-170.0), 1.0);
        // This is the 180 boundary cross
        reset(dm.spinPID,dm.drivePID);
        dm.driveModule.setRadiansAndSpeed(Math.toRadians(170.0), 1.0);
        verify(dm.spinPID, times(1)).setReference(
                AdditionalMatchers.eq(Math.toRadians(-190.0) * Constants.RADIANS_TO_SPIN_ENCODER,.00001),
                ArgumentMatchers.eq(ControlType.kPosition));
        verify(dm.drivePID, times(1)).setReference(1.0 * Constants.MAX_DRIVE_VELOCITY,
                ControlType.kVelocity);
    }

    /**
     * The forwards-backwards logic finds the least spin of the wheel (>= 90.0) and the appropriate forward/backward
     * multiplier. A concern it the programming has been making sure the forward-backward states are handled
     * correctly. Specifically, if I have 2 forward commands it a row or 2 backwards commands in a row, does the
     * module correctly remember what it is doing and not do a 180 degree spin.
     */
    @Test
    @DisplayName("Test backwards forwards")
    void test_forward_backward() {
        // Go forward 2 steps
        InitializedDriveModule dm = new InitializedDriveModule();
        dm.driveModule.setRadiansAndSpeed(0.0, 1.0);
        dm.driveModule.setRadiansAndSpeed(0.0, 1.0);
        verify(dm.spinPID, times(2)).setReference( 0.0, ControlType.kPosition);
        verify(dm.drivePID, times(2)).setReference(1.0 * Constants.MAX_DRIVE_VELOCITY,
                ControlType.kVelocity);
        // Go backwards (180 degrees) 2 steps - should be no spin and negative speed
        reset(dm.spinPID,dm.drivePID);
        dm.driveModule.setRadiansAndSpeed(Math.PI, 1.0);
        dm.driveModule.setRadiansAndSpeed(Math.PI, 1.0);
        verify(dm.spinPID, times(2)).setReference( 0.0, ControlType.kPosition);
        verify(dm.drivePID, times(2)).setReference(-1.0 * Constants.MAX_DRIVE_VELOCITY,
                ControlType.kVelocity);
        // Go forwards again steps - should be no spin and positive speed
        reset(dm.spinPID,dm.drivePID);
        dm.driveModule.setRadiansAndSpeed(0.0, 1.0);
        dm.driveModule.setRadiansAndSpeed(0.0, 1.0);
        verify(dm.spinPID, times(2)).setReference( 0.0, ControlType.kPosition);
        verify(dm.drivePID, times(2)).setReference(1.0 * Constants.MAX_DRIVE_VELOCITY,
                ControlType.kVelocity);
    }
}

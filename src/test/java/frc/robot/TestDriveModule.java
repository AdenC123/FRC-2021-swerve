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

    @Test
    @DisplayName("Test Calibration")
    void test_calibration() {
        // basic code representations for physical hardware
        CANSparkMax driveMotor = mock(CANSparkMax.class);
        CANSparkMax spinMotor = mock(CANSparkMax.class);
        AnalogPotentiometer analogEncoder = mock(AnalogPotentiometer.class);
        when(analogEncoder.get()).thenReturn(0.375);
        // derived representations of components embedded in the physical hardware
        CANEncoder driveEncoder = mock(CANEncoder.class);
        CANPIDController drivePID = mock(CANPIDController.class);
        CANEncoder spinEncoder = mock(CANEncoder.class);
        CANPIDController spinPID = mock(CANPIDController.class);
        DriveModule driveModule = new DriveModule(driveMotor, driveEncoder, drivePID,
                spinMotor, spinEncoder, spinPID,
                analogEncoder, 0.125);
        // OK, this is the example in the technical documentation, which should have
        // set the spin encoder position to 4.5
        verify(spinEncoder, times(1)).setPosition(4.5);
    }
}

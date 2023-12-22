package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurnMotorCharacterizationSubsystem extends SubsystemBase {
    private final TalonFX m_motor;
    private final CANcoder m_encoder;

    private final TorqueCurrentFOC m_currentReq = new TorqueCurrentFOC(0).withMaxAbsDutyCycle(0.5).withUpdateFreqHz(1000);

    private final StatusSignal<Double> m_encoderPosition;

    public TurnMotorCharacterizationSubsystem(int id, String canbus) {
        m_motor = new TalonFX(id, canbus);
        m_motor.getConfigurator().apply(new TalonFXConfiguration()); // Factory default
        m_motor.optimizeBusUtilization();
        m_encoder = new CANcoder(id, canbus);
        m_encoder.getConfigurator().apply(new CANcoderConfiguration()); // Factory default
        m_encoderPosition = m_encoder.getPosition();
        m_encoderPosition.setUpdateFrequency(1000);
        m_encoder.optimizeBusUtilization();
    }

    public void setCurrent(double amps) {
        m_currentReq.Output = amps;
        m_motor.setControl(m_currentReq);
    }

    public double getPosition() {
        return m_encoderPosition.waitForUpdate(0.02).getValueAsDouble();
    }
}

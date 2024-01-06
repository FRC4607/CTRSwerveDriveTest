package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class CouplingCharacterizationSubsystem extends TurnMotorCharacterizationSubsystem {
    private final TalonFX m_drive;
    private final StatusSignal<Double> m_drivePos;
    public CouplingCharacterizationSubsystem(int turnid, int driveid, String canbus) {
        super(turnid, canbus);
        m_currentReq.MaxAbsDutyCycle = 0.05;

        m_drive = new TalonFX(driveid, canbus);
        m_drive.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)));
        m_drivePos = m_drive.getPosition();
        m_drivePos.setUpdateFrequency(1000);
        m_drive.optimizeBusUtilization();
    }

    public void updateSignals() {
        BaseStatusSignal.waitForAll(0.002, m_drivePos, m_encoderPosition);
    }

    public double getTurnPosition() {
        return m_encoderPosition.getValueAsDouble();
    }

    public double getDrivePosition() {
        return m_drivePos.getValueAsDouble();
    }
}

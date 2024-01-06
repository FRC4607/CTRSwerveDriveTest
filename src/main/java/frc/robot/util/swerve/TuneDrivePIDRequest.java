package frc.robot.util.swerve;

import java.lang.reflect.Field;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TuneDrivePIDRequest implements SwerveRequest {
    private boolean configChanged = false;
    private final Slot0Configs m_config = new Slot0Configs().withKS(2.447242424);
    private final SwerveModuleState m_state = new SwerveModuleState();
    
    @Override
    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modules) {
        if (m_config.kP != SmartDashboard.getNumber("Drive_kP", 0.0)) {
            m_config.kP = SmartDashboard.getNumber("Drive_kP", 0.0);
            configChanged = true;
        }
        if (m_config.kD != SmartDashboard.getNumber("Drive_kD", 0.0)) {
            m_config.kD = SmartDashboard.getNumber("Drive_kD", 0.0);
            configChanged = true;
        }
        if (m_state.speedMetersPerSecond != SmartDashboard.getNumber("Drive_target", 0.0)) {
            m_state.speedMetersPerSecond = SmartDashboard.getNumber("Drive_target", 0.0);
        }
        for (int i = 0; i < modules.length; i++) {
            if (configChanged) {
                modules[i].getDriveMotor().getConfigurator().apply(m_config);
            }
            modules[i].apply(m_state, DriveRequestType.Velocity, SteerRequestType.MotionMagicExpo);
        }
        configChanged = false;
        return StatusCode.OK;
    }
}
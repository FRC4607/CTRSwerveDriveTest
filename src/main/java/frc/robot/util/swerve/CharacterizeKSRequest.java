package frc.robot.util.swerve;

import java.lang.reflect.Field;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CharacterizeKSRequest implements SwerveRequest {
    private static final SwerveModuleState ZERO_STATE = new SwerveModuleState();

    private enum State {
        INIT("#000000"),
        REMOVE_BACKLASH("#888888"),
        WAIT_FOR_STOP("#FF0000"),
        WAIT_FOR_START("#00FF00"),
        TERMINATE("#FFFFFF");

        private final String hexID;

        private State(String hexID) {
            this.hexID = hexID;
        }

        public String getHexValue() {
            return hexID;
        }
    }

    private State m_state = State.INIT;

    private double m_pos = 0;
    private double m_current = 5;

    private int m_loopCounter = 0;

    private final DoubleLogEntry m_log = new DoubleLogEntry(DataLogManager.getLog(), "Estimated Drive kS Value", "{\"unit\":\"A\"}");

    public double getFF(SwerveModule module) {
        SmartDashboard.putString("Test State", m_state.toString());
        SmartDashboard.putString("Test State Color", m_state.getHexValue());
        switch (m_state) {
            case INIT: { // Startup, remove any backlash with a high value
                m_pos = module.getPosition(true).distanceMeters;
                m_current = 5;
                m_state = State.REMOVE_BACKLASH;
                return 5.0;
            }
            case REMOVE_BACKLASH: { // Wait until we see movement, then stop
                double newPos = module.getPosition(true).distanceMeters;
                if (Math.abs(newPos - m_pos) > 0.002) { // If the positions aren't equal, set up for next state
                    m_pos = newPos;
                    m_loopCounter = 0;
                    m_state = State.WAIT_FOR_STOP;
                    m_current = 0.0;
                    return 0.0;
                }
                return 5.0;
            }
            case WAIT_FOR_STOP: { // Wait for the module to come to a complete stop
                double newPos = module.getPosition(true).distanceMeters;
                if (Math.abs(newPos - m_pos) > 0.002) { // If there is a difference, reset the counter
                    m_pos = newPos;
                    m_loopCounter = 0;
                    return 0.0;
                }
                else {
                    m_loopCounter++;
                }
                if (m_loopCounter == 60) { // If 60 calls have had no movement
                    m_pos = module.getPosition(true).distanceMeters; // Difference might be <= 0.01
                    m_loopCounter = 0; // Reset the loop counter
                    m_state = State.WAIT_FOR_START; // Move on
                }
                return 0.0;
            }
            case WAIT_FOR_START: { // Wait for the module to start
                double newPos = module.getPosition(true).distanceMeters;
                if (Math.abs(newPos - m_pos) <= 0.002) { // If the positions are (almost) equal, increase the current
                    m_current += 0.5 / 250.0; // 0.5 A/sec
                    return m_current;
                }
                else { // If there is a difference, update the position, stop applying current, log current, reset current, and wait to stop
                    m_pos = newPos;
                    if (m_current >= 1.0) {
                        m_state = State.WAIT_FOR_STOP;
                        m_log.append(m_current);
                        SmartDashboard.putNumber("kS Test Result", m_current);
                        m_current = 0.0;
                        return 0.0;
                    }
                    return m_current;
                }
            }
            default:
                // Should be unreachable
                return 0.0;
        }
    }

    @Override
    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modules) {
        double current = getFF(modules[0]);
        for (SwerveModule module : modules) {
            VelocityTorqueCurrentFOC request;
            try {
                Field requestField = module.getClass().getDeclaredField("m_velocityTorqueSetter");
                requestField.setAccessible(true);
                request = (VelocityTorqueCurrentFOC) requestField.get(module);
            }
            catch (NoSuchFieldException e) {
                // uh oh
                System.err.println(e.toString());
                return StatusCode.GeneralError;
            }
            catch (IllegalAccessException e) {
                // uh oh
                System.err.println(e.toString());
                return StatusCode.GeneralError;
            }
            request.FeedForward = current;
            module.apply(ZERO_STATE, DriveRequestType.Velocity);
        }
        return StatusCode.OK;
    }
}
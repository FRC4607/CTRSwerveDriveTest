package frc.robot.util.swerve;

import java.lang.reflect.Field;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.kinematics.SwerveModuleState;
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
    private double m_currentPrev = 0;
    private double m_current = 5;

    private double m_direction = -1.0; // Will be flipped in init
    private boolean m_removedBacklash = false;

    private int m_loopCounter = 0;

    public double getFF(SwerveModule module) {
        SmartDashboard.putString("Test State", m_state.toString());
        SmartDashboard.putString("Test State Color", m_state.getHexValue());
        switch (m_state) {
            case INIT: { // Remove any backlash with a high value
                m_direction *= -1.0;
                m_pos = module.getPosition(true).distanceMeters;
                m_state = State.REMOVE_BACKLASH;
                return 5.0 * m_direction;
            }
            case REMOVE_BACKLASH: { // Wait until we see movement, then stop
                double newPos = module.getPosition(true).distanceMeters;
                if (Double.compare(m_pos, newPos) != 0) { // If the positions aren't equal, set up for next state
                    m_pos = newPos;
                    m_loopCounter = 0;
                    m_state = State.WAIT_FOR_STOP;
                    return 0.0;
                }
                return 5.0 * m_direction;
            }
            case WAIT_FOR_STOP: { // Wait for the module to come to a complete stop
                double newPos = module.getPosition(true).distanceMeters;
                if (Double.compare(m_pos, newPos) != 0) { // If there is a difference, reset the counter
                    m_pos = newPos;
                    m_loopCounter = 0;
                    return 0.0;
                }
                else {
                    m_loopCounter++;
                }
                if (m_loopCounter == 250) { // If 250 loops have had no movement
                    if (m_removedBacklash) { // Time to turn around
                        m_loopCounter = 0; // Reset the loop counter
                        m_removedBacklash = false; // Next time go ahead
                        m_state = State.INIT; // Move on
                        return 0.0;
                    }
                    else { // Keep going
                        double halfDiff = Math.abs(m_current - m_currentPrev) / 2.0;
                        m_currentPrev = m_current;
                        m_current -= halfDiff; // Subtract half the difference to apply less torque
                        m_loopCounter = 0; // Reset the loop counter
                        m_removedBacklash = true; // Next time turn around
                        m_state = State.WAIT_FOR_START; // Move on
                        return m_current * m_direction;
                    }
                }
                return 0.0;
            }
            case WAIT_FOR_START: { // Wait for the module to start
                double newPos = module.getPosition(true).distanceMeters;
                if (Double.compare(m_pos, newPos) == 0) { // If the positions are equal, increment the counter
                    m_loopCounter++;
                }
                else { // If there is a difference, update the position, reset the counter, stop applying current, and wait to stop
                    m_pos = newPos;
                    m_loopCounter = 0;
                    m_state = State.WAIT_FOR_STOP;
                    return 0.0;
                }
                if (m_loopCounter == 250) { // If 250 loops have had no movement
                    double halfDiff = Math.abs(m_current - m_currentPrev) / 2.0;
                    if (Math.abs(halfDiff) < 0.0005) { // If the difference between measurements is less than 0.001, stop trying to move and terminate
                        m_state = State.TERMINATE;
                        SmartDashboard.putNumber("kS Test Result", m_current);
                        return 0.0;
                    }
                    else { // Add half the difference, reset the loop, and apply the new current
                        m_currentPrev = m_current;
                        m_current += halfDiff;
                        m_loopCounter = 0;
                        return m_current * m_direction;
                    }
                }
                return m_current * m_direction;
            }
            case TERMINATE:
                m_pos = 0;
                m_currentPrev = 0;
                m_current = 0;

                m_loopCounter = 0;

                m_direction = -1.0;
                m_removedBacklash = false;
                return 0.0;

            default:
                // The code should never reach here
                return 0.0;
        }
    }

    @Override
    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modules) {
        double current = getFF(modules[0]);
        for (SwerveModule module : modules) {
            VelocityTorqueCurrentFOC request;
            try {
                Field requestField = modules.getClass().getDeclaredField("m_velocityTorqueSetter");
                requestField.setAccessible(true);
                request = (VelocityTorqueCurrentFOC) requestField.get(module);
            }
            catch (NoSuchFieldException e) {
                // uh oh
                return StatusCode.GeneralError;
            }
            catch (IllegalAccessException e) {
                // uh oh
                return StatusCode.GeneralError;
            }
            request.FeedForward = current;
            module.apply(ZERO_STATE, DriveRequestType.Velocity);
        }
        return StatusCode.OK;
    }
}
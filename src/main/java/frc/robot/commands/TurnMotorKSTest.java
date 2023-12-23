package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurnMotorCharacterizationSubsystem;

public class TurnMotorKSTest extends Command {
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
    private double m_current = 0;

    private int m_loopCounter = 0;

    private final TurnMotorCharacterizationSubsystem m_motor;

    public TurnMotorKSTest(TurnMotorCharacterizationSubsystem subsystem) {
        m_motor = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        SmartDashboard.putString("Test State", m_state.toString());
        SmartDashboard.putString("Test State Color", m_state.getHexValue());
        switch (m_state) {
            case INIT: { // Startup, remove any backlash with a high value
                m_pos = m_motor.getPosition();
                m_motor.setCurrent(5);
                m_current = 5.0;
                m_state = State.REMOVE_BACKLASH;
                break;
            }
            case REMOVE_BACKLASH: { // Wait until we see movement, then stop
                double newPos = m_motor.getPosition();
                if (Double.compare(m_pos, newPos) != 0) { // If the positions aren't equal, set up for next state
                    m_pos = newPos;
                    m_motor.setCurrent(0);
                    m_loopCounter = 0;
                    m_state = State.WAIT_FOR_STOP;
                }
                break;
            }
            case WAIT_FOR_STOP: { // Wait for the module to come to a complete stop
                double newPos = m_motor.getPosition();
                if (Double.compare(m_pos, newPos) != 0) { // If there is a difference, reset the counter
                    m_pos = newPos;
                    m_loopCounter = 0;
                    break;
                }
                else {
                    m_loopCounter++;
                }
                if (m_loopCounter == 50) { // If 50 loops have had no movement
                    double halfDiff = Math.abs(m_current - m_currentPrev) / 2.0;
                    m_currentPrev = m_current;
                    m_current -= halfDiff; // Subtract half the difference to apply less torque
                    m_loopCounter = 0; // Reset the loop counter
                    m_motor.setCurrent(m_current); // Apply the new current
                    m_state = State.WAIT_FOR_START; // Move on
                }
                break;
            }
            case WAIT_FOR_START: { // Wait for the module to start
                double newPos = m_motor.getPosition();
                if (Double.compare(m_pos, newPos) == 0) { // If the positions are equal, increment the counter
                    m_loopCounter++;
                }
                else { // If there is a difference, update the position, reset the counter, stop applying current, and wait to stop
                    m_pos = newPos;
                    m_loopCounter = 0;
                    m_motor.setCurrent(0);
                    m_state = State.WAIT_FOR_STOP;
                    break;
                }
                if (m_loopCounter == 50) { // If 50 loops have had no movement
                    double halfDiff = Math.abs(m_current - m_currentPrev) / 2.0;
                    if (Math.abs(halfDiff) < 0.0005) { // If the difference between measurements is less than 0.001, stop trying to move and terminate
                        m_state = State.TERMINATE;
                        m_motor.setCurrent(0);
                    }
                    else { // Add half the difference, reset the loop, and apply the new current
                        m_currentPrev = m_current;
                        m_current += halfDiff;
                        m_loopCounter = 0;
                        m_motor.setCurrent(m_current);
                    }
                }
                break;
            }
            case TERMINATE:
                break; // let end take care of this
        }
    }

    @Override
    public boolean isFinished() {
        return m_state == State.TERMINATE;
    }

    @Override
    public void end(boolean i) {
        m_motor.setCurrent(0);
        SmartDashboard.putNumber("kS Test Result", m_current);

        m_state = State.INIT;

        m_pos = 0;
        m_currentPrev = 0;
        m_current = 0;

        m_loopCounter = 0;

        SmartDashboard.putString("Test State", m_state.toString());
        SmartDashboard.putString("Test State Color", m_state.getHexValue());
    }
}

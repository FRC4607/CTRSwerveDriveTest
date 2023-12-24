package frc.robot.commands;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurnMotorCharacterizationSubsystem;

public class TurnMotorKSTest extends Command {
    private enum State {
        INIT("#000000"),
        REMOVE_BACKLASH("#888888"),
        WAIT_FOR_STOP("#FF0000"),
        WAIT_FOR_START("#00FF00");

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
    private double m_current = 0;

    private int m_loopCounter = 0;

    private final TurnMotorCharacterizationSubsystem m_motor;

    private final DoubleLogEntry m_log = new DoubleLogEntry(DataLogManager.getLog(), "Estimated Turn kS Value", "{\"unit\":\"A\"}");

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
                m_current = 5;
                m_state = State.REMOVE_BACKLASH;
                break;
            }
            case REMOVE_BACKLASH: { // Wait until we see movement, then stop
                double newPos = m_motor.getPosition();
                if (Math.abs(newPos - m_pos) > 0.2/360.0) { // If the positions aren't equal, set up for next state
                    m_pos = newPos;
                    m_motor.setNeutral();
                    m_loopCounter = 0;
                    m_state = State.WAIT_FOR_STOP;
                }
                break;
            }
            case WAIT_FOR_STOP: { // Wait for the module to come to a complete stop
                double newPos = m_motor.getPosition();
                if (Math.abs(newPos - m_pos) > 0.2/360.0) { // If there is a difference, reset the counter
                    m_pos = newPos;
                    m_loopCounter = 0;
                    break;
                }
                else {
                    m_loopCounter++;
                }
                if (m_loopCounter == 13) { // If 13 loops have had no movement
                    m_pos = m_motor.getPosition(); // Difference might be <= 0.2
                    m_loopCounter = 0; // Reset the loop counter
                    m_state = State.WAIT_FOR_START; // Move on
                }
                break;
            }
            case WAIT_FOR_START: { // Wait for the module to start
                double newPos = m_motor.getPosition();
                if (Math.abs(newPos - m_pos) <= 0.2/360.0) { // If the positions are (almost) equal, increase the current
                    m_current += 0.5 / 50.0; // 0.5 A/sec, or 0.01 A/loop
                    m_motor.setCurrent(m_current);
                    break;
                }
                else { // If there is a difference, update the position, stop applying current, log current, reset current, and wait to stop
                    m_pos = newPos;
                    if (m_current >= 2.0) { // Outlier rejection
                        m_motor.setNeutral();
                        m_state = State.WAIT_FOR_STOP;
                        m_log.append(m_current);
                        SmartDashboard.putNumber("kS Test Result", m_current);
                        m_current = 0.0;
                    }
                    break;
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean i) {
        m_motor.setNeutral();

        m_state = State.INIT;

        m_pos = 0;
        m_current = 0;

        m_loopCounter = 0;

        SmartDashboard.putString("Test State", m_state.toString());
        SmartDashboard.putString("Test State Color", m_state.getHexValue());
    }
}

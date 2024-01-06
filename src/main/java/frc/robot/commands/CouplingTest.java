package frc.robot.commands;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CouplingCharacterizationSubsystem;

public class CouplingTest extends Command {
    private final CouplingCharacterizationSubsystem m_subsystem;

    private double m_turnOffset;
    private double m_driveOffset;

    private boolean m_backlashRemoved;

    private final DoubleLogEntry m_log = new DoubleLogEntry(DataLogManager.getLog(), "Coupling Test Result");

    public CouplingTest(CouplingCharacterizationSubsystem subsystem) {
        m_subsystem = subsystem;
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {
        m_backlashRemoved = false;
        m_subsystem.updateSignals();
        m_turnOffset = m_subsystem.getTurnPosition();
        m_driveOffset = m_subsystem.getDrivePosition();
        m_subsystem.setCurrent(10.0);
    }

    @Override
    public void execute() {
        m_subsystem.updateSignals();
        double turn = m_subsystem.getTurnPosition() - m_turnOffset;
        double drive = m_subsystem.getDrivePosition() - m_driveOffset;
        if (turn >= 1 && drive >= 1 ) {
            if (m_backlashRemoved) {
                m_log.append(drive/turn);
                SmartDashboard.putNumber("Coupling Test Result", drive/turn);
            }
            else {
                m_turnOffset = m_subsystem.getTurnPosition();
                m_driveOffset = m_subsystem.getDrivePosition();
                m_backlashRemoved = true;
            }
        }
    }

    @Override
    public void end(boolean i) {
        m_subsystem.setNeutral();
    }
}

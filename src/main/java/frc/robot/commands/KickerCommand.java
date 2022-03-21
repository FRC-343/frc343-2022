package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Kicker;

public class KickerCommand extends CommandBase {

    private final Kicker m_kicker;

    public KickerCommand(Kicker kicker) {
        m_kicker = kicker;
        addRequirements(m_kicker);
    }

    @Override
    public void execute() {
        if (AimShootCommand.activateKicker || IntakeCommand.activateKicker) {
            m_kicker.setKicker(1.0);
        } else {
            m_kicker.setKicker(0.0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

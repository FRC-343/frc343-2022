package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;

public class IntakeCommand extends CommandBase {
    private final Intake m_intake;
    private final Kicker m_kicker;

    private double kIntakeSpeed;

    public IntakeCommand(Intake intake, Kicker kicker, double intakeSpeed) {
        m_intake = intake;
        m_kicker = kicker;
        kIntakeSpeed = intakeSpeed;
        addRequirements(m_intake, m_kicker);
    }

    public IntakeCommand(Intake intake, Kicker kicker) {
        this(intake, kicker, 0.8); // defaults to .8 speed
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_intake.lower();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_intake.setIntake(kIntakeSpeed);

        if (!m_intake.getCellDetector()) { // if no ball is in chamber run the kicker so it goes into chanber, leaving
            m_kicker.setKicker(1.0); // room for the 2nd ball in the hopper
        } else {
            m_kicker.setKicker(0.0);
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intake.setIntake(0);
        m_kicker.setKicker(0);
        m_intake.raise();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

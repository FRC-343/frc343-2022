package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {
    private final Intake m_intake;

    public IntakeCommand(Intake intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_intake.lower();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_intake.setIntake(0.80);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intake.setIntake(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

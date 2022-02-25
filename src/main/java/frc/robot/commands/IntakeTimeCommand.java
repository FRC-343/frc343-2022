package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;

public class IntakeTimeCommand extends CommandBase {
    private final Intake m_intake;
    private final double time;
    private Timer t;

    public IntakeTimeCommand(Intake intake, double time) {
        m_intake = intake;
        this.time = time;
        addRequirements(m_intake);

        t = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_intake.lower();
        t.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_intake.setIntake(0.8);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intake.setIntake(0);
        t.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return t.get() >= time;
    }
}

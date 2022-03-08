package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import edu.wpi.first.wpilibj.Timer;

public class IntakeTimeCommand extends CommandBase {
    private final Intake m_intake;
    private final Kicker m_kicker;
    private final double time;
    private Timer t;

    public IntakeTimeCommand(Intake intake, Kicker kicker, double time) { // this command is mostly just for auto and testing
        m_intake = intake;
        m_kicker = kicker;
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
        m_intake.raise();
        t.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return t.get() >= time;
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;

public class IntakeTimeCommand extends CommandBase {
    private final Intake m_intake;
    private final Hopper m_hopper;
    private final boolean m_noHopper;
    private final double time;
    private Timer t;

    public IntakeTimeCommand(Intake intake, Hopper hooooooooooooooooooooOooooopper, boolean noHopper, double time) {
        m_intake = intake;
        m_hopper = hooooooooooooooooooooOooooopper;
        m_noHopper = noHopper;
        this.time = time;
        addRequirements(m_intake, m_hopper);

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
        m_intake.setIntake(0.65);
        if (!m_noHopper) {
            if (m_hopper.checkReady()) {
                m_hopper.setHopper(-0.6);
                m_hopper.setKicker(0.24);
            } else {
                m_hopper.setHopper(0);
                m_hopper.setKicker(0);
            }
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_hopper.setHopper(0);
        m_hopper.setKicker(0);
        m_intake.setIntake(0);
        t.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return t.get() >= time;
    }
}

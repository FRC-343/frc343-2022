package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class Intake2Command extends CommandBase {
    private final Intake m_intake;
    private final Kicker m_kicker;
    private final Shooter m_shooter;
    private final Timer t;

    private double kIntakeSpeed;

    public Intake2Command(double intakeSpeed) {
        m_intake = Intake.getInstance();
        m_kicker = Kicker.getInstance();
        m_shooter = Shooter.getInstance();
        addRequirements(m_intake, m_kicker, m_shooter);

        kIntakeSpeed = intakeSpeed;
        t = new Timer();
    }

    public Intake2Command() {
        this(0.8); // defaults to .8 speed
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        t.reset();
        t.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_intake.setIntake(kIntakeSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intake.setIntake(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (t.get() >= (Math.sqrt(2) / 2)) { // no comet // no comment
            return true;
        } else {
            return false;
        }

    }
}
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class IntakeCommand extends CommandBase {
    private final Intake m_intake;
    private final Kicker m_kicker;
    private final Shooter m_shooter;

    private double kIntakeSpeed;
    private boolean m_raise;

    public IntakeCommand(Intake intake, Kicker kicker, Shooter shooter, double intakeSpeed, boolean raise) {
        m_intake = intake;
        m_kicker = kicker;
        m_shooter = shooter;
        kIntakeSpeed = intakeSpeed;
        m_raise = raise;
        addRequirements(m_intake, m_kicker, m_shooter);
    }

    public IntakeCommand(Intake intake, Kicker kicker, Shooter shooter) {
        this(intake, kicker, shooter, 0.8, true); // defaults to .8 speed
    }

    public IntakeCommand(Intake intake, Kicker kicker, Shooter shooter, double intakeSpeed) {
        this(intake, kicker, shooter, intakeSpeed, true);
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

        if (!m_kicker.getCellDetector()) { // if no ball is in chamber run the kicker so it goes into chanber, leaving
            m_kicker.setKicker(1.0); // room for the 2nd ball in the hopper
            m_shooter.shoot(0.0);
        } else { // if getCellDetector()
            if (Robot.kUseColorSensor) {
                if (!m_kicker.isBadCargo()) { // if good cargo stop kicker
                    m_kicker.setKicker(0.0);
                    m_shooter.shoot(0.0);
                } else if (m_kicker.isBadCargo()) { // if bad then shoot out
                    m_shooter.shoot(13, 13);
                    m_kicker.setKicker(1);
                }
            } else {
                m_kicker.setKicker(0.0);
            }
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intake.setIntake(0);
        m_kicker.setKicker(0);
        m_shooter.shoot(0);
        if (m_raise) {
            m_intake.raise();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

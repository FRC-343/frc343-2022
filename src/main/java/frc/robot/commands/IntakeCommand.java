package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;

public class IntakeCommand extends CommandBase {
    private final Intake m_intake;
    private final Kicker m_kicker;

    private double kIntakeSpeed;
    private boolean m_raise;

    public static boolean activateKicker = false;

    public static double activateShooter[] = { 0, 0 };

    public IntakeCommand(Intake intake, Kicker kicker, double intakeSpeed, boolean raise) {
        m_intake = intake;
        m_kicker = kicker;
        kIntakeSpeed = intakeSpeed;
        m_raise = raise;
        addRequirements(m_intake);
    }

    public IntakeCommand(Intake intake, Kicker kicker) {
        this(intake, kicker, 0.8, true); // defaults to .8 speed
    }

    public IntakeCommand(Intake intake, Kicker kicker, double intakeSpeed) {
        this(intake, kicker, intakeSpeed, true);
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
            activateKicker = true; // room for the 2nd ball in the hopper
            shoot(0, 0);
        } else { // if getCellDetector()
            if (Robot.kUseColorSensor) {
                if (!m_kicker.isBadCargo()) { // if good cargo stop kicker
                    activateKicker = false;
                    shoot(0, 0);
                } else if (m_kicker.isBadCargo()) { // if bad then shoot out
                    shoot(50, -95); 
                    activateKicker = true;
                }
            } else {
                activateKicker = false;
            }
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intake.setIntake(0);
        activateKicker = false;
        shoot(0, 0);
        if (m_raise) {
            m_intake.raise();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    private void shoot(double bottomSpeed, double topSpeed) {
        activateShooter[0] = bottomSpeed;
        activateShooter[1] = topSpeed;
    }
}

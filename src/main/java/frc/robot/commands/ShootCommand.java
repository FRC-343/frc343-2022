package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends CommandBase {
    private static final double kTopShootSpeed = 39.00; // rev per sec 125.0
    private static final double kTopShootReadySpeed = 34.0; // rev per sec

    private static final double kBottomShootSpeed = 70.00; // rev per sec 125.0
    private static final double kBottomShootReadySpeed = 68.0; // rev per sec

    private final Shooter m_shooter;
    private final Kicker m_kicker;
    private final boolean m_waitForAim;
    private Timer t;
    private final double time;

    public ShootCommand(Shooter shooter, Kicker kicker, boolean waitForAim) {
        m_shooter = shooter;
        m_kicker = kicker;
        m_waitForAim = waitForAim;
        addRequirements(m_shooter, m_kicker);

        t = new Timer();
        time = 3;
    }

    public ShootCommand(Shooter shooter, Kicker kicker) {
        this(shooter, kicker, false); // defaults to false if not given, shooter will fire even if not aimed
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_waitForAim) { // checks to see if aimed before firing
            m_shooter.shoot(kBottomShootSpeed, kTopShootSpeed);
            if (m_shooter.getBottomShooterRPS() >= kBottomShootReadySpeed
                    && m_shooter.getTopShooterRPS() >= kTopShootReadySpeed && Hood.isAimed()) {
                m_kicker.setKicker(1.0);
            } else {
                m_kicker.setKicker(0);
            }
        } else { // fire even if not aiming

            m_shooter.shoot(kBottomShootSpeed, kTopShootSpeed);
            if (m_shooter.getBottomShooterRPS() >= kBottomShootReadySpeed
                    && m_shooter.getTopShooterRPS() >= kTopShootReadySpeed) {
                m_kicker.setKicker(1.0);
            } else {
                m_kicker.setKicker(0);
            }
        }

        System.out.println("top = " + m_shooter.getTopShooterRPS() + "\n bottom = " + m_shooter.getBottomShooterRPS());

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.shoot(0);
        m_kicker.setKicker(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return t.get() >= time;

        // return false;
    }
}

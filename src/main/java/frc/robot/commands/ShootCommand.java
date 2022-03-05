package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class ShootCommand extends CommandBase {
    private static double kTopShootSpeed;
    private static double kTopShootReadySpeed;

    private static double kBottomShootSpeed;
    private static double kBottomShootReadySpeed = 70; // rev per sec

    private final Shooter m_shooter;
    private final Kicker m_kicker;
    private final Vision m_vision;
    private final boolean m_waitForAim; // for multitasking
    private final boolean m_stopAfterTime; // for auto
    private Timer t;
    private final double time;

    public ShootCommand(Shooter shooter, Kicker kicker, Vision vision, boolean waitForAim, boolean stopAfterTime) {
        m_shooter = shooter;
        m_kicker = kicker;
        m_vision = vision;

        addRequirements(m_shooter, m_kicker, m_vision);

        m_waitForAim = waitForAim;
        m_stopAfterTime = stopAfterTime;

        if (AimCommand.kShooterSpeedFromAim > 0.0) { // try using aiming value next
            kBottomShootReadySpeed = AimCommand.kShooterSpeedFromAim;
        } else {
            kBottomShootReadySpeed = 70;
        }

        if (kBottomShootReadySpeed == 70) {
            kBottomShootSpeed = kBottomShootReadySpeed * (8.0 / 7); // bottom speed = 1/7 more than bottom ready speed, 80 rps
        } else { 
            kBottomShootSpeed = kBottomShootReadySpeed * 1.2; // higher value so the ready speed will reach the speed it is aiming for
        }

        kTopShootReadySpeed = kBottomShootReadySpeed / 2.0; // top ready speed = 1/2 of bottom ready speed, 35 rps
        kTopShootSpeed = kTopShootReadySpeed * (8.0 / 7); // top speed = 1/7 more than top ready speed, 40 rps

        t = new Timer();
        time = 2.0;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        t.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_waitForAim) { // checks to see if aimed before firing
            m_shooter.shoot(kBottomShootSpeed, kTopShootSpeed);
            if (m_shooter.getBottomShooterRPS() >= kBottomShootReadySpeed
                    && m_shooter.getTopShooterRPS() >= kTopShootReadySpeed && Hood.isAimed() && m_vision.isAimed()) {
                m_kicker.setKicker(1.0);
            } else {
                m_kicker.setKicker(0);
            }
        } else { // fire even if not aiming

            m_shooter.shoot(kBottomShootSpeed, kTopShootSpeed);
            if (m_shooter.getBottomShooterRPS() >= kBottomShootReadySpeed
                    && m_shooter.getTopShooterRPS() >= kTopShootReadySpeed) {
                // Timer.delay(.5);
                m_kicker.setKicker(1.0);

            } else {
                m_kicker.setKicker(0);
            }
        }
        System.out.println(
                "t = " + m_shooter.getTopShooterRPS() + "\n b = " + m_shooter.getBottomShooterRPS());

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.shoot(0);
        m_kicker.setKicker(0);
        t.reset();
        System.out.println("done");
        AimCommand.kShooterSpeedFromAim = -1;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_stopAfterTime) {
            return t.get() >= time;
        } else {
            return false;
        }

        // return false;
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class ShootCommand extends CommandBase {
    // in Rev/Sec
    private double kTopShootSpeed;
    private double kTopShootReadySpeed;
    private double kBottomShootSpeed;
    private double kBottomShootReadySpeed;

    private final Shooter m_shooter;
    private final Vision m_vision;

    private double y; // ty from limelight
    private double v; // tv from limelight, # = # of targets

    private Timer t = new Timer(); // for ending shooting
    private static boolean stopShooterAfterTime; // for auto
    private static double time;

    private static int waitForAim; // 0 = false, 1 = true, 2 = only turret, 3 = only hood

    private static boolean useVariableSpeed;
    private static double shooterDesiredSpeed[] = { 0, 0 };

    private double shooterSpeed;

    public static double activateKicker = 0;

    public static double activateShooter[] = { 0, 0 }; // bottom speed, top speed

    public ShootCommand(Shooter shooter, Vision vision) {

        m_shooter = shooter;
        m_vision = vision;

        addRequirements(); // vision doesn't run any motors..., it just grabs values

        refreshAimValues();

        time = 4.0;
        shooterSpeed = 70;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        t.start();
        t.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        refreshAimValues();
        if (useVariableSpeed) {
            setShooterSpeed(getShooterSpeed());
        } else {
            setShooterSpeed(shooterDesiredSpeed[0], shooterDesiredSpeed[1]);
        }
        shootShooter();

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shoot(0, 0);
        activateKicker = 0;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (stopShooterAfterTime) {
            return t.get() >= time;
        } else {
            return false;
        }

    }

    private void shootShooter() {
        shoot(kBottomShootSpeed, kTopShootSpeed);
        if (waitForAim == 1) {
            if (AimCommand.isAimFinished()) { // both turret and hood aimed
                shootActivateKicker();
            }
        } else if (waitForAim == 2) {
            if (AimCommand.isTurretAimed()) { // only turret
                shootActivateKicker();
            }
        } else if (waitForAim == 3) {
            if (AimCommand.isHoodAimed()) { // only hood
                shootActivateKicker();
            }
        } else if (waitForAim == 0) { // neither is required
            shootActivateKicker();
        }
    }

    private void shoot(double bottomSpeed, double topSpeed) {
        activateShooter[0] = bottomSpeed;
        activateShooter[1] = topSpeed;
    }

    private void shootActivateKicker() {
        if (m_shooter.getBottomShooterRPS() >= kBottomShootReadySpeed
                && m_shooter.getTopShooterRPS() >= kTopShootReadySpeed) {
            activateKicker = 1;
        } else {
            activateKicker = 0;
        }
    }

    private void setShooterSpeed(double bottomspeed, double topSpeed) {
        kBottomShootReadySpeed = bottomspeed;
        kTopShootReadySpeed = topSpeed;

        // TODO all this should change so it works better (shooter characterization)
        if (kBottomShootReadySpeed <= 70) {
            kBottomShootSpeed = kBottomShootReadySpeed * (8.0 / 7); // bottom speed = 1/7 more than bottom ready speed, 80 rps
        } else { // kBottomShootReadySpeed > 70
            kBottomShootSpeed = kBottomShootReadySpeed * 1.2; // higher value so the ready speed will reach the speed it is aiming for
        }

        kTopShootSpeed = kTopShootReadySpeed * (8.0 / 7); // top speed = 1/7 more than top ready speed, 40 rps
    }

    private void setShooterSpeed(double speed) {
        setShooterSpeed(speed, (speed / 2)); // set topSpeed higher so less rotation
    }

    private double getShooterSpeed() {
        if (v == 1) {
            if (y > 10) {
                shooterSpeed = 65;
            } else if (y > 5.1) {
                shooterSpeed = 70;
            } else if (y > 1.9) {
                shooterSpeed = 75;
            } else if (y <= 1.9) {
                shooterSpeed = 80;
            }
        }

        return shooterSpeed;
    }

    private void refreshAimValues() {
        v = m_vision.getTv();
        y = m_vision.getTy();
    }

    //Settings called before calling shootCommand
    public static void useStandardAutoAim() {
        useVariableSpeed = true;
        waitForAim = 1; // true
        stopShooterAfterTime = false;
    }

    public static void useStandardAutoAimForAutonomous(double seconds) {
        useStandardAutoAim();
        stopShooterAfterTime = true;
        time = seconds;
    }

    public static void useLowGoal() {
        useVariableSpeed = false;
        shooterDesiredSpeed[0] = 23;
        shooterDesiredSpeed[1] = 11;
        waitForAim = 0;
        stopShooterAfterTime = false;
    }

    public static void custom(boolean useVarSpeed, double bottomSpeed, double topSpeed, int waitAim,
            double seconds) {

        useVariableSpeed = useVarSpeed;
        waitForAim = waitAim;

        if (!useVariableSpeed) {
            shooterDesiredSpeed = new double[] { bottomSpeed, topSpeed };
        }

        if (seconds > 0) {
            stopShooterAfterTime = true;
            time = seconds;
        } else {
            stopShooterAfterTime = false;
        }

    }

}

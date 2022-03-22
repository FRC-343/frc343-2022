package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class AimShootCommand extends CommandBase {
    // in Rev/Sec
    private double kTopShootSpeed;
    private double kTopShootReadySpeed;
    private double kBottomShootSpeed;
    private double kBottomShootReadySpeed;

    private final Shooter m_shooter;
    private final Kicker m_kicker;
    private final Hood m_hood;
    private final Turret m_turret;
    private final Vision m_vision;

    private final boolean m_stopShooterAfterTime; // for auto
    private final boolean m_lowGoal;
    private final boolean m_notUseColorSensor;

    private int m_aimShootMode;

    private Timer t = new Timer(); // for ending shooting
    private Timer t2 = new Timer(); // for color spitting
    private final double time;

    private double y; // ty from limelight
    private double x; // tx from limelight
    private double v; // tv from limelight, # = # of targets
    private double d; // horizontal distance to targe

    private final double goalHeight = 104; // inches
    private final double limeLightHeight = 31.5;
    private final double limeLightMountAngleToGround = 17; // degrees

    private double kTurretPrecision;
    private double kTurretSpeed;

    private double prev_heading_error;

    private double shooterSpeed;

    private int stepNumber; // used to keep track of where we are with complicated aimShootMode's

    public static boolean activateKicker = false; // true = run at 1.0

    public static double activateShooter[] = { 0, 0 }; // bottom speed, top speed

    // aimShootMode explanation:
    // ---------------------------------------------------------------------------------
    // -2 = aim, then shoot with no overlap
    // -1 = keep aiming the whole time, shoot when aimed (like 0, but still keep aiming)
    // 0 = aim while charging shooter, then shoot

    // 1 = aim only
    // 2 = aim only, and don't stop aiming
    // 3 = shoot only (no aiming or speed changing (default 70rps))
    // 4 = shoot only (with speed changing based on distance)

    public AimShootCommand(Shooter shooter, Kicker kicker, Hood hood, Turret turret, Vision vision, int aimShootMode,
            boolean stopShooterAfterTime, boolean lowGoal, boolean notUseColorSensor) {

        m_shooter = shooter;
        m_kicker = kicker;
        m_hood = hood;
        m_turret = turret;
        m_vision = vision;

        addRequirements(m_hood, m_turret, m_vision); // no kicker or shooter because overlap with intake, activate<subsysetm> tells the required subsystem to run so that it doesn't kill anything

        m_aimShootMode = aimShootMode;

        m_stopShooterAfterTime = stopShooterAfterTime;
        m_lowGoal = lowGoal;
        m_notUseColorSensor = notUseColorSensor;

        refreshAimValues();

        time = 4.0;
        shooterSpeed = 70;

    }

    public AimShootCommand(Shooter shooter, Kicker kicker, Hood hood, Turret turret, Vision vision, int aimShootMode) {
        this(shooter, kicker, hood, turret, vision, aimShootMode, false, false, false); // defaults to aim-shoot similtaneously, no stopping shooting, no low goal, and using the colorSensor
    }

    public AimShootCommand(Shooter shooter, Kicker kicker, Hood hood, Turret turret, Vision vision) {
        this(shooter, kicker, hood, turret, vision, -1); // defaults aim-shoot-mode
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        t.start();
        t.reset();

        stepNumber = 0;

        kTurretPrecision = 1.5;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // SmartDashboard.putNumber("distance in inches", d);
        System.out.println("Distance = " + d);
        refreshAimValues();

        if (m_aimShootMode == -2) {
            modeNeg2();
        } else if (m_aimShootMode == -1) {
            modeNeg1();
        } else if (m_aimShootMode == 0) {
            mode0();
        } else if (m_aimShootMode == 1) {
            mode1();
        } else if (m_aimShootMode == 2) {
            mode1(); // it uses the same mode, but has a different isFinished()
        } else if (m_aimShootMode == 3) {
            mode3();
        } else if (m_aimShootMode == 4) {
            mode4();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shoot(0, 0);
        activateKicker = false;
        t.stop();
        t.reset();
        stepNumber = 0;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_stopShooterAfterTime) {
            return t.get() >= time;
        } else {
            if (m_aimShootMode == 1) {
                return isAimFinished(); // be done when done aiming
            } else {
                return false; // never be done for all (besides 1) when m_stopShooterAfterTime = false
            }
        }

    }

    private void modeNeg2() { // aim, then shoot with no overlap
        if (stepNumber == 0) {
            aimHood();
            aimTurret();
            if (isAimFinished()) {
                stepNumber = 1;
                setShooterSpeed(getShooterSpeed());
            }
        } else {
            shootShooter(false);
        }

    }

    private void modeNeg1() { // keep aiming the whole time, shoot when aimed
        aimHood();
        aimTurret();
        setShooterSpeed(getShooterSpeed());
        shootShooter(true);
    }

    private void mode0() {
        if (stepNumber == 0) {
            aimHood();
            aimTurret();
            setShooterSpeed(getShooterSpeed());
            shootShooter(true); // spin up shooter wheels
            if (isAimFinished()) {
                stepNumber = 1;
            }
        } else {
            shootShooter(false); // not require to be aimed to fire
        }
    }

    private void mode1() { // aim only
        aimTurret();
        aimHood();
    }

    private void mode3() { // shoot only
        setShooterSpeed();
        shootShooter(false);
    }

    private void mode4() { // shoot only but with speed calculated by distance
        setShooterSpeed(getShooterSpeed());
        shootShooter(false);
    }

    private void shootShooter(boolean waitForAim) {
        if (!m_notUseColorSensor && m_kicker.isBadCargo()) {
            t2.reset();
            t2.start();
        }
        if (!m_notUseColorSensor && (m_kicker.isBadCargo() || t2.get() < .5)) {
            shootBadCargo();
        } else {
            shoot(kBottomShootSpeed, kTopShootSpeed);
            if (waitForAim) {
                if (isAimFinished()) {
                    shootActivateKicker();
                }
            } else {
                shootActivateKicker();
            }
        }
    }

    private void shoot(double bottomSpeed, double topSpeed) {
        activateShooter[0] = bottomSpeed;
        activateShooter[1] = topSpeed;
    }

    private void shootActivateKicker() {
        if (m_shooter.getBottomShooterRPS() >= kBottomShootReadySpeed
                && m_shooter.getTopShooterRPS() >= kTopShootReadySpeed) {
            activateKicker = true;
        } else {
            activateKicker = false;
        }
    }

    private void shootBadCargo() {
        shoot(13, 13);
        if (m_shooter.getBottomShooterRPS() <= 20 && m_shooter.getTopShooterRPS() <= 20) {
            activateKicker = true;
        } else {
            activateKicker = false;
        }
    }

    private void setShooterSpeed(double bottomspeed, double topSpeed) {
        if (!m_lowGoal) {
            kBottomShootReadySpeed = bottomspeed;
        } else { // lowGoal
            kBottomShootReadySpeed = 20;
        }

        if (kBottomShootReadySpeed <= 70) {
            kBottomShootSpeed = kBottomShootReadySpeed * (8.0 / 7); // bottom speed = 1/7 more than bottom ready speed, 80 rps
        } else { // kBottomShootReadySpeed > 70
            kBottomShootSpeed = kBottomShootReadySpeed * 1.2; // higher value so the ready speed will reach the speed it is aiming for
        }

        kTopShootReadySpeed = topSpeed; // top ready speed = 1/2 of bottom ready speed, 35 rps
        kTopShootSpeed = kTopShootReadySpeed * (8.0 / 7); // top speed = 1/7 more than top ready speed, 40 rps
    }

    private void setShooterSpeed(double speed) {
        setShooterSpeed(speed, speed / 2);
    }

    private void setShooterSpeed() {
        setShooterSpeed(65);
    }

    private double getShooterSpeed() {
        // speed formula will go here later, but make sure it rounds to avoid changing speeds slightly constantly
        if (v == 1) {
            if (d < 154) {
                shooterSpeed = 65;
            } else if (d < 180) {
                shooterSpeed = 70;
            } else if (d < 256) {
                shooterSpeed = 75;
            } else if (d >= 256) {
                shooterSpeed = 80;
            }
        }

        return shooterSpeed;
    }

    private void refreshAimValues() {
        x = m_vision.getTx();
        v = m_vision.getTv();
        y = m_vision.getTy(); // put distance formula in here later

        double angleFromGround = 0.01745329 * (y + limeLightMountAngleToGround); // find total angle and change to rad
        d = (goalHeight - limeLightHeight) / Math.tan(angleFromGround); // inches

    }

    private void refreshTurretPrecision(double speed) { // designed to get the precision based on speed (on distance)
        if (speed == 65) {
            kTurretPrecision = 2;
        } else if (speed == 70) {
            kTurretPrecision = 1.5;
        } else if (speed == 75) {
            kTurretPrecision = 1.0;
        } else if (speed >= 80) {
            kTurretPrecision = .5;
        }
    }

    private void aimTurret() {
        aimTurretSpeed();
        refreshTurretPrecision(getShooterSpeed());

        if (x > kTurretPrecision) {
            m_turret.spin(kTurretSpeed);
        } else if (x < -kTurretPrecision) {
            m_turret.spin(-kTurretSpeed);
        } else {
            m_turret.spin(0.0);
        }
    }

    private void aimTurretAlt() {
        kTurretPrecision = .5;
        double kTargetP = .01;
        double kTargetD = 0.0;
        
        if (Math.abs(x) > kTurretPrecision) {
            m_turret.spin(MathUtil.clamp(kTargetP * x + kTargetD * (x - prev_heading_error), -.5, .5));
        }

        prev_heading_error = x;

    }

    private void aimTurretSpeed() {
        double speed;
        speed = Math.abs(x) / 35.0; // equivilent to a PID (P only), goes proportionally slower the closer you are
        if (speed > .5) { // increase these to .5 if it doesn't break
            speed = .5;
        } else if (speed < .2) {
            speed = .2;
        }

        kTurretSpeed = speed;
    }

    private void aimHood() {
        if (v == 1) {
            if (d < 154) { // 65 rps
                m_hood.aim(0.2065 * d * d + -28.9906 * d + 1407.1326);
            } else if (d < 180) { // 70 rps
                m_hood.aim(0.4001 * d * d + -119.50488 * d + 9914.97874);
            } else if (d < 256) { // 75 rps
                m_hood.aim(-0.081 * d * d + 49.40145 * d + -4988.84872);
            } else if (d >= 256) { // 80 rps
                m_hood.aim(-0.6618 * d * d + 378.75 * d + -51490.58824);
            }
        }
    }

    private boolean isAimFinished() {
        return m_hood.isAimed() && m_vision.isAimed(kTurretPrecision) && v == 1;
    }
}

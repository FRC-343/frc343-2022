package frc.robot.commands;

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

    // private final double goalHeight = 104; // inches
    // private final double limeLightHeight = 31.5;
    // private final double limeLightMountAngleToGround = 17; // degrees

    private double kTurretPrecision;
    private double kTurretSpeed;

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
    // 5 = shoot only for tarmat edge
    // 6 = shoot only for safe hanger thingy
    // 7 = like -1 but accounts for x plane motion

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
        } else if (m_aimShootMode == 5) {
            mode5();
        } else if (m_aimShootMode == 6) {
            mode6();
        } else if (m_aimShootMode == 7) {
            mode7();
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
        aimTurret();
        setShooterSpeed(75);
        if (m_vision.isAimed(kTurretPrecision)) {
            shootShooter(false);
        }
    }

    private void mode4() { // shoot only but with speed calculated by distances
        setShooterSpeed(getShooterSpeed());
        shootShooter(false);
    }

    private void mode5() {
        m_hood.aim(Math.pow(Math.E, Math.PI) * 50); // 1150
        setShooterSpeed(70);
        aimTurret();
        if (isAimFinished()) {
            shootShooter(false);
        }
    }

    private void mode6() {
        m_hood.aim(1800); // 1800
        setShooterSpeed(70);
        aimTurret();
        if (isAimFinished()) {
            shootShooter(false);
        }
    }

    private void mode7() { // shooting while moving
        aimHood();
        aimTurretAlt();
        setShooterSpeed(getShooterSpeed());
        shootShooter(true);
    }

    private void shootShooter(boolean waitForAim) {
        // if (!m_notUseColorSensor && m_kicker.isBadCargo()) {
        // t2.reset();
        // t2.start();
        // }
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
        // if (m_shooter.getBottomShooterRPS() <= 20 && m_shooter.getTopShooterRPS() <= 20) {
        activateKicker = true;
        // } else {
        // activateKicker = false;
        // }
    }

    private void setShooterSpeed(double bottomspeed, double topSpeed) {
        if (!m_lowGoal) {
            kBottomShootReadySpeed = bottomspeed;
            kTopShootReadySpeed = topSpeed;
        } else { // lowGoal
            kBottomShootReadySpeed = 23;
            kTopShootReadySpeed = kBottomShootReadySpeed / 2;
        }

        if (kBottomShootReadySpeed <= 70) {
            kBottomShootSpeed = kBottomShootReadySpeed * (8.0 / 7); // bottom speed = 1/7 more than bottom ready speed, 80 rps
        } else { // kBottomShootReadySpeed > 70
            kBottomShootSpeed = kBottomShootReadySpeed * 1.2; // higher value so the ready speed will reach the speed it is aiming for
        }

        kTopShootSpeed = kTopShootReadySpeed * (8.0 / 7); // top speed = 1/7 more than top ready speed, 40 rps
    }

    private void setShooterSpeed(double speed) {
        setShooterSpeed(speed, (speed / 2));
    }

    private void setShooterSpeed() {
        setShooterSpeed(70);
    }

    private double getShooterSpeed() {
        // speed formula will go here later, but make sure it rounds to avoid changing speeds slightly constantly
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
        x = m_vision.getTx();
        v = m_vision.getTv();
        y = (m_vision.getTy() + 2); // put distance formula in here later
    }

    private void refreshTurretPrecision(double speed) { // designed to get the precision based on speed (on distance)
        if (speed == 65) {
            if (y > 6) {
                kTurretPrecision = 2; //was three
            } else {
                kTurretPrecision = 2.0;
            }
        } else if (speed == 70) {
            kTurretPrecision = 1.5;
        } else if (speed == 75) {
            kTurretPrecision = 1.0;
        } else if (speed >= 80) {
            kTurretPrecision = .5;
        }
    }

    private void aimTurret() {
        aimTurretMain(x);
    }

    private void aimTurretAlt() {
        double robotSpeed = Drive.getSpeed();

        double degreesOfTarget = m_vision.getThor() / 320 * 54;
        double kConstant = 3;
        double offset = degreesOfTarget * kConstant * robotSpeed;
        if (m_turret.getEncoder() > 125) {
            offset *= -1;
        }

        aimTurretMain(x - offset);
    }

    private void aimTurretMain(double tx) {
        aimTurretSpeed();
        refreshTurretPrecision(getShooterSpeed());

        if (tx > kTurretPrecision) {
            m_turret.spin(kTurretSpeed);
        } else if (tx < -kTurretPrecision) {
            m_turret.spin(-kTurretSpeed);
        } else {
            m_turret.spin(0.0);
        }
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
            if (y > 10) { // 65 rps
                m_hood.aim(-85.6237 * y + 2001); // possibly increase soon (originiall lowered this and 70rps by 300 to account for cargo inflation)
            } else if (y > 5.1) { // 70 rps
                m_hood.aim(27.6693 * y * y - 556.39365 * y + 3700); // -300, +122 (over lowered this value, or I messed with it at the end and it will mess things up)
            } else if (y > 1.9) { // 75 rps
                m_hood.aim(21.4843 * y * y - 291.0156 * y + 2500); // +185 (recently increased this becaues close to safezoon we be undershooting)
            } else if (y <= -1.9) { // 80 rps
                m_hood.aim(71.6124 * y * y - 353.6925 * y + 1984.3842); // sniper man good
            }
        }
    }

    private boolean isAimFinished() {
        return m_hood.isAimed() && m_vision.isAimed(kTurretPrecision) && v == 1;
    }
}

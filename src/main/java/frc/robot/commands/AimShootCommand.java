package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
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
    private Timer t2 = new Timer(); //for color spitting
    private final double time;

    private double y; // ty from limelight
    private double x; // tx from limelight
    private double v; // tv from limelight, # = # of targets
    private double d; // horizontal distance to targe

    private final double goalHeight = 96; // inches
    private final double limeLightHeight = 20;
    private final double limeLightMountAngleToGround = 20; // degrees

    private double kTurretPrecision;
    private double kTurretSpeed;

    private double shooterSpeed;

    private int stepNumber; // used to keep track of where we are with complicated aimShootMode's

    // aimShootMode explanation:
    // ---------------------------------------------------------------------------------
    // -2 = aim, then shoot with no overlap
    // -1 = keep aiming the whole time, shoot when aimed (like 0, but still keep aiming)
    // 0 = aim while charging shooter, then shoot

    // 1 = aim only
    // 2 = aim only, and don't stop aiming
    // 3 = shoot only (no aiming or speed changing (default 70rps))
    // 4 = shoot only (with speed changing based on distance)
    // 5 = shoot only (with speed for upper hub, when against lower hub)

    public AimShootCommand(Shooter shooter, Kicker kicker, Hood hood, Turret turret, Vision vision, int aimShootMode,
            boolean stopShooterAfterTime, boolean lowGoal, boolean notUseColorSensor) {

        m_shooter = shooter;
        m_kicker = kicker;
        m_hood = hood;
        m_turret = turret;
        m_vision = vision;

        addRequirements(m_shooter, m_kicker, m_hood, m_turret, m_vision);

        m_aimShootMode = aimShootMode;

        m_stopShooterAfterTime = stopShooterAfterTime;
        m_lowGoal = lowGoal;
        m_notUseColorSensor = notUseColorSensor;

        refreshAimValues();

        time = 5.0;
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
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.shoot(0);
        m_kicker.setKicker(0);
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

    private void mode5() { // shoot to high goal when next to goal (and hood all the way down)
        setShooterSpeed(65, 65); // TODO change speeds to get a high goal shot good
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
            m_shooter.shoot(kBottomShootSpeed, kTopShootSpeed);

            if (waitForAim) {
                if (isAimFinished()) {
                    shootActivateKicker();
                }
            } else {
                shootActivateKicker();
            }
        }

    }

    private void shootActivateKicker() {
        if (m_shooter.getBottomShooterRPS() >= kBottomShootReadySpeed
                && m_shooter.getTopShooterRPS() >= kTopShootReadySpeed) {
            m_kicker.setKicker(1.0);
        } else {
            m_kicker.setKicker(0);
        }
    }

    private void shootBadCargo() {
        m_shooter.shoot(13, 13);
        if (m_shooter.getBottomShooterRPS() <= 15 && m_shooter.getTopShooterRPS() <= 15) {
            m_kicker.setKicker(1.0);
        }
    }

    private void setShooterSpeed(double bottomspeed, double topSpeed) {
        if (!m_lowGoal) {
            kBottomShootReadySpeed = bottomspeed;
        } else { // lowGoal
            kBottomShootReadySpeed = 24;
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
        setShooterSpeed(70);
    }

    private double getShooterSpeed() {
        // speed formula will go here later, but make sure it rounds to avoid changing speeds slightly constantly
        if (v == 1) {
            if (y < 18 && y > -.5) {
                shooterSpeed = 70;
            } else { // if (y <= -.5)
                shooterSpeed = 75;
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

    private void refreshTurretPrecision() {
        if (d > 2400) {
            kTurretPrecision = .5;
        } else if (d > 1200) {
            kTurretPrecision = 1.0;
        } else if (d > 600) {
            kTurretPrecision = 1.5;
        } else {
            kTurretPrecision = 2.0;
        }
    }

    private void aimTurret() {
        aimTurretSpeed();

        if (x > kTurretPrecision) {
            m_turret.spin(kTurretSpeed);
        } else if (x < -kTurretPrecision) {
            m_turret.spin(-kTurretSpeed);
        } else {
            m_turret.spin(0.0);
        }
    }

    private void aimTurretSpeed() {
        kTurretSpeed = Math.abs(x) / 30.0; // equivilent to a PID, goes proportionally slower the closer you are
        if (kTurretSpeed > .4) { // increase these to .5 if it doesn't break
            kTurretSpeed = .4;
        } else if (kTurretSpeed < .2) {
            kTurretSpeed = .2;
        }
    }

    private void aimTurretSpeedAlternate() {
        double minTurretSpeed;
        if (kTurretPrecision <= .5) {
            minTurretSpeed = .1;
        } else if (kTurretPrecision <= 1.0) {
            minTurretSpeed = .15;
        } else if (kTurretPrecision <= 1.5) {
            minTurretSpeed = .2;
        } else {
            minTurretSpeed = 2.5;
        }

        if (Math.abs(x) <= kTurretPrecision * 2) {
            kTurretSpeed = minTurretSpeed;
        } else if (Math.abs(x) <= kTurretPrecision * 3) {
            kTurretSpeed = (minTurretSpeed + Robot.kMaxTurretSpeed) / 2;
        } else {
            kTurretSpeed = .4;
        }
    }

    private void aimHood() {
        if (y < 18 && y > -.5) {
            m_hood.aim(1000 + y * -52.5726);
        } else { // if (d <= -.5
            m_hood.aim(600.835 + y * -185.9704);
        }
    }

    private boolean isAimFinished() {
        return m_hood.isAimed() && m_vision.isAimed(kTurretPrecision) && v == 1;
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class AimCommand extends CommandBase {

    private final Hood m_hood;
    private final Turret m_turret;
    private final Vision m_vision;

    private double turretPrecision;
    private double turretSpeed;

    private double x;
    private double y;
    private double v;

    private static boolean isHoodAimed = false;
    private static boolean isTurretAimed = false;
    private static double isTarget = 0;

    public AimCommand(Hood hood, Turret turret, Vision vision) {
        refreshAimValues();
        m_hood = hood;
        m_turret = turret;
        m_vision = vision;

        addRequirements(m_hood, m_turret);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        refreshAimValues();
        refreshIsAimedValues();
        aimHood();
        aimTurretMain(x);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;

    }

    private void refreshAimValues() {
        x = m_vision.getTx();
        v = m_vision.getTv();
        y = m_vision.getTy(); // put distance formula in here later
    }

    private void refreshIsAimedValues() { //have to use the static values since isAimed needs to be static to access in ShootCommand
        isHoodAimed = m_hood.isAimed();
        isTurretAimed = m_vision.isAimed(turretPrecision);
        isTarget = v;
    }

    private void aimHood() {
        if (v == 1) {
            if (y > 10) { // 65 rps
                m_hood.aim(-85.6237 * y + 2001); // possibly increase soon (originiall lowered this and 70rps by 300 to account for cargo inflation)
            } else if (y > 5.1) { // 70 rps
                m_hood.aim(27.6693 * y * y - 556.39365 * y + 3700); // -300, +122 (over lowered this value)
            } else if (y > 1.9) { // 75 rps
                m_hood.aim(21.4843 * y * y - 291.0156 * y + 2500); // +185 (recently increased this becaues close to safezoon we be undershooting)
            } else if (y <= -1.9) { // 80 rps
                m_hood.aim(71.6124 * y * y - 353.6925 * y + 1984.3842); // sniper man good
            }
        }
    }

    private void aimTurretMain(double tx) {
        aimTurretSpeed();
        refreshTurretPrecision();

        if (tx > turretPrecision) {
            m_turret.spin(turretSpeed);
        } else if (tx < -turretPrecision) {
            m_turret.spin(-turretSpeed);
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

        turretSpeed = speed;
    }

    private void refreshTurretPrecision() { // designed to get the precision based on speed (on distance)
        turretPrecision = 1;
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

    public static boolean isAimFinished() {
        return isHoodAimed && isTurretAimed && isTarget == 1;
    }
    
    public static boolean isHoodAimed() {
        return isHoodAimed;
    }

    public static boolean isTurretAimed() {
        return isTurretAimed;
    }
}

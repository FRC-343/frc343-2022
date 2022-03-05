package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class AimCommand extends CommandBase {
    // private static final double kTargetP = -0.125;
    // private static final double kTargetD = -1.0; 
    // private static double prev_heading_error = 0.0;

    private final Vision m_vision;
    private final Hood m_hood;
    private final Turret m_turret;

    private double kTurretPrecision = 2.0; 
    private double kTurretSpeed = .3;

    public static double kShooterSpeedFromAim = -1.0;

    public AimCommand(Vision vision, Hood hooooooooood, Turret turret) {
        m_vision = vision;
        m_hood = hooooooooood;
        m_turret = turret;
        addRequirements(m_vision, m_hood, m_turret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double heading_error = m_vision.getTx();
        double x = m_vision.getTy();

        // if (Math.abs(heading_error) > 2.0) {
        // m_drive.drive(0, kTargetP * heading_error + kTargetD *
        // (heading_error-prev_heading_error));
        // }

        // prev_heading_error = heading_error;

        kTurretSpeed = Math.abs(heading_error) / 20.0; //equivilent to a PID, goes proportionally slower the closer you are
        if (kTurretSpeed > .4) { //increase these to 5 if it doesn't break
            kTurretSpeed = .4;
        } else if (kTurretSpeed < .2) {
            kTurretSpeed = .2;
        }

        if (heading_error > kTurretPrecision) {
            m_turret.spin(kTurretSpeed);
        } else if (heading_error < -kTurretPrecision) {
            m_turret.spin(-kTurretSpeed);
        } else {
            m_turret.spin(0.0);
        }

        if (x < 20 && x > -5) {
            m_hood.aim(4.5725 * x * x - 139.38 * x + 1015.2);
        } else { //if (x <= -5 && x > -12) {
            m_hood.aim(9.0476 * x * x + 0.9524 * x + 1008.6);
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_hood.move(0.0);
        m_turret.spin(0.0);
        double x = m_vision.getTy();
        if (x < 20 && x > -5) {
            kShooterSpeedFromAim = 70;
        } else { //if (x <= -5 && x > -12) {
            kShooterSpeedFromAim = 75;
        }
        System.out.println("Done with aiming");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Math.abs(m_vision.getTx()) < kTurretPrecision && Hood.isAimed());
    }

}

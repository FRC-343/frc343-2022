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

    private final double kTurretPrecision = 1.0; 
    private final double kTurretSpeed = .7;

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

        // if (Math.abs(heading_error) > 2.0) {
        // m_drive.drive(0, kTargetP * heading_error + kTargetD *
        // (heading_error-prev_heading_error));
        // }

        // prev_heading_error = heading_error;

        if (heading_error > kTurretPrecision) {
            m_turret.spin(kTurretSpeed);
        } else if (heading_error < -kTurretPrecision) {
            m_turret.spin(-kTurretSpeed);
        } else {
            m_turret.spin(0.0);
        }

        // m_hood.aim(m_vision.getTy()); 
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_hood.move(0.0);
        m_turret.spin(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Math.abs(m_vision.getTx()) < kTurretPrecision && Hood.isAimed());
    }

}

package frc.robot.commands.driveCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drive;

public class DriveTurnCommand extends CommandBase {
    private final Drive m_drive;
    private final double m_rot;
    private final double m_speed;

    private Pose2d m_startPose = new Pose2d();

    public DriveTurnCommand(double rot, double speed, Drive drive) {
        m_rot = rot;
        m_speed = speed;
        m_drive = drive;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_startPose = m_drive.getPose();

        m_drive.drive(0, m_speed);
    }

    @Override
    public void execute() {
        m_drive.drive(0, m_speed); //this makes us turn ccw if positive
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return m_drive.getPose().minus(m_startPose).getRotation().getDegrees() >= m_rot;
    }

}

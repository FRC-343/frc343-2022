package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drive;

public class DriveDistanceCommand extends CommandBase {
    private final Drive m_drive;

    private final double m_distance;
    private final double m_speed;
    private final double m_rot;
    private final boolean m_toSpin;

    private double kStartHeading;

    private Pose2d m_startPose = new Pose2d(0, 0, new Rotation2d(0));

    public DriveDistanceCommand(double distance, double speed, Drive drive, double rot, boolean toSpin) {
        m_distance = distance;
        m_speed = speed;
        m_drive = drive;
        m_rot = rot;
        m_toSpin = toSpin;
        kStartHeading = 0;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_startPose = m_drive.getPose();

        m_drive.drive(m_speed, 0);
        kStartHeading = m_drive.getHeading();
    }

    @Override
    public void execute() {
        if (m_toSpin) {
            m_drive.drive(m_speed, 1);

        } else {
            m_drive.drive(m_speed, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0);
    }

    @Override
    public boolean isFinished() {
        if (m_toSpin) {
             return Math.abs(m_drive.getHeading() - kStartHeading) >= m_rot;
        } else {
            return m_drive.getPose().minus(m_startPose).getTranslation().getNorm() >= m_distance;
        }

    }

}

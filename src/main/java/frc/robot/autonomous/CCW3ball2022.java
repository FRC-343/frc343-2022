package frc.robot.autonomous;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.AimCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TrajectoryCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class CCW3ball2022 extends SequentialCommandGroup {
    private final double maxSpeed;
    private final double maxAcceleration;

    public CCW3ball2022(Drive drive, Intake intake, Kicker kicker, Vision vision, Hood hood, Shooter shooter,
            Turret turret) {

        maxSpeed = Drive.kMaxSpeed / 4;
        maxAcceleration = Drive.kMaxAcceleration / 4;

        TrajectoryConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(drive.getRightFeedforward(),
                drive.getKinematics(), 11.0);

        // Create config for trajectory
        TrajectoryConfig forwardPickupConfig = new TrajectoryConfig(maxSpeed, maxAcceleration)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(drive.getKinematics())
                // Apply the voltage constraint
                .addConstraint(voltageConstraint);

        // Create config for trajectory
        TrajectoryConfig reversePickupConfig = new TrajectoryConfig(maxSpeed, maxAcceleration)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(drive.getKinematics())
                // Apply the voltage constraint
                .addConstraint(voltageConstraint).setReversed(true);

        // commands in this autonomous
        addCommands(
                // pickup trajectory 1st ball
                new ParallelDeadlineGroup(
                        new TrajectoryCommand(
                                TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(1.5 * Math.PI)), // 270 degrees
                                        List.of(),
                                        new Pose2d(-0.7, -1.0, new Rotation2d(.96)), forwardPickupConfig), // 55 Degrees
                                drive),
                        new IntakeCommand(intake, kicker)),
                // rotate towards second ball
                new TrajectoryCommand(
                        TrajectoryGenerator.generateTrajectory(new Pose2d(0.0, 0.0, new Rotation2d(.96)), List.of(),
                                new Pose2d(0.0, 0.0, new Rotation2d(.4)), forwardPickupConfig), // change to reverse if not work
                        drive),
                // fire 2
                new AimCommand(vision, hood, turret), new ShootCommand(shooter, kicker),
                //go to second ball
                new ParallelDeadlineGroup(
                        new TrajectoryCommand(
                                TrajectoryGenerator.generateTrajectory(new Pose2d(0.0, 0.0, new Rotation2d(.4)), // 23 degrees
                                        List.of(),
                                        new Pose2d(-2.6, 1.1, new Rotation2d(.4)), forwardPickupConfig), // 23 Degrees
                                drive),
                        new IntakeCommand(intake, kicker)),
                //fire last ball
                new AimCommand(vision, hood, turret), new ShootCommand(shooter, kicker));
    }
}

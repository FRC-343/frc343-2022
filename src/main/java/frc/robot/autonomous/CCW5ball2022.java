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
import frc.robot.commands.IntakeTimeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TrajectoryCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class CCW5ball2022 extends SequentialCommandGroup {

    public CCW5ball2022(Drive drive, Intake intake, Hopper hopper, Vision vision, Hood hood, Shooter shooter) {
        TrajectoryConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(drive.getRightFeedforward(),
                drive.getKinematics(), 11.0);

        // Create config for trajectory
        TrajectoryConfig forwardPickupConfig = new TrajectoryConfig(Drive.kMaxSpeed / 4, Drive.kMaxAcceleration / 4)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(drive.getKinematics())
                // Apply the voltage constraint
                .addConstraint(voltageConstraint);

        // Create config for trajectory
        TrajectoryConfig reversePickupConfig = new TrajectoryConfig(Drive.kMaxSpeed / 4, Drive.kMaxAcceleration / 4)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(drive.getKinematics())
                // Apply the voltage constraint
                .addConstraint(voltageConstraint).setReversed(true);


        // commands in this autonomous
        addCommands(
                // drop intake
                new InstantCommand(intake::lower, intake),
                // fire 1st cargo
                new AimCommand(vision, hood, drive), new ShootCommand(shooter, hopper),
                // pickup trajectory
                new ParallelDeadlineGroup(
                        new TrajectoryCommand(
                                TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, Rotation2d.fromDegrees(270)),
                                        List.of(new Translation2d(-0.72, -1.36),
                                                new Translation2d(-1.04, -1.16)),
                                        new Pose2d(-2.72, -0.36, Rotation2d.fromDegrees(188)), forwardPickupConfig), drive),
                        new IntakeCommand(intake, hopper, false)),
                // fire next two cargo 
                new AimCommand(vision, hood, drive), new ShootCommand(shooter, hopper),
                // trajectory to drive to terminal
                new ParallelDeadlineGroup(
                        new TrajectoryCommand(
                                TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, Rotation2d.fromDegrees(188)),
                                        List.of(new Translation2d(-3.7, -.4)),
                                        new Pose2d(-3.7, -.4, Rotation2d.fromDegrees(205)), forwardPickupConfig), drive),
                        new IntakeCommand(intake, hopper, false)),
                
                new IntakeTimeCommand(intake, hopper, false, 2),
                
                // drive backwards toward goal
                new TrajectoryCommand(
                        TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, Rotation2d.fromDegrees(205)), List.of(),
                                new Pose2d(4.32, .16, Rotation2d.fromDegrees(203)), reversePickupConfig), drive),
                // fire last 2 cargo for a full five cargo auto 
                new AimCommand(vision, hood, drive), new ShootCommand(shooter, hopper)
        );
    }
}

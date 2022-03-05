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
import frc.robot.commands.driveCommands.TrajectoryCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Turret;

public class CCW5ball2022Alt extends SequentialCommandGroup {

    public CCW5ball2022Alt(Drive drive, Intake intake, Kicker kicker, Vision vision, Hood hood, Shooter shooter, Turret turret) {
        TrajectoryConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
                drive.getRightFeedforward(),
                drive.getKinematics(), 11.0);

        // Create config for trajectory
        TrajectoryConfig forwardPickupConfig = new TrajectoryConfig(/*Drive.kMaxSpeed,
                Drive.kMaxAcceleration*/.5, .5)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(drive.getKinematics())
                        // Apply the voltage constraint
                        .addConstraint(voltageConstraint);

        // Create config for trajectory
        TrajectoryConfig reversePickupConfig = new TrajectoryConfig(/*Drive.kMaxSpeed,
                Drive.kMaxAcceleration*/ .5, .5)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(drive.getKinematics())
                        // Apply the voltage constraint
                        .addConstraint(voltageConstraint).setReversed(true);

        // commands in this autonomous
        addCommands(
                // drop intake
                new InstantCommand(intake::lower, intake),
                // drive to first cargo below/adove
                new ParallelDeadlineGroup(
                        new TrajectoryCommand(

                                TrajectoryGenerator.generateTrajectory(
                                        new Pose2d(0, 0, Rotation2d
                                                .fromDegrees(245)),
                                        List.of(),
                                        new Pose2d(-0.5, -1.32,
                                                Rotation2d.fromDegrees(
                                                        245)),
                                        forwardPickupConfig),
                                drive),

                        new IntakeCommand(intake, kicker)),
                // u turn backwards to the right/left so that turret can fire
                new ParallelDeadlineGroup(
                        new TrajectoryCommand(
                                TrajectoryGenerator.generateTrajectory(
                                        new Pose2d(-0.5, -1.32, Rotation2d
                                                .fromDegrees(245)),
                                        List.of(),
                                        new Pose2d(0.5, -1.0,
                                                Rotation2d.fromDegrees(
                                                        180)),
                                        reversePickupConfig),
                                drive),
                        new IntakeCommand(intake, kicker)),
                //Aim and Fire
                new AimCommand(vision, hood, turret), new ShootCommand(shooter, kicker, vision, false, true),
                //drive to terminal
                new ParallelDeadlineGroup(
                        new TrajectoryCommand(
                                TrajectoryGenerator.generateTrajectory(
                                        new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
                                        List.of(),
                                        new Pose2d(7.0, .33, Rotation2d.fromDegrees(170)),
                                        forwardPickupConfig),
                                drive),
                        new IntakeCommand(intake, kicker)),
                // u turn backwards down/up
                new ParallelDeadlineGroup(
                        new TrajectoryCommand(
                                TrajectoryGenerator.generateTrajectory(
                                        new Pose2d(0, 0, Rotation2d.fromDegrees(170)),
                                        List.of(new Translation2d(.99, 0)),
                                        new Pose2d(-0.99, -0.66, Rotation2d.fromDegrees(22)),
                                        reversePickupConfig),
                                drive),
                        new IntakeCommand(intake, kicker)),
                //drive closer to the final cargo / hub
                new TrajectoryCommand(
                        TrajectoryGenerator.generateTrajectory(new Pose2d(-0.99, -0.66, Rotation2d.fromDegrees(22)),
                                List.of(),
                                new Pose2d(3.3, 0.66, Rotation2d.fromDegrees(22)),
                                forwardPickupConfig),
                        drive),
                // fire 2 cargo 
                new AimCommand(vision, hood, turret), new ShootCommand(shooter, kicker, vision, false, true),
                //pick up final cargo
                new ParallelDeadlineGroup(
                        new TrajectoryCommand(
                                TrajectoryGenerator.generateTrajectory(
                                        new Pose2d(0, 0, Rotation2d.fromDegrees(22)),
                                        List.of(),
                                        new Pose2d(0.99, 0.66, Rotation2d.fromDegrees(22)),
                                        forwardPickupConfig),
                                drive),
                        new IntakeCommand(intake, kicker)),
                //fire final cargo
                /*new AimCommand(vision, hood, turret),*/ new ShootCommand(shooter, kicker, vision, false, true));
    }
}

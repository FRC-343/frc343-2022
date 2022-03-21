package frc.robot.autonomous;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AimShootCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PresetHoodCommand;
import frc.robot.commands.PresetTurretCommand;
import frc.robot.commands.driveCommands.TrajectoryCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class FourBallAuto extends SequentialCommandGroup {
  private final double maxSpeed;
  private final double maxAcceleration;

  public FourBallAuto(Drive drive, Intake intake, Kicker kicker, Vision vision, Hood hood, Shooter shooter,
      Turret turret) {

    maxSpeed = Drive.kMaxSpeed / 6;
    maxAcceleration = Drive.kMaxAcceleration;

    TrajectoryConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
        drive.getRightFeedforward(),
        drive.getKinematics(), 11.0);

    // Create config for trajectory
    TrajectoryConfig forwardPickupConfig = new TrajectoryConfig(maxSpeed, maxAcceleration)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(drive.getKinematics())
        // Apply the voltage constraint
        .addConstraint(voltageConstraint);

    // Create config for trajectory
    TrajectoryConfig backPickupConfig = new TrajectoryConfig(maxSpeed, maxAcceleration)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(drive.getKinematics())
        // Apply the voltage constraint
        .addConstraint(voltageConstraint).setReversed(true);

    // commands in this autonomous
    addCommands(
        new ParallelDeadlineGroup(
            new TrajectoryCommand(
                TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0, 0, new Rotation2d(0)), // technically we start facing 38 degrees up from 180
                    List.of(),
                    new Pose2d(1.2, 1.6, Rotation2d.fromDegrees(90)),
                    forwardPickupConfig),
                drive),
            new IntakeCommand(intake, kicker),
            new PresetHoodCommand(hood, 1300, true),
            new PresetTurretCommand(turret, 45, true)),
        // fire 2
        new AimShootCommand(shooter, kicker, hood, turret, vision, -1, true, false, true),
        // go to Terminal
        new ParallelDeadlineGroup(
            new TrajectoryCommand(
                TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(232)), // start at 0,0 now but the degrees are now fixed universally
                    List.of(),
                    new Pose2d(-3.1, -4.5, Rotation2d.fromDegrees(225)),
                    forwardPickupConfig),
                drive),
            new IntakeCommand(intake, kicker)),

        new ParallelDeadlineGroup(new WaitCommand(1), new IntakeCommand(intake, kicker)),
        // back up while aiming
        new ParallelDeadlineGroup(
            new TrajectoryCommand(
                TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(225)), // start at 0,0 now but the degrees are still fixed universally
                    List.of(),
                    new Pose2d(-0.5, 1.0, Rotation2d.fromDegrees(315)),
                    backPickupConfig),
                drive),
            new AimShootCommand(shooter, kicker, hood, turret, vision, 2, true, false, true)),
        // drive forward a little while aiming
        new ParallelDeadlineGroup(
            new TrajectoryCommand(
                TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(315)), // start at 0,0 now but the degrees are still fixed universally
                    List.of(),
                    new Pose2d(3.5, 0.0, Rotation2d.fromDegrees(0)),
                    forwardPickupConfig),
                drive),
            new AimShootCommand(shooter, kicker, hood, turret, vision, 2, true, false, true)),
        // fire 2 terminal cargo
        new AimShootCommand(shooter, kicker, hood, turret, vision, -1, true, false, true));

  }
}

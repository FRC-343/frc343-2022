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

  public CCW3ball2022(Drive drive, Intake intake, Kicker kicker, Vision vision, Hood hood, Shooter shooter, Turret turret) {
    TrajectoryConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(drive.getRightFeedforward(),
        drive.getKinematics(), 11.0);

    // Create config for trajectory
    TrajectoryConfig forwardPickupConfig = new TrajectoryConfig(Drive.kMaxSpeed/4, Drive.kMaxAcceleration/4)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(drive.getKinematics())
        // Apply the voltage constraint
        .addConstraint(voltageConstraint);

    // commands in this autonomous
    addCommands(
        // drop intake
        new InstantCommand(intake::lower, intake),
        // fire 1st cargo
        new AimCommand(vision, hood, turret), new ShootCommand(shooter, kicker, hood),
        // pickup trajectory
        // new ParallelDeadlineGroup(
            // new TrajectoryCommand(TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, Rotation2d.fromDegrees(270)), 
            //     List.of(new Translation2d(-0.67, -1.33), 
            //             new Translation2d(-1.11, -0.89),   
            //             new Translation2d(-3.2, -.20)
            //             ),
            //     new Pose2d(-3.0, .45, Rotation2d.fromDegrees(30)), forwardPickupConfig), drive),
            // new IntakeCommand(intake)
    //    ),
       new AimCommand(vision, hood, turret), new ShootCommand(shooter, kicker, hood)
    );
  }
}

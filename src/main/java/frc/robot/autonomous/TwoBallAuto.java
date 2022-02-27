package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.AimCommand;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class TwoBallAuto extends SequentialCommandGroup {
  private static final double kDriveDistance = 1.3;
  private static final double kDriveSpeed = 1;

  public TwoBallAuto(Drive drive, Intake intake, Kicker kicker, Vision vision, Hood hood, Shooter shooter,
      Turret turret) {
    // commands in this autonomous
    addCommands(
        // drop intake
        new InstantCommand(intake::lower, intake),
        // drive and intake
        new ParallelDeadlineGroup(
            new DriveDistanceCommand(kDriveDistance, kDriveSpeed, drive, 0.0, false),
            new IntakeCommand(intake, kicker)),
        // rotate
        new DriveDistanceCommand(0, 1, drive, 100, true),
        // aim
        new AimCommand(vision, hood, turret), new ShootCommand(shooter, kicker));
  }
}

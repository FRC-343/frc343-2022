package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.AimCommand;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class JustBackItUpAndShoot extends SequentialCommandGroup {
  private static final double kBackupDriveDistance = 0.5;
  private static final double kBackupDriveSpeed = -0.6;

  public JustBackItUpAndShoot(Drive drive, Intake intake, Kicker kicker, Vision vision, Hood hood, Shooter shooter, Turret turret) {
    // commands in this autonomous
    addCommands(
        // drop intake
        new InstantCommand(intake::lower, intake),
        // backup
        new DriveDistanceCommand(kBackupDriveDistance, kBackupDriveSpeed, drive),
        // aim
        new AimCommand(vision, hood, turret), new ShootCommand(shooter, kicker));
  }
}

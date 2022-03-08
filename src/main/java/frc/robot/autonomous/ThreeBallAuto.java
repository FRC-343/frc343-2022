package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.AimCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.driveCommands.DriveDistanceCommand;
import frc.robot.commands.driveCommands.DriveTurnCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class ThreeBallAuto extends SequentialCommandGroup {
    private static final double kDriveDistance = 1.4;
    private static final double kDriveSpeed = 1;

    public ThreeBallAuto(Drive drive, Intake intake, Kicker kicker, Vision vision, Hood hood, Shooter shooter,
            Turret turret) {
        // commands in this autonomous
        addCommands(
                // drop intake
                new InstantCommand(intake::lower, intake),
                // drive and intake
                new ParallelDeadlineGroup(
                        new DriveDistanceCommand(kDriveDistance, kDriveSpeed, drive),
                        new IntakeCommand(intake, kicker)),
                // new PresetTurretCommand(turret, 30)),
                // rotate
                new ParallelDeadlineGroup(
                        new DriveTurnCommand(-117, -kDriveSpeed, drive),
                        new IntakeCommand(intake, kicker)),
                // aim
                new AimCommand(vision, hood, turret), new ShootCommand(shooter, kicker, true, false),

                new ParallelDeadlineGroup(
                        new DriveDistanceCommand(2.8, kDriveSpeed, drive),
                        new AimCommand(vision, hood, turret, false),
                        new IntakeCommand(intake, kicker)),

                new ParallelDeadlineGroup(
                        new AimCommand(vision, hood, turret),
                        new IntakeCommand(intake, kicker)),

                new ShootCommand(shooter, kicker, true, false));

    }
}
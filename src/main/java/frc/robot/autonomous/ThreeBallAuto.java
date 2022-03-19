package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AimShootCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PresetHoodCommand;
import frc.robot.commands.PresetTurretCommand;
import frc.robot.commands.driveCommands.DriveDistanceCommand;
import frc.robot.commands.driveCommands.DriveTurnCommand;
import frc.robot.subsystems.Climbing;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class ThreeBallAuto extends SequentialCommandGroup {
    private static final double kDriveSpeed = 1;

    public ThreeBallAuto(Drive drive, Intake intake, Kicker kicker, Vision vision, Hood hood, Shooter shooter,
            Turret turret, Climbing climbing) {
        // commands in this autonomous
        addCommands(
                // do the first things while doing the turret preset
                // drop intake and climbing
                new InstantCommand(intake::lower, intake),
                // drive and intake
                new ParallelDeadlineGroup(
                        new DriveDistanceCommand(1.1, kDriveSpeed, drive),
                        new IntakeCommand(intake, kicker, shooter),
                        new PresetHoodCommand(hood, 1200, true),
                        new PresetTurretCommand(turret, 65, true)),
                // rotate
                new ParallelDeadlineGroup(
                        new DriveTurnCommand(83, kDriveSpeed, drive),
                        new IntakeCommand(intake, kicker, shooter)),
                // rotate turret while doing all the stuff above
                // aim and shoot
                new AimShootCommand(shooter, kicker, hood, turret, vision, -1, true, false, true),

                new DriveDistanceCommand(0.7, kDriveSpeed, drive),

                new ParallelDeadlineGroup(
                        new DriveDistanceCommand(1.3, kDriveSpeed, drive),
                        new IntakeCommand(intake, kicker, shooter)),

                new ParallelDeadlineGroup(
                        new WaitCommand(2),
                        new DriveTurnCommand(15, kDriveSpeed, drive),
                        new IntakeCommand(intake, kicker, shooter),
                        new PresetTurretCommand(turret, 70)),

                new AimShootCommand(shooter, kicker, hood, turret, vision, -1, true, false, true));

    }
}

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.commands.ShootingRelatingCommands.*;
import frc.robot.commands.ShootingRelatingCommands.SpecificCommands.*;
import frc.robot.commands.driveCommands.*;
import frc.robot.subsystems.*;

public class ThreeBallAuto extends SequentialCommandGroup {
    private static final double kDriveSpeed = 1;

    public ThreeBallAuto(Drive drive, Intake intake, Kicker kicker, Vision vision, Hood hood, Shooter shooter,
            Turret turret) {
        // commands in this autonomous
        addCommands(
                // drop intake
                new InstantCommand(intake::lower, intake),
                // drive and intake
                new ParallelDeadlineGroup(
                        new DriveDistanceCommand(1.1, kDriveSpeed, drive),
                        new IntakeCommand(intake),
                        new PresetHoodCommand(hood, 1000, true),
                        new PresetTurretCommand(turret, 65, true)),
                // rotate
                new ParallelDeadlineGroup(
                        new DriveTurnCommand(115, kDriveSpeed, drive),
                        new IntakeCommand(intake)),
                // rotate turret while doing all the stuff above
                // aim and shoot
                new AimShootCommandAuto(hood, turret, vision, shooter),

                new DriveDistanceCommand(0.7, kDriveSpeed, drive),

                new ParallelDeadlineGroup(
                        new DriveDistanceCommand(1.3, kDriveSpeed, drive),
                        new IntakeCommand(intake)),

                new ParallelDeadlineGroup(
                        new WaitCommand(2),
                        new DriveTurnCommand(15, kDriveSpeed, drive),
                        new IntakeCommand(intake),
                        new AimCommand(hood, turret, vision)),

                new AimShootCommandAuto(hood, turret, vision, shooter));

    }
}

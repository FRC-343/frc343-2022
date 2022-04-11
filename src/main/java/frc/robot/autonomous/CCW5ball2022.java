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
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Turret;

public class CCW5ball2022 extends SequentialCommandGroup {

    public CCW5ball2022(Drive drive, Intake intake, Kicker kicker, Vision vision, Hood hood, Shooter shooter,
            Turret turret, Climbing m_climbing) {

        // commands in this autonomous
        addCommands(
                // drop intake
                new InstantCommand(intake::lower, intake),
                // drive and intake
                new ParallelDeadlineGroup(
                        new DriveDistanceCommand(1.1, 3, drive),
                        new IntakeCommand(intake),
                        new PresetHoodCommand(hood, 1000, true),
                        new PresetTurretCommand(turret, 0, true)),
                // rotate
                new ParallelDeadlineGroup(
                        new DriveTurnCommand(68, 2, drive), //was 56
                        new IntakeCommand(intake),
                        new PresetTurretCommand(turret, 10, true)),

                // aim
                new ParallelDeadlineGroup(
                        new AimShootCommand(shooter, kicker, hood, turret, vision, -1, true,
                                false, true),
                        new IntakeCommand(intake)),
                new DriveDistanceCommand(4.0, 3, drive),
                new ParallelDeadlineGroup(
                        new DriveDistanceCommand(1.6, 1, drive),
                        new IntakeCommand(intake)),
                new ParallelDeadlineGroup(
                        new WaitCommand(1.0),
                        new IntakeCommand(intake)),

                new ParallelDeadlineGroup(
                        new DriveTurnCommand(100, 2, drive),
                        new PresetTurretCommand(turret, 60, true),
                        new IntakeCommand(intake)),
                new PresetTurretCommand(turret, 80, true),
                new ParallelDeadlineGroup(
                        new DriveDistanceCommand(3.5, 3, drive),
                        new IntakeCommand(intake),
                        new AimShootCommand(shooter, kicker, hood, turret, vision, 2)),

                new ParallelDeadlineGroup(
                        new AimShootCommand(shooter, kicker, hood, turret, vision, -1, false,
                                false, true),
                        new IntakeCommand(intake))

        );
    }
}

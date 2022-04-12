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

public class TwoBallAuto extends SequentialCommandGroup {

    public TwoBallAuto(Drive drive, Intake intake, Kicker kicker, Vision vision, Hood hood, Shooter shooter,
            Turret turret) {
        // commands in this autonomous
        addCommands(
                // drop intake
                new InstantCommand(intake::lower, intake),
                // drive and intake
                new ParallelDeadlineGroup(
                        new DriveDistanceCommand(1.1, 3, drive),
                        new IntakeCommand(intake),
                        new PresetHoodCommand(hood, 1100, true),
                        new PresetTurretCommand(turret, 30, true)),
                // rotate
                new ParallelDeadlineGroup(
                        new DriveTurnCommand(110, 2, drive),
                        new AimShootCommandAuto(hood, turret, vision, shooter),
                        new IntakeCommand(intake)),

                new WaitCommand(1),
                new ParallelDeadlineGroup(
                        new AimShootCommandAuto(hood, turret, vision, shooter),
                        new IntakeCommand(intake))

        );
    }
}

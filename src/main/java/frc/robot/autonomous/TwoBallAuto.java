package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

public class TwoBallAuto extends SequentialCommandGroup {

    public TwoBallAuto(Drive drive, Intake intake, Kicker kicker, Vision vision, Hood hood, Shooter shooter,
            Turret turret, Climbing climbing) {
        // commands in this autonomous
        addCommands(
                // drop intake
                new InstantCommand(intake::lower, intake),
                // drive and intake
                new ParallelDeadlineGroup(
                        new DriveDistanceCommand(1.1, 3, drive),
                        new IntakeCommand(intake, kicker, shooter),
                        new PresetHoodCommand(hood, 1100, true),
                        new PresetTurretCommand(turret, 90, true)
                        ),
                // rotate
                new ParallelDeadlineGroup(
                        new DriveTurnCommand(80, 2, drive),
                        new IntakeCommand(intake, kicker, shooter)),

                // aim
                new AimShootCommand(shooter, kicker, hood, turret, vision, -1, true, false, true),
                new ParallelDeadlineGroup(
                        new DriveDistanceCommand(3, 1, drive), 
                        new IntakeCommand(intake, kicker, shooter)
                ));
                // new AimShootCommand(shooter, kicker, hood, turret, vision, -1, true, false, true));
    }
}

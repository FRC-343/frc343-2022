package frc.robot.commands.ShootingRelatingCommands.SpecificCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootingRelatingCommands.AimCommand;
import frc.robot.commands.ShootingRelatingCommands.ShootCommand;
import frc.robot.subsystems.*;

public class AimShootMoveCommand extends SequentialCommandGroup {

  public AimShootMoveCommand(Hood h, Turret t, Vision v, Shooter s) {

    addCommands(
        new InstantCommand(AimCommand::useMovingAutoAim),
        new InstantCommand(ShootCommand::useStandardAutoAim),
        new ParallelDeadlineGroup(
            new ShootCommand(s, v),
            new AimCommand(h, t, v))

    );
  }
}

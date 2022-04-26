package frc.robot.commands.ShootingRelatingCommands.SpecificCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootingRelatingCommands.ShootCommand;

public class ShootSpecificSpeedCommand extends SequentialCommandGroup {

  public ShootSpecificSpeedCommand(double bottomSpeed, double topSpeed) {
    addCommands(
        new InstantCommand(() -> ShootCommand.useCustom(false, bottomSpeed, topSpeed, 0, 0)),
        new ShootCommand()
    );
  }

  public ShootSpecificSpeedCommand() {
    this(70, 40);
  }
}

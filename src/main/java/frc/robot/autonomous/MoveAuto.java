package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.SimpleDriveCommand;
import frc.robot.subsystems.Drive;

public class MoveAuto extends SequentialCommandGroup {
  private static final double kDriveTime = 3;
  private static final double kDriveSpeed = 1;
  private static final double kWaitTime = 10;

  public MoveAuto(Drive drive) {
    // commands in this autonomous
    addCommands(new SimpleDriveCommand(drive, kDriveSpeed, kDriveTime, kWaitTime)); //wait 10 seconds, then drive forward at 1m/s for 2 seconds

  }
}

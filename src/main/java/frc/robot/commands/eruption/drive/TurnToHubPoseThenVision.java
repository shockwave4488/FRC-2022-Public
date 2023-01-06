package frc.robot.commands.eruption.drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TurnToHubPoseThenVision extends SequentialCommandGroup {
  public TurnToHubPoseThenVision(
      SwerveTurnToHub hubCentricSwerveDriveCommand,
      VisionAlignToTarget visionAlignToTargetCommand) {
    super(hubCentricSwerveDriveCommand, visionAlignToTargetCommand);
  }
}

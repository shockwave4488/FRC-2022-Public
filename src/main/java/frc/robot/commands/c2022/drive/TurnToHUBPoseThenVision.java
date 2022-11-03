package frc.robot.commands.c2022.drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.SwerveDrive;

public class TurnToHUBPoseThenVision extends SequentialCommandGroup {

  public TurnToHUBPoseThenVision(
      SwerveTurnToHUB hubCentricSwerveDriveCommand,
      VisionAlignToTarget visionAlignToTargetCommand,
      SwerveDrive swerve) {
    addRequirements(swerve);
    addCommands(hubCentricSwerveDriveCommand, visionAlignToTargetCommand);
  }
}

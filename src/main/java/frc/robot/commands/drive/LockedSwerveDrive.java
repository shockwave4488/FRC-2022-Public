package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.SwerveDrive;

public class LockedSwerveDrive extends CommandBase {
  private final SwerveDrive swerve;

  public LockedSwerveDrive(SwerveDrive swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    swerve.drive(0, 0, 0, true);
  }
}

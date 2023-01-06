package frc.robot.commands.c2022.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.SwerveDrive;

public class LockedSwerveDrive extends CommandBase {
  public enum LockedMode {
    Stop,
    XShape,
    Offset
  }

  private final SwerveDrive swerve;
  private LockedMode mode;
  private boolean forceMode = true;
  private final SendableChooser<LockedMode> modeSelector = new SendableChooser<>();

  /**
   * Drive setting preventing driver input, while locking robot in place.
   *
   * @param swerve SwerveDrive subsystem
   * @param mode Wheel behavior mode: | Stop = Hold current position | XShape = Hold wheel cross
   *     position (wheels pointed to center) | 3 = Hold position with wheels 45 degrees off from
   *     each other
   */
  public LockedSwerveDrive(SwerveDrive swerve, LockedMode mode) {
    this.swerve = swerve;
    this.mode = mode;
    addRequirements(swerve);
  }

  public LockedSwerveDrive(SwerveDrive swerve) {
    this(swerve, LockedMode.Stop);
  }

  public LockedSwerveDrive(SwerveDrive swerve, String selectorName) {
    this(swerve);
    forceMode = false;
    modeSelector.setDefaultOption("Current position", LockedMode.Stop);
    modeSelector.addOption("X-position", LockedMode.XShape);
    modeSelector.addOption("Offset positions", LockedMode.Offset);
    SmartDashboard.putData("LockedSwerveDrive " + selectorName, modeSelector);
  }

  @Override
  public void initialize() {
    if (!forceMode) {
      mode = modeSelector.getSelected();
    }

    switch (mode) {
      case XShape:
        swerve.setModuleStates(
            new SwerveModuleState[] {
              new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
              new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)),
              new SwerveModuleState(0, new Rotation2d(Math.PI * 3 / 4)),
              new SwerveModuleState(0, new Rotation2d(-Math.PI * 3 / 4))
            });
        break;
      case Offset:
        swerve.setModuleStates(
            new SwerveModuleState[] {
              new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)),
              new SwerveModuleState(0, new Rotation2d(0)),
              new SwerveModuleState(0, new Rotation2d(-Math.PI / 2)),
              new SwerveModuleState(0, new Rotation2d(-Math.PI * 3 / 4))
            });
        break;
      case Stop:
      default:
        break;
    }
  }
}

package frc.robot.commands.eruption.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.sensors.NavX;
import frc.lib.sensors.vision.Limelight;
import frc.lib.sensors.vision.VisionCamera.CameraPositionConstants;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.eruption.Shooter;
import java.util.function.Supplier;

/** Unused shooter charge class, instead use SpinFlywheel with false for {@code end} */
public class ShooterChargeDist extends CommandBase {
  private final Shooter shooter;
  // private final NavX gyro;
  private final Limelight limelight;
  private final Supplier<Pose2d> currentPoseSupplier;
  private final double powOfCamHUBHeightDiff; // meters
  private final double aRPMInterpConstant;
  private final double bRPMInterpConstant;
  // private final double aRPMToHoodInterpConstant;
  // private final double bRPMToHoodInterpConstant;
  private Pose2d currentPose;
  private double translationDistToShooter; // meters
  private double threeDDistToShooter; // meters
  private double desiredRPM = -1;
  private double desiredHoodPosition = -1;

  private double yOffset;

  public ShooterChargeDist(
      Shooter shooter,
      NavX gyro,
      Limelight limelight,
      Supplier<Pose2d> currentPoseSupplier,
      double aRPMInterpConstant,
      double bRPMInterpConstant,
      double aRPMToHoodInterpConstant,
      double bRPMToHoodInterpConstant) {
    this.shooter = shooter;
    // this.gyro = gyro;
    this.limelight = limelight;
    this.currentPoseSupplier = currentPoseSupplier;
    CameraPositionConstants limelightConstants = limelight.getCameraPositionConsts();
    powOfCamHUBHeightDiff =
        Math.pow(
            Units.inchesToMeters(
                FieldConstants.TARGET_HEIGHT_METERS - limelightConstants.camHeight),
            2);
    this.aRPMInterpConstant = aRPMInterpConstant;
    this.bRPMInterpConstant = bRPMInterpConstant;
    // this.aRPMToHoodInterpConstant = aRPMToHoodInterpConstant;
    // this.bRPMToHoodInterpConstant = bRPMToHoodInterpConstant;
  }

  @Override
  public void execute() {
    if (limelight.hasTargets()) {
      yOffset = -limelight.getBestTarget().get().getY().getDegrees();
      desiredRPM = shooter.getRPMFromYOffset(yOffset);
      desiredHoodPosition = shooter.getHoodLevelFromRPM(desiredRPM);
    } else {
      currentPose = currentPoseSupplier.get();
      translationDistToShooter =
          currentPose.getTranslation().getDistance(Constants.FieldConstants.HUB_CENTER);
      SmartDashboard.putNumber("ShooterDefault Translation To Hub", translationDistToShooter);
      threeDDistToShooter = Math.sqrt(translationDistToShooter + powOfCamHUBHeightDiff);
      SmartDashboard.putNumber("Shooter Default 3D Dist", threeDDistToShooter);
      if (threeDDistToShooter < 2) {
        desiredRPM = 1900;
        desiredHoodPosition = 30;
      } else {
        desiredRPM = (threeDDistToShooter * aRPMInterpConstant) + bRPMInterpConstant;
        desiredHoodPosition = shooter.getHoodLevelFromRPM(desiredRPM);
        // desiredHoodPosition = (desiredRPM * aRPMToHoodInterpConstant) + bRPMToHoodInterpConstant;
      }
    }

    SmartDashboard.putNumber("DefaultShoot DesiredRPM", desiredRPM);
    SmartDashboard.putNumber("DefaultShoot DesiredHoodPosition", desiredHoodPosition);
    shooter.setRPM(desiredRPM, 1, 10000);
    shooter.setHoodPosition(desiredHoodPosition);
  }
}

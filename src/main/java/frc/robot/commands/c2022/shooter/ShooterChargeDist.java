package frc.robot.commands.c2022.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.sensors.Limelight;
import frc.lib.sensors.Limelight.DistanceEstimationConstants;
import frc.lib.sensors.NavX;
import frc.robot.Constants;
import frc.robot.subsystems.c2022.Shooter;
import java.util.function.Supplier;

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
  private static final double INCHES_TO_METERS = 0.0254;
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
      DistanceEstimationConstants limelightConstants,
      Supplier<Pose2d> currentPoseSupplier,
      double aRPMInterpConstant,
      double bRPMInterpConstant,
      double aRPMToHoodInterpConstant,
      double bRPMToHoodInterpConstant) {
    this.shooter = shooter;
    // this.gyro = gyro;
    this.limelight = limelight;
    this.currentPoseSupplier = currentPoseSupplier;
    powOfCamHUBHeightDiff =
        Math.pow(
            (limelightConstants.targetHeight - limelightConstants.camHeight) * INCHES_TO_METERS, 2);
    this.aRPMInterpConstant = aRPMInterpConstant;
    this.bRPMInterpConstant = bRPMInterpConstant;
    // this.aRPMToHoodInterpConstant = aRPMToHoodInterpConstant;
    // this.bRPMToHoodInterpConstant = bRPMToHoodInterpConstant;
  }

  @Override
  public void execute() {
    if (limelight.hasTarget()) {
      yOffset = limelight.getY();
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

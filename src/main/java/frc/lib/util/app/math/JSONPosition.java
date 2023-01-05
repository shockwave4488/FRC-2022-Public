// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.app.math;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class JSONPosition {
  public double X;
  public double Y;
  public double Z;
  public double Roll;
  public double Yaw;
  public double Pitch;
  public String DistType = "meters";
  public String AngleType = "degrees";

  public Transform3d toTransform() {
    return new Transform3d(
        new Translation3d(toMeters(X), toMeters(Y), toMeters(Z)),
        new Rotation3d(toRadians(Roll), toRadians(Pitch), toRadians(Yaw)));
  }

  private double toMeters(double value) {
    switch (DistType) {
      case "meters":
        return value;
      case "inches":
        return Units.inchesToMeters(value);
      default:
        throw new RuntimeException("Unsupported distance type: " + DistType);
    }
  }

  private double toRadians(double value) {
    switch (DistType) {
      case "radians":
        return value;
      case "degrees":
        return Math.toRadians(value);
      default:
        throw new RuntimeException("Unsupported angle type: " + DistType);
    }
  }
}

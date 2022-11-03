package frc.robot.commands.c2022.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.sensors.NavX;
import frc.robot.subsystems.drive.SwerveDrive;

public class RotateToAngle extends CommandBase {

  private final SwerveDrive swerve;
  private static final double TURN_P = 0.1;
  private static final double TURN_I = 0.001;
  private static final double TURN_D = 0;
  private PIDController turnPID = new PIDController(TURN_P, TURN_I, TURN_D);
  private double desiredAngle;
  private NavX gyro;
  private double currentAngle;
  private int doneCounter = 0;
  private static final double ANGLE_RANGE = 4;
  private static final double DONE_COUNTER_MIN = 10;

  public RotateToAngle(SwerveDrive swerve, NavX gyro, double desiredAngle) {

    this.swerve = swerve;
    addRequirements(swerve);

    this.desiredAngle = desiredAngle;
    this.gyro = gyro;

    turnPID.enableContinuousInput(-180, 180);
    turnPID.setTolerance(DONE_COUNTER_MIN);
  }

  @Override
  public void execute() {
    currentAngle = gyro.getYaw().getDegrees();
    turnPID.setSetpoint(desiredAngle);
    double rotSpeed = turnPID.calculate(currentAngle);
    swerve.drive(0, 0, rotSpeed, true);

    SmartDashboard.putNumber("Current Angle", currentAngle);
    SmartDashboard.putNumber("Desired Angle", desiredAngle);
  }

  @Override
  public boolean isFinished() {
    double angleDifference = Math.abs(currentAngle - desiredAngle);
    SmartDashboard.putNumber("Angle Difference", angleDifference);
    if (angleDifference < ANGLE_RANGE) {
      doneCounter++;
    } else {
      doneCounter = 0;
    }
    return (doneCounter > DONE_COUNTER_MIN);
  }
}

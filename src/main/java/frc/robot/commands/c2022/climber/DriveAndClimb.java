package frc.robot.commands.c2022.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.sensors.NavX;
import frc.robot.commands.c2022.defaults.DefaultSwerveDrive;
import frc.robot.commands.c2022.drive.LockedSwerveDrive;
import frc.robot.subsystems.c2022.Climber;
import frc.robot.subsystems.drive.SwerveDrive;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveAndClimb extends SequentialCommandGroup {
  private ClimbMotionTest climbMotionTest;
  private ClimberLiftToHeight climberLiftToHeight;

  public DriveAndClimb(
      SwerveDrive swerve,
      Climber climber,
      NavX gyro,
      int desiredFirstHeight,
      DoubleSupplier power,
      BooleanSupplier doneDriving,
      BooleanSupplier abort,
      DefaultSwerveDrive defaultSwerveDrive) {
    climberLiftToHeight = new ClimberLiftToHeight(climber, desiredFirstHeight, abort);
    climbMotionTest = new ClimbMotionTest(climber, power);

    addCommands(
        climberLiftToHeight,
        new WaitUntilCommand(doneDriving).deadlineWith(defaultSwerveDrive),
        climbMotionTest.alongWith(new LockedSwerveDrive(swerve)));
  }
}

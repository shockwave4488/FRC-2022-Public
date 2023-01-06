package frc.robot.commands.eruption.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.eruption.Shooter;

public class ShooterWait extends CommandBase {
  private final Shooter shooter;
  private static final double COAST_RPM = 0;
  private static final double RESTING_HOOD_POS = 32;

  public ShooterWait(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
    SmartDashboard.putNumber("ShooterWaitRPM", COAST_RPM);
    SmartDashboard.putNumber("ShooterWaitHoodValue", RESTING_HOOD_POS);
  }

  public void initialize() {
    shooter.setRPM(COAST_RPM);
    shooter.setHoodPosition(RESTING_HOOD_POS);
  }

  @Override
  public void execute() {
    shooter.setRPM(SmartDashboard.getNumber("ShooterWaitRPM", COAST_RPM));
    shooter.setHoodPosition(SmartDashboard.getNumber("ShooterWaitHoodValue", RESTING_HOOD_POS));
  }
}

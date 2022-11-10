package frc.robot.commands.c2022.climber;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.flowcontrol.EdgeTrigger;
import frc.robot.subsystems.c2022.Climber;

public class ClimberHighRungPitchless extends CommandBase {
  private final Climber climber;
  private HighRungClimbState state;
  private int minCountCycles = 10;
  private int errorEpsilon = 5000;
  private double startingTime = -99999;
  private double waitTime = 1;
  private EdgeTrigger trigger;

  static enum HighRungClimbState {
    LowerToMid(261000),
    PullMid(-40000),
    ReleaseMid(150000),
    Done(150000);

    int height;

    /** @param height The tick value you want to the climber to be at */
    private HighRungClimbState(int height) {
      this.height = height;
    }
  }

  public ClimberHighRungPitchless(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    trigger = new EdgeTrigger(false);
    startingTime = -99999;
    state = HighRungClimbState.LowerToMid;
    climber.setClimberPosition(state.height, minCountCycles, errorEpsilon);
  }

  @Override
  public void execute() {
    SmartDashboard.putString("Pitchless climb state", state.toString());
    switch (state) {
      case LowerToMid:
        if (climber.isStable()) {
          state = HighRungClimbState.PullMid;
          setClimberPos();
        }
        break;
      case PullMid:
        if (climber.isStable()) {
          if (trigger.getRisingUpdate(true)) {
            startingTime = Timer.getFPGATimestamp();
          }
          if (Timer.getFPGATimestamp() > startingTime + waitTime) {
            state = HighRungClimbState.ReleaseMid;
            setClimberPos();
          }
        }
        break;
      case ReleaseMid:
        if (climber.isStable()) {
          state = HighRungClimbState.Done;
          setClimberPos();
        }
        break;
      case Done:
        climber.setClimberNeutralState(NeutralMode.Brake);
        break;
    }
  }

  private void setClimberPos() {
    climber.setClimberPosition(state.height, minCountCycles, errorEpsilon);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

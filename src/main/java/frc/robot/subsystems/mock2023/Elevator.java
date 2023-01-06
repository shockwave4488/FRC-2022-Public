package frc.robot.subsystems.mock2023;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.sensors.HallEffect;
import frc.lib.wpiextensions.ShockwaveSubsystemBase;

public class Elevator extends ShockwaveSubsystemBase {

  private static final int ELEVATOR_TOP_LIMIT = 500; // Update after testing
  private static final int ELEVATOR_BOTTOM_LIMIT = 0; // This will probably stay the same
  private HallEffect hallEffect;
  private Solenoid candyGrabber;
  private TalonSRX elevator;
  private boolean grabberClosed;
  private NeutralMode elevatorNeutralMode;
  private static final int REALLY_BIG_NUMBER = 999999;

  private int desiredTicks = 0;

  public Elevator(int hallEffectID, int candyGrabberID, int elevatorID, int PCM_ID) {
    hallEffect = new HallEffect(hallEffectID);
    candyGrabber = new Solenoid(PCM_ID, PneumaticsModuleType.CTREPCM, candyGrabberID);
    elevator = new TalonSRX(elevatorID);
    elevatorNeutralMode = NeutralMode.Brake;
    setElevatorNeutralState(elevatorNeutralMode);
    elevator.configFactoryDefault();
    elevator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    elevator.config_kF(0, 0);
    elevator.config_kP(0, .1);
    elevator.config_kI(0, 0);
    elevator.config_kD(0, 0);
    elevator.setSensorPhase(true);
  }

  public boolean getHallEffect() {
    return hallEffect.get();
  }

  public void openCandyGrabber() {
    grabberClosed = false;
  }

  public void closeCandyGrabber() {
    grabberClosed = true;
  }

  public boolean getCandyGrabberStatus() {
    return grabberClosed;
  }

  public int getElevatorPosition() {
    return (int) elevator.getSelectedSensorPosition();
  }

  public int getUpperLimit() {
    return ELEVATOR_TOP_LIMIT;
  }

  public int getLowerLimit() {
    return ELEVATOR_BOTTOM_LIMIT;
  }

  public void setElevatorDown() {
    setElevatorPosition(ELEVATOR_BOTTOM_LIMIT, 0, REALLY_BIG_NUMBER);
  }

  public void setElevatorUp() {
    setElevatorPosition(ELEVATOR_TOP_LIMIT, 0, REALLY_BIG_NUMBER);
  }

  public void setElevatorPosition(int ticks, int minCountCycles, int epsilon) {
    /* Stop the robot from moving arm any further down if it's about to destroy the robot while
    still allowing it to move the arm back up */
    ticks = Math.max(ticks, ELEVATOR_BOTTOM_LIMIT);
    ticks = Math.min(ticks, ELEVATOR_TOP_LIMIT);

    desiredTicks = ticks;
  }

  public void setElevatorNeutralState(NeutralMode state) {
    elevatorNeutralMode = state;
    elevator.setNeutralMode(state);
  }

  @Override
  public void onStart() {}

  @Override
  public void periodic() {
    if (getHallEffect()) {
      elevator.setSelectedSensorPosition(0);
    }
    elevator.set(ControlMode.Position, desiredTicks);
    SmartDashboard.putNumber("elevatordesiredTicks", desiredTicks);
    candyGrabber.set(grabberClosed);
  }

  @Override
  public void onStop() {
    elevator.set(ControlMode.Disabled, 0);
  }

  @Override
  public void zeroSensors() {
    if (getHallEffect() == true) {
      elevator.setSelectedSensorPosition(0);
    }
  }

  @Override
  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Elevator Position Ticks", getElevatorPosition());
    SmartDashboard.putNumber("Elevator Desired Ticks", desiredTicks);
    SmartDashboard.putBoolean("HallEffect", getHallEffect());
    SmartDashboard.putBoolean("Grabber", getCandyGrabberStatus());
  }

  @Override
  public void setUpTrackables() {}
}

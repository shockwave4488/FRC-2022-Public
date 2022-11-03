package frc.robot.subsystems.c2022;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.lib.wpiextensions.ShockwaveSubsystemBase;

public class SmartPCM extends ShockwaveSubsystemBase {

  private final Compressor compressor;

  public SmartPCM(int PCM_ID) {
    compressor = new Compressor(PCM_ID, PneumaticsModuleType.CTREPCM);
  }

  public void startCompressor() {
    compressor.enableDigital();
  }

  public void stopCompressor() {
    compressor.disable();
  }

  @Override
  public void updateSmartDashboard() {}

  @Override
  public void zeroSensors() {}

  @Override
  public void setUpTrackables() {}

  @Override
  public void onStart() {
    startCompressor();
  }

  @Override
  public void onStop() {
    stopCompressor();
  }
}

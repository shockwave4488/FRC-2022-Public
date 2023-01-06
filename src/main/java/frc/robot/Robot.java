// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.IRobotContainer;
import frc.lib.PreferencesParser;
import frc.lib.logging.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final Logger logger = new Logger();
  private final PreferencesParser prefs = new PreferencesParser(logger);
  private final RobotSelector robotSelector = new RobotSelector(prefs, logger);
  private IRobotContainer m_robotContainer;
  private NetworkTable table;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = robotSelector.getRobot();
    m_robotContainer.runZeroSensors();
    m_robotContainer.runSetUpTrackables();

    table = NetworkTableInstance.getDefault().getTable("/LiveWindow/Ungrouped/Scheduler");
    logger.addStringTrackable(
        () -> (String.join(", ", table.getEntry("Names").getStringArray(new String[0]))),
        "CommandScheduler",
        4,
        "Running Commands");

    logger.createFiles();

    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5805, "limelight.local", 5805);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    m_robotContainer.runUpdateSmartDashboard();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.runOnStop();

    logger.writeToLogFormatted(this, "Robot Disabled!");
    if (logger.initialized()) {
      logger.flush();
    }
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    logger.initialize();
    logger.writeToLogFormatted(this, "Autonomous Initialized!");
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    m_robotContainer.runZeroSensors();
    m_robotContainer.runOnStart();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    logger.writeToLogFormatted(m_autonomousCommand, "Autonomous Command Called");
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    logger.update();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (!logger.initialized()) {
      logger.initialize();
    }
    logger.writeToLogFormatted(this, "Teleop Initialized!");

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.runOnStart();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    logger.update();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    if (!logger.initialized()) {
      logger.initialize();
    }
    logger.writeToLogFormatted(this, "Test Mode Initialized!");
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}

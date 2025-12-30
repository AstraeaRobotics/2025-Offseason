package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    m_robotContainer.registerNamedCommands();
    
    SmartDashboard.putString("Robot Status", "Initialized with Superstructure");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
    var superstructure = m_robotContainer.getSuperstructure();
    SmartDashboard.putString("Superstructure State", 
        superstructure.getCurrentState().toString());
    SmartDashboard.putBoolean("Has Vision Target", 
        superstructure.hasVisionTarget());
  }

  @Override
  public void disabledInit() {
    SmartDashboard.putString("Robot Mode", "DISABLED");
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    SmartDashboard.putString("Robot Mode", "AUTONOMOUS");
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      SmartDashboard.putString("Auto Status", "Running: " + m_autonomousCommand.getName());
    } else {
      SmartDashboard.putString("Auto Status", "No auto selected");
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    SmartDashboard.putString("Robot Mode", "TELEOP");
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      SmartDashboard.putString("Auto Status", "Cancelled");
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    SmartDashboard.putString("Robot Mode", "TEST");
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
    SmartDashboard.putString("Robot Mode", "SIMULATION");
  }

  @Override
  public void simulationPeriodic() {
  }
}
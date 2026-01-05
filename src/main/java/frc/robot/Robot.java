package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private UsbCamera m_usbCamera;

  public Robot() {
    m_robotContainer = new RobotContainer();

    try {
      m_usbCamera = CameraServer.startAutomaticCapture();
      m_usbCamera.setResolution(320, 240);
      m_usbCamera.setFPS(15);
      System.out.println("USB Cam started");
    } catch (Exception e){
      System.err.println("Cam didn't load: " + e.getMessage());
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
    // Update auto status every loop
    m_robotContainer.updateAutoStatus();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
        m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
        m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}

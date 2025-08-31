// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.videoio.VideoWriter;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private VideoWriter m_videoWriter; //used to save videos to the RIO from the Robot's USB Cam
  private CvSink m_videoSource;
  private int kCamFPS;
  private int[] kCamRes;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  private void setUpCamera(int fps, int[] resolution){
    kCamFPS = fps;
    kCamRes = resolution;

    //MAYBE ONLY USE CVSINK IF USING BOTH HAS PERFORMANCE IMPACT
    UsbCamera camera = CameraServer.startAutomaticCapture(); //used for dashboards
    camera.setResolution(kCamRes[0],kCamRes[1]);

    //Set this to the native FPS to avoid firmware errors 
    camera.setFPS(30);

    m_videoSource = CameraServer.getVideo();

    String filename = "/home/lvuser/" + System.nanoTime() + ".avi";

    //temp video saving code
    int fourcc = VideoWriter.fourcc('M', 'J', 'P', 'G');
    m_videoWriter = new VideoWriter(filename, fourcc, kCamFPS, new Size(kCamRes[0], kCamRes[1]));
  }

  public Robot() {

    setUpCamera(1, new int[] {640, 480}); //
    SmartDashboard.putBoolean("Record Video", false);
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
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
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_videoWriter.release();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    boolean record = SmartDashboard.getBoolean("Record Video", false);

    if(record){
      long currentTimeMicroseconds = (long) (Timer.getTimestamp() * 1000000); 
      int microsecBetweenFrames = 1000000 / kCamFPS; //1000000 is the microsec in 1 sec 

      if(currentTimeMicroseconds - m_videoSource.getLastFrameTime() >= microsecBetweenFrames){
        System.out.println(currentTimeMicroseconds);
        System.out.println(m_videoSource.getLastFrameTime());
        System.out.println();
        Mat frame = new Mat();
        long timestamp = m_videoSource.grabFrame(frame);

        if(timestamp != 0){
          m_videoWriter.write(frame);
        }
      }
    }

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

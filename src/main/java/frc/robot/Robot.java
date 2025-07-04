// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  CommandXboxController controller;
  UsbCamera driveCam;
  UsbCamera clawCam;
  UsbCamera elevatorCam;

  VideoSink server;

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  public DigitalInput elevatorStopTop;
  public DigitalInput elevatorStopBottom;

  boolean topLimitPressed;
  boolean bottomLimitPressed;

  Thread m_visionThread;

  /** Called once at the beginning of the robot program. */
  public Robot() {

    //driveCam = CameraServer.startAutomaticCapture(0);
    //clawCam = CameraServer.startAutomaticCapture(0);
    //elevatorCam = CameraServer.startAutomaticCapture(2);

   //driveCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    //clawCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    //elevatorCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
    elevatorStopTop = new DigitalInput(1);
    elevatorStopBottom = new DigitalInput(0);

    topLimitPressed = false;
    bottomLimitPressed = false;

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
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

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
  
  System.out.println("Bottom Speed: " + m_robotContainer.elevateSpeedBottom);
  System.out.println("Top speed: " + m_robotContainer.elevateSpeedTop);
  /* 
    //Disables upwards elevator movement
    if (elevatorStopTop.get() == true){
      //m_robotContainer.m_elevator.disableUp();
      topLimitPressed = true;
    }
    if (elevatorStopTop.get() == false){
      //m_robotContainer.m_elevator.enableUp();
      topLimitPressed = false;
    }

    //Disables downwards elevator movement
    if (elevatorStopBottom.get() == true){
      //m_robotContainer.m_elevator.disableDown();
      bottomLimitPressed = true;
    }
    if (elevatorStopBottom.get() == false){
      //m_robotContainer.m_elevator.enableDown();
      bottomLimitPressed = false;
    }*/

  
    //Handles elevator controller bindings


    if (elevatorStopTop.get() == false){
      if (m_robotContainer.m_subsystemController.povUp().getAsBoolean() == true){
            m_robotContainer.m_elevator.elevateUp(-1);
      }
      if (m_robotContainer.m_subsystemController.povUp().getAsBoolean() == false){
            m_robotContainer.m_elevator.elevateUp(0);
      }
    if (elevatorStopTop.get() == true){
      m_robotContainer.m_elevator.elevateUp(0);
      }
    }

    if (elevatorStopBottom.get() == false){
      if (m_robotContainer.m_subsystemController.povDown().getAsBoolean() == true){
            m_robotContainer.m_elevator.elevateDown(1);
      }
      if (m_robotContainer.m_subsystemController.povDown().getAsBoolean() == false){
            m_robotContainer.m_elevator.elevateDown(0);
      } 
    if (elevatorStopBottom.get() == true){
      m_robotContainer.m_elevator.elevateDown(0);
      }
    }
  }

/*     if (bottomLimitPressed == false){
      if (m_robotContainer.m_subsystemController.povDown().getAsBoolean() == false && m_robotContainer.m_subsystemController.povUp().getAsBoolean() == false){
        m_robotContainer.m_elevator.elevateDown(0);  
      }
    }
      */

  

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}

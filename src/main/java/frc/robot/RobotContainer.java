
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
/*import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;*/
import edu.wpi.first.wpilibj.XboxController;
/*import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;*/
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoDriveForward;
import frc.robot.commands.LeftAuto;
import frc.robot.commands.RightAuto;
//import frc.robot.commands.MiddleAuto;
//import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Climb;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import java.util.List;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
 
  // The robot's subsystems
  final DriveSubsystem m_robotDrive = new DriveSubsystem();
  //private final Elevator m_elevator = new Elevator();
  //private final Claw m_claw = new Claw();
  private final Climb m_climb = new Climb();

   //DigitalInput elevatorStop;




  public boolean limit_pressed = true;
  private double climbmult = 1;

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  // The subsystem controller
  CommandXboxController m_subsystemController = new CommandXboxController(OIConstants.kSubsystemControllerPort);

  // Controller Buttons
  Trigger xButton = m_driverController.x();

 int defaultCommandSpeed = 1;

  public void disableDrive(){
    defaultCommandSpeed = 0;
  }
  public void enableDrive(){
    defaultCommandSpeed = 1;
  }


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //elevatorStop = new DigitalInput(0);

    // Configure default commands

  //  if (limit_pressed == false){
        m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true,
                    defaultCommandSpeed),
                m_robotDrive));
    }
//  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    /* old event based xboxcontroller
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    */
    //pass in swerve and multiplier



    

    m_driverController.a().whileTrue(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true,
                0.2),
            m_robotDrive)
    );

    m_driverController.y().whileTrue(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                 -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true,
                0.75),
            m_robotDrive)
    );

    //Subsystems
   /*  elevatorStop.get().whileTrue(
        new StartEndCommand(
            () -> m_robotDrive.drive(1,0, 0, false, .16),
            () -> m_robotDrive.drive(1,0, 0, false, .0),
            m_robotDrive)
    );       
*/

    m_driverController.rightTrigger().whileTrue(
        new StartEndCommand(
            () -> m_robotDrive.setX(), 
            () -> m_robotDrive.setX(), 
            m_robotDrive)
    );

  
    m_subsystemController.x().whileTrue(
        new StartEndCommand(
            () -> climbmult = 2, 
            () -> climbmult = 1,
            m_climb)
    );

    m_subsystemController.y().whileTrue(
        new StartEndCommand(
            () -> climbmult = 1.5, 
            () -> climbmult = 1,
            m_climb)
    );

    m_subsystemController.b().whileTrue(
        new StartEndCommand(
            () -> climbmult = .5, 
            () -> climbmult = 1,
            m_climb)
    );

    m_subsystemController.rightBumper().whileTrue(
        new StartEndCommand(
            () -> m_climb.useClimb(.5*climbmult), 
            () -> m_climb.useClimb(0), 
            m_climb)
    );


  m_subsystemController.leftBumper().whileTrue(   
        new StartEndCommand(
            () -> m_climb.useClimb(-.5*climbmult), 
            () -> m_climb.useClimb(0), 
            m_climb)
    );
  }

   /*  m_subsystemController.povRight().whileTrue(
        new StartEndCommand(
            () -> m_claw.useClaw(-1), 
            () -> m_claw.useClaw(0), 
            m_claw)
    );
    m_subsystemController.povLeft().whileTrue(
        new StartEndCommand(
            () -> m_claw.useClaw(.8), 
            () -> m_claw.useClaw(0),/ 
            m_claw)
    );
    m_subsystemController.povUp().whileTrue(
        new StartEndCommand(
            () -> m_elevator.elevate(1), 
            () -> m_elevator.elevate(0), 
            m_claw)
    );
    m_subsystemController.povDown().whileTrue(
        new StartEndCommand(
            () -> m_elevator.elevate(-1), 
            () -> m_elevator.elevate(0), 
            m_claw)
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    /*
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    */
    //return new LeftAuto(m_robotDrive, 15);
    return new RightAuto(m_robotDrive, 20);
    //return new AutoDriveForward(m_robotDrive, 15);
    //return new MiddleAuto(m_robotDrive, 15);
    //return null;
  }
}

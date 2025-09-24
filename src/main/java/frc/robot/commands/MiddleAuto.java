// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Claw;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MiddleAuto extends Command {
  // Creates a new AutoDriveForward. 
  private DriveSubsystem m_DriveSubsystem;
  private double timeToRun;
  private double initTime;
  private Claw claw_motor;

  public MiddleAuto(DriveSubsystem mainDriveSubsystem, Claw m_claw, double time) {
    
    claw_motor = m_claw;
    m_DriveSubsystem = mainDriveSubsystem;
    timeToRun = time;
    initTime = Timer.getTimestamp();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_DriveSubsystem.setX();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override

  //runs for less tha or equal to 3 seconds
  public void execute() {
    while (Timer.getTimestamp() - initTime <= 3){ 
      m_DriveSubsystem.drive(1,0, 0, false, .5);
    }
    while (Timer.getTimestamp() - initTime >3 && Timer.getTimestamp() - initTime <= 5){
      m_DriveSubsystem.drive(0,0, 0, false, 0);
     claw_motor.useClaw(-1); 
    }
    //claw_motor.useClaw(0);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(Timer.getTimestamp() - initTime);
    //
    if (Timer.getTimestamp() - initTime >= timeToRun){
      return true;
    }
    else {
      return false;
    }
  }
}



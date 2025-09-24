// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveSubsystem;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Claw;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LeftAuto extends Command {
  /** Creates a new AutoDriveForward. */
  private DriveSubsystem m_DriveSubsystem; 
  private double timeToRun; // Duration of command
  private double initTime; // Time when the command started

  public LeftAuto(DriveSubsystem mainDriveSubsystem, Claw m_claw, double time) {
    
    timeToRun = time;
    initTime = Timer.getTimestamp();
    m_DriveSubsystem = mainDriveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Tumbleweed
  }
  
  // Called every time the scheduler runs  while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(Timer.getTimestamp() - initTime);

      while (Timer.getTimestamp() - initTime < 2){
        m_DriveSubsystem.drive(1,0, 0, false, .075);
      }
  
      while (Timer.getTimestamp() - initTime >= 2 && (Timer.getTimestamp() - initTime <= 2.5)){
        m_DriveSubsystem.drive(0,0, -.22, false, 1);
      }
  
      while(Timer.getTimestamp() - initTime > 2.5 && (Timer.getTimestamp() - initTime <= 4.5)){
        m_DriveSubsystem.drive(1,0, 0, false, 0.15);
      }
    /*
    while (Timer.getTimestamp() - initTime <= 5){
      System.out.println(Timer.getTimestamp() - initTime);
      m_DriveSubsystem.drive(1,0, 0, false, .1);
    }

    while(Timer.getTimestamp() - initTime > 5 && (Timer.getTimestamp() - initTime <= 10)){
      System.out.println(Timer.getTimestamp() - initTime);
      m_DriveSubsystem.drive(0,0, .1, false, .13);
      }
    while(Timer.getTimestamp() - initTime > 10 && (Timer.getTimestamp() - initTime <= 15)){
      System.out.println(Timer.getTimestamp() - initTime);
      m_DriveSubsystem.drive(1,0, 0, false, .16);

    }
    if (Timer.getTimestamp() - initTime >= 15){

    }
*/
  }  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Prints the amount of time the command took to run
    System.out.println(Timer.getTimestamp() - initTime); 

    if (Timer.getTimestamp() - initTime >= timeToRun){
      return true;
    }
    else {
      return false;
    }
  }

  private double feetToSpeed(double feet) {
    return feet / 24.93;
  }
}
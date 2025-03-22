// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveSubsystem;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LeftAuto extends Command {
  /** Creates a new AutoDriveForward. */
  private Claw motor;
  private DriveSubsystem m_DriveSubsystem; 
  private double timeToRun;
  private double initTime;
  private boolean runClawAuto;


  public LeftAuto(DriveSubsystem mainDriveSubsystem, Claw w, double time) {
    motor = w;
    timeToRun = time;
    initTime = Timer.getTimestamp();
    m_DriveSubsystem = mainDriveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveSubsystem.set45Deg(-1);
  }

  // Called every time the scheduler runs  while the command is scheduled.
  @Override
  public void execute() {
    while (Timer.getTimestamp() - initTime <= 5){
      m_DriveSubsystem.drive(1,0, 0, false, .26868);
    }
    if (Timer.getTimestamp() - initTime == 5){
      m_DriveSubsystem.drive(0,0, 0, false, .0);
    }
    while(Timer.getTimestamp() - initTime > 5 && (Timer.getTimestamp() - initTime <= 10)){
      m_DriveSubsystem.setForward();
      m_DriveSubsystem.drive(1,0, 0, false, .26868);
      }
    while(Timer.getTimestamp() - initTime > 10 && (Timer.getTimestamp() - initTime <= 15)){
      m_DriveSubsystem.drive(0,0, 0, false, .0);
      motor.useClaw(.6);
    }
    if (Timer.getTimestamp() - initTime >= 15){
      motor.useClaw(0);
    }

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
      motor.useClaw(0);
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoDriveForward extends Command {
  /** Creates a new AutoDriveForward. */
  private DriveSubsystem m_DriveSubsystem;
  private double timeToRun;
  private double initTime;

  public AutoDriveForward(DriveSubsystem mainDriveSubsystem, double time) {
    
    m_DriveSubsystem = mainDriveSubsystem;
    timeToRun = time;
    initTime = Timer.getTimestamp();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Timer.getTimestamp() - initTime <= .25){
      m_DriveSubsystem.drive(0,0, .5, false, 1);
    }
    else{
      m_DriveSubsystem.drive(0,0, 0, false, 0);
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
      return true;
    }
    else {
      return false;
    }
  }
}


package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveSubsystem;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Elevator;
public class ElevatorAutoL2 extends Command {
    

/* You should conside using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands 
 
  /** Creates a new AutoDriveForward. */
  private Elevator mElevator; 
  private double timeToRun; // Duration of command
  private double initTime; // Time when the command started

  public ElevatorAutoL2(double time, Elevator m_elevator) {
    initTime = Timer.getTimestamp(); // Stores current time
    mElevator = m_elevator;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Prints the amount of time the command has been running
    System.out.println(Timer.getTimestamp() - initTime);
      // Before 2 seconds
      while (Timer.getTimestamp() - initTime < 2){
        mElevator.elevate(.5d);
      }
      // After 2 seconds and before 2.5 seconds
      /*while (Timer.getTimestamp() - initTime >= 2 && (Timer.getTimestamp() - initTime <= 2.5)){
        mElevator.elevate(.5d);
      }
      // After 2.5 seconds and before 4.5 seconds
      while(Timer.getTimestamp() - initTime > 2.5 && (Timer.getTimestamp() - initTime <= 4.5)){
        mElevator.elevate(.5d);
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
    // Conversion
    return feet / 24.93;
  }
}



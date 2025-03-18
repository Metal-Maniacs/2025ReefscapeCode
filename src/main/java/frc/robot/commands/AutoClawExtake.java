// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoClawExtake extends Command {
  /** Creates a new AutoDriveForward. */
  private Claw motor;
  private double timeToRun;
  private double initTime;
  private boolean runClawAuto;

  public AutoClawExtake(Claw w, double time) {
    motor = w;
    timeToRun = time;
    initTime = Timer.getTimestamp();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   System.out.println("running claw auto");
   while(runClawAuto)
   {
      motor.useClaw(0.3);
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
      runClawAuto = false;
      return true;
    }
    else {
      return false;
    }
  }
}

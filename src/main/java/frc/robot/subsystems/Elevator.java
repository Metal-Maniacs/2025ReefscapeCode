// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class Elevator extends SubsystemBase {

  double ElevatorMultUp = 1;
  double ElevatorMultDown = 1;

  private SparkMax m_elevatorMotor;
  public Elevator() {
    m_elevatorMotor = new SparkMax(DriveConstants.kElevatorCanId, MotorType.kBrushed);
  }

  public void disableUp(){
    ElevatorMultUp = 0;
  }
  public void enableUp(){
    ElevatorMultUp = 1;
  }
  public void disableDown(){
    ElevatorMultDown = 0;
  }
  public void enableDown(){
    ElevatorMultDown = 1;
  }

/*
  public void elevate(double speed){
    m_elevatorMotor.set(speed);
  }
    */

  public void elevateUp(double elevationSpeed) {
    m_elevatorMotor.set(elevationSpeed*ElevatorMultUp);
  }
  
  public void elevateDown(double elevationSpeed) {
    m_elevatorMotor.set(elevationSpeed*ElevatorMultDown);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

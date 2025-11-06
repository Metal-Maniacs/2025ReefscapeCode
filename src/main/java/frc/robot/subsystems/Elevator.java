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
  
  // Changes the speed of elevator
  double ElevatorMultDown = 1;

  private SparkMax m_elevatorMotor;
  
  public Elevator() {
    m_elevatorMotor = new SparkMax(DriveConstants.kElevatorCanId, MotorType.kBrushless);
  }

  public void disableUp() {
    ElevatorMultUp = 0;
  }
  // Changes the speed of elevator
  public void enableUp() {
    //og: 1
    ElevatorMultUp = 1;
  }

  public void disableDown() {
    ElevatorMultDown = 0;
  }

  // Changes the speed of elevator
  public void enableDown() {
    //og: -1
    ElevatorMultDown = -1;
  }

  public void elevate(double speed) {
    m_elevatorMotor.set(speed);
  }
    
/* 
  public void elevateUp(double elevationSpeed) {
    m_elevatorMotor.set(elevationSpeed*ElevatorMultDown);
    System.out.println(elevationSpeed);
  }
  
  public void elevateDown(double elevationSpeed) {
    m_elevatorMotor.set(elevationSpeed*ElevatorMultDown);
  }
  
  */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run (Every 20 )
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

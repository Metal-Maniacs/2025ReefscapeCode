// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class Elevator extends SubsystemBase {

  WPI_TalonSRX m_elevatorMotor;
  public Elevator() {
    m_elevatorMotor = new WPI_TalonSRX(DriveConstants.kElevatorCanId);
  }

  public void elevate(double elevationSpeed) {
    m_elevatorMotor.set(elevationSpeed);
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

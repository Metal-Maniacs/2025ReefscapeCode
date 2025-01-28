package frc.robot;

import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
    private static final Drivetrain m_drivetrain = new Drivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        //configure the bindings for the controller
    }
}
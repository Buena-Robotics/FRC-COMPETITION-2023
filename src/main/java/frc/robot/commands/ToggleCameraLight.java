package frc.robot.commands;

import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ToggleCameraLight extends CommandBase{
    private final DrivetrainSubsystem m_drive;

    public ToggleCameraLight(DrivetrainSubsystem subsystem) {
        m_drive = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        if(Constants.circleLight.get() == Value.kReverse)
            Constants.circleLight.set(Value.kForward);
        else
            Constants.circleLight.set(Value.kReverse);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ToggleCameraLight extends CommandBase{
    private final DrivetrainSubsystem m_drive;

    public ToggleCameraLight(DrivetrainSubsystem subsystem) {
        m_drive = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}

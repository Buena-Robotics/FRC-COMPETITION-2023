package frc.robot.commands;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;


public class SwapLight extends CommandBase {
    public static enum Color {
        Purple,
        Yellow,
    }; 
    private final DrivetrainSubsystem m_drive;
    private final Color color;

    public SwapLight(DrivetrainSubsystem m_drive, Color color){
        this.m_drive = m_drive;
        this.color = color;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        switch (color){
            case Purple:
                Constants.light.set(Value.kForward);
                break;
            case Yellow:
                // Constants.light.set(Value.kOff);
                Constants.light.set(Value.kReverse);
                break;
        }

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

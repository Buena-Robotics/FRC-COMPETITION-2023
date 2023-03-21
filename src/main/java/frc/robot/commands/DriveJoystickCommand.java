package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nerds.utils.GamePositionUtils;
import frc.robot.nerds.utils.Math2;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveJoystickCommand extends CommandBase{

    private final DrivetrainSubsystem m_drive;

    public DriveJoystickCommand(DrivetrainSubsystem subsystem) {
        m_drive = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_drive.arcade_drive();
		SmartDashboard.putNumber("leftEncoder", Math2.round(m_drive.leftEncoder.getPosition(),2));
		SmartDashboard.putNumber("rightEncoder", Math2.round(m_drive.rightEncoder.getPosition(),2));
		// SmartDashboard.putNumber("xPosition", Math2.round(m_drive.gyroscope.getDisplacementX(),2));
		// SmartDashboard.putNumber("yPosition", Math2.round(m_drive.gyroscope.getDisplacementY(), 2));
        // System.out.println(m_drive.gyroscope.getDisplacementX()+"::"+m_drive.gyroscope.getDisplacementY());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}

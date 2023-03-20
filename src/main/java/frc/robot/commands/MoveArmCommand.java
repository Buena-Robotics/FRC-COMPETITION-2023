package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmCommand extends CommandBase{

    private final ArmSubsystem armSubsystem;
    private boolean m_isRotatingInwards;

    public MoveArmCommand(ArmSubsystem subsystem, boolean isRotatingInwards) {
        armSubsystem = subsystem;
        m_isRotatingInwards = isRotatingInwards;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        System.out.println(Constants.armSubsystem.armEncoder.getPosition());
        armSubsystem.rotateArm();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

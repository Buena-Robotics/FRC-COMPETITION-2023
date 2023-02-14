package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToPosition extends CommandBase {
    
    private final ArmSubsystem armSubsystem;
    private int m_dPOV;
    private int destination;

    public MoveArmToPosition(ArmSubsystem subsystem, int pov) {
        armSubsystem = subsystem;
        m_dPOV = pov;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        switch(m_dPOV){
            case(270):
                destination = 0;
                armSubsystem.rotateArmToPosition(destination);
                break;
                //Move to very back / starting pos
            case(180):
                destination = -23;
                armSubsystem.rotateArmToPosition(destination);
                break;
                //Move to shelf height
            case(90):
                destination = -35;
                armSubsystem.rotateArmToPosition(destination);
                break;
                //Move to lower score
            case(0):
                destination = -70;
                armSubsystem.rotateArmToPosition(destination);
                break;
                //Move to high score
            default: break;
            
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return Math.round(armSubsystem.armEncoder.getPosition()) != destination;
    }
}

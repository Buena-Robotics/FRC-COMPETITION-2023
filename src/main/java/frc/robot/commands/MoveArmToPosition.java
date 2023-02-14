package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToPosition extends CommandBase {
    
    private final ArmSubsystem armSubsystem;
    private int m_position;

    public MoveArmToPosition(ArmSubsystem subsystem, int position) {
        armSubsystem = subsystem;
        m_position = position;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}



    @Override
    public void execute() {
        switch(m_position){
            case(270):
                System.out.println("Move to very back / starting pos");
                armSubsystem.moveToPosition(0);
                break;
                //Move to very back / starting pos
            case(180):
                System.out.println("Move to shelf height");
                armSubsystem.moveToPosition(-23);
                break;
                //Move to shelf height
            case(90):
                System.out.println("Move to low score");
                armSubsystem.moveToPosition(-35);
                break;
                //Move to lower score
            case(0):
                System.out.println("Move to high score");
                armSubsystem.moveToPosition(-70);
                break;
                //Move to high score
            
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}

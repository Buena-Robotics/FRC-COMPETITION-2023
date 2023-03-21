package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.nerds.utils.ControllerUtils;
import frc.robot.nerds.utils.ControllerUtils.DPadDirection;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToPosition extends CommandBase {
    
    private final ArmSubsystem armSubsystem;
    private DPadDirection dpad;
    private int destination;

    public MoveArmToPosition(ArmSubsystem subsystem, DPadDirection dpad) {
        armSubsystem = subsystem;
        this.dpad = dpad;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        switch(dpad){
            case RIGHT:
                destination = 0;
                armSubsystem.rotateArmToPosition(destination);
                break;
                //Move to very back / starting pos
            case LEFT:
                destination = (int)SmartDashboard.getNumber("Shelf Height Destination", -19);
                armSubsystem.rotateArmToPosition(destination);
                armSubsystem.openClaw();
                break;
                //Move to shelf height
            case DOWN:
                destination = (int)SmartDashboard.getNumber("Lower Score Destination", -35);
                armSubsystem.rotateArmToPosition(destination);
                break;
                //Move to lower score
            case UP:
                destination = (int)SmartDashboard.getNumber("High Score Height", -70);
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
        return (Math.round(armSubsystem.armEncoder.getPosition()) == destination) || !armSubsystem.armSensorZeroed;
    }
}

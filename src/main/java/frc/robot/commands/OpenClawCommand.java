package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class OpenClawCommand extends CommandBase {

    private ArmSubsystem arm;

    public OpenClawCommand(ArmSubsystem arm) {
        this.arm = arm;
        addRequirements(arm);
    }
    
    @Override
    public void initialize() {
        arm.openClaw();
        super.initialize();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

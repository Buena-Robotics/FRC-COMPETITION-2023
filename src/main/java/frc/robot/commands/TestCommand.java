package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class TestCommand extends CommandBase {
    
    private ArmSubsystem arm;
    private String debugText;
    private boolean done = false;

    public TestCommand(ArmSubsystem arm, String debugText) {
        this.arm = arm;
        this.debugText = debugText;
    }

    @Override
    public void execute() {
        System.out.println(debugText);
        super.execute();
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ToggleGrabCommand extends CommandBase{

    private final ArmSubsystem arm;

    public ToggleGrabCommand(ArmSubsystem subsystem) {
        arm = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        arm.togglePiston();
    }

    @Override
    public void execute() {
        System.out.println("Pnuem");
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}

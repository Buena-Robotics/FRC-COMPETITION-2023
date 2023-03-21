package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nerds.utils.AimUtils;
import frc.robot.subsystems.DrivetrainSubsystem;

public class LookAtPositionCommand extends CommandBase {
    
    private DrivetrainSubsystem drivetrain;
    private double x, z;

    public LookAtPositionCommand(DrivetrainSubsystem drivetrain, double x, double z) {
        this.drivetrain = drivetrain;
        this.x = x;
        this.z = z;
    }

    @Override
    public void execute() {
        double yaw = AimUtils.getRotationFromPosition(x, 0, z)[0];
        // Turn robot til robot yaw matches calculated yaw
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveBalanceCommand extends CommandBase{
    private final DrivetrainSubsystem driveTrain; 
    public DriveBalanceCommand(DrivetrainSubsystem driveTrain){
        this.driveTrain = driveTrain; 
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }
}

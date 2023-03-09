package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnAngleCommand extends CommandBase{

    private final DrivetrainSubsystem drivetrain;
    private final int degree;

    public TurnAngleCommand(DrivetrainSubsystem subsystem, int degree) {
        drivetrain = subsystem;
        this.degree=degree;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}


    int gyroClampedAngle(){
        return Math.round(drivetrain.gyroscope.getYaw()) + 180;
    }
    @Override
    public void execute() {
        final double kP = 0.1;
        final double min = 0.1;
        // if(gyroClampedAngle() + 1 == degree || gyroClampedAngle() - 1 == degree){
        //     break;
        // }
        double normalizedDistance = (kP/degree) * (degree-gyroClampedAngle()); // x

        if(degree-gyroClampedAngle() > 180){
            normalizedDistance *= -1;
        }

        double outputSpeed = normalizedDistance < min && normalizedDistance > 0 ? min : normalizedDistance;
        double newOutputSpeed = (outputSpeed > -min && outputSpeed < 0) ? -min : outputSpeed;
        drivetrain.m_drive.arcadeDrive(newOutputSpeed, 0.0);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        int deg = Math.round(drivetrain.gyroscope.getYaw())+180;
        return deg == degree ||  deg+1 == degree || deg-1 == degree;
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nerds.utils.TimerUtil;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RotateTimeCommand extends CommandBase {

    private DrivetrainSubsystem drivetrain;
    private double driveTimeSec;
    // private static TimerUtil timer = new TimerUtil();
    private boolean backwards = false;
    private double speed = 0.25;
    private Timer timer = new Timer();

    public RotateTimeCommand(DrivetrainSubsystem drivetrain, long driveTimeSec, boolean backwards, double speed) {
        this.drivetrain = drivetrain;
        this.driveTimeSec = driveTimeSec;
        this.backwards = backwards;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        timer.start();
        while(timer.get() >= driveTimeSec){
            //backwards ? speed : -speed
            drivetrain.m_drive.arcadeDrive(speed, 0.25);
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }
    @Override
    public boolean isFinished() {
        return timer.get() >= driveTimeSec; 
    }
}

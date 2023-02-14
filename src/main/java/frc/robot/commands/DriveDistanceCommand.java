package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nerds.utils.GamePositionUtils;
import frc.robot.nerds.utils.Vec2;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveDistanceCommand extends CommandBase{

    private final DrivetrainSubsystem drivetrain;
    private final double speed;

    private Vec2 startPos;
    private Vec2 goalPos;
    private double distance;

    public DriveDistanceCommand(DrivetrainSubsystem subsystem, double speed, double distance) {
        drivetrain = subsystem;
        this.speed = speed;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        startPos = GamePositionUtils.getInstance().getRobotPos();
        // Calculate how far the robot must travel in the x and z direction
        double x = (distance * Math.sin(Math.toRadians(GamePositionUtils.getInstance().getRobotYaw())));
        double z = (distance * Math.cos(Math.toRadians(GamePositionUtils.getInstance().getRobotYaw())));
        goalPos = startPos.add(new Vec2(x, z));
    }

    @Override
    public void execute() {
        drivetrain.driveByDistance(speed);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return startPos.equals(goalPos);
    }
}

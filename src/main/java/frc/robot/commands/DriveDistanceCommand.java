package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nerds.utils.GamePositionUtils;
import frc.robot.nerds.utils.Vec2;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveDistanceCommand extends CommandBase{

    private final DrivetrainSubsystem drivetrain;
    
    private Vec2 startPos;
    private Vec2 goalPos;
    private double distance;
    
    private double kP = 0.2;
    private double kI = 0.1;
    private double kD = 0.05;
    private final PIDController pid = new PIDController(kP, kI, kD);

    public DriveDistanceCommand(DrivetrainSubsystem subsystem, double distance) {
        drivetrain = subsystem;
        this.distance = distance*17.618;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        pid.reset();
        // startPos = GamePositionUtils.getInstance().getRobotPos();
        // // Calculate how far the robot must travel in the x and z direction

        // double x = (distance * Math.sin(Math.toRadians(GamePositionUtils.getInstance().getRobotYaw())));
        // double z = (distance * Math.cos(Math.toRadians(GamePositionUtils.getInstance().getRobotYaw())));
        // goalPos = startPos.add(new Vec2(x, z));
        drivetrain.leftEncoder.setPosition(0);
        drivetrain.rightEncoder.setPosition(0);
    }

    @Override
    public void execute() {
        double pidCalc = pid.calculate(Math.abs(drivetrain.leftEncoder.getPosition()) , Math.abs(distance));
        pidCalc = pidCalc > 0.3 ? 0.3 : pidCalc;
        drivetrain.leftMotors.set(distance > 0 ? -pidCalc : pidCalc);
        drivetrain.rightMotors.set(distance > 0 ? pidCalc : -pidCalc);
        // System.out.println(drivetrain.leftEncoder.getPosition());
        // Vec2 distanceVec = GamePositionUtils.getInstance().getRobotPos().subtract(goalPos);
        // System.out.println(
            // pidCalc 
            // );
        // drivetrain.m_drive.arcadeDrive(0,  pid.calculate(Math.sqrt( 
        //     Math.pow(distanceVec.getX(), 2) + 
        //     Math.pow(distanceVec.getY(), 2)
        //     ), distance)
        // );
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        //70.47247 4Meter
        // 17.618061975 1 Meter
        // return GamePositionUtils.getInstance().getRobotPos().equals(goalPos);
        return Math.abs(drivetrain.leftEncoder.getPosition())  >= Math.abs(distance);
    }
}

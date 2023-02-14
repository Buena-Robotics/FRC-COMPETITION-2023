package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DriveJoystickCommand;
import frc.robot.nerds.utils.ControllerUtils;

public class DrivetrainSubsystem extends SubsystemBase {
    
    public final CANSparkMax leftBackMotor = new CANSparkMax(Constants.LEFTBACKMOTORPORT, MotorType.kBrushless); //MotorController Type
    public final CANSparkMax leftFrontMotor = new CANSparkMax(Constants.LEFTFRONTKMOTORPORT, MotorType.kBrushless); //MotorController Type
    public final CANSparkMax rightBackMotor = new CANSparkMax(Constants.RIGHTBACKMOTORPORT, MotorType.kBrushless);//MotorControllerThang
    public final CANSparkMax rightFrontMotor = new CANSparkMax(Constants.RIGHTFRONTMOTORPORT, MotorType.kBrushless); //MotorController Type

    public final MotorControllerGroup leftMotors = new MotorControllerGroup(leftBackMotor,leftFrontMotor);
    public final MotorControllerGroup rightMotors = new MotorControllerGroup(rightBackMotor,rightFrontMotor);
    public final DifferentialDrive m_drive = new DifferentialDrive(leftMotors, rightMotors);

    public double rampRate = 0.5;
    
    public DrivetrainSubsystem() {
        setDefaultCommand(new DriveJoystickCommand(this));
    }

    @Override public void periodic() {}

    public void set_max_drive_speed(double maximumSpeed){ 
        m_drive.setMaxOutput(maximumSpeed); 
    }

    public boolean leftStickTurn;

    public void arcade_drive(){ 
        leftStickTurn = true; //SmartDashboard.getBoolean("Left stick turn", true);
        rampRate = 0.5; //SmartDashboard.getNumber("Ramp rate", 0.5);
        leftBackMotor.setClosedLoopRampRate(rampRate);
        leftBackMotor.setOpenLoopRampRate(rampRate);
        leftFrontMotor.setClosedLoopRampRate(rampRate);
        leftFrontMotor.setOpenLoopRampRate(rampRate);
        rightBackMotor.setClosedLoopRampRate(rampRate);
        rightBackMotor.setOpenLoopRampRate(rampRate);
        rightFrontMotor.setClosedLoopRampRate(rampRate);
        rightFrontMotor.setOpenLoopRampRate(rampRate);
        m_drive.arcadeDrive(leftStickTurn ? ControllerUtils.controller.getLeftX() : ControllerUtils.controller.getRightX(), ControllerUtils.controller.getLeftY(), true);
    }

    public void drive(double moveSpeed, double turnSpeed) {
        m_drive.arcadeDrive(moveSpeed, turnSpeed);
    }
}
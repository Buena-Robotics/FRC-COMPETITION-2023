package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DriveJoystickCommand;
import frc.robot.nerds.utils.ControllerUtils;
import frc.robot.nerds.utils.GamePositionUtils;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;

public class DrivetrainSubsystem extends SubsystemBase {
    
    public final CANSparkMax leftBackMotor = new CANSparkMax(Constants.LEFTBACKMOTORPORT, MotorType.kBrushless); //MotorController Type
    public final CANSparkMax leftFrontMotor = new CANSparkMax(Constants.LEFTFRONTKMOTORPORT, MotorType.kBrushless); //MotorController Type
    public final CANSparkMax rightBackMotor = new CANSparkMax(Constants.RIGHTBACKMOTORPORT, MotorType.kBrushless);//MotorControllerThang
    public final CANSparkMax rightFrontMotor = new CANSparkMax(Constants.RIGHTFRONTMOTORPORT, MotorType.kBrushless); //MotorController Type

    public final MotorControllerGroup leftMotors = new MotorControllerGroup(leftBackMotor,leftFrontMotor);
    public final MotorControllerGroup rightMotors = new MotorControllerGroup(rightBackMotor,rightFrontMotor);

    public final RelativeEncoder leftEncoder = leftFrontMotor.getEncoder();
    public final RelativeEncoder rightEncoder = rightFrontMotor.getEncoder();

    public final DifferentialDrive m_drive = new DifferentialDrive(leftMotors, rightMotors);

    public AHRS gyroscope = new AHRS(I2C.Port.kOnboard);

    public DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(20));
    public DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getRotation(), GamePositionUtils.getInstance().encoderToMeters(leftBackMotor.getEncoder().getPosition()), GamePositionUtils.getInstance().encoderToMeters(rightBackMotor.getEncoder().getPosition()));

    private double ks = 0.25475, kv = 3.0476, ka = 0.43128;
    private double kp = 0.0028552, ki = 0, kd = 0;

    public SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.25475, 3.0476, 0.43128);

    public PIDController rightController = new PIDController(0.0028552, 0, 0);
    public PIDController leftController = new PIDController(0.0028552, 0, 0);


    public Pose2d pose;


    public double rampRate = 0.5;
    
    public DrivetrainSubsystem() {
        setDefaultCommand(new DriveJoystickCommand(this));
        gyroscope.calibrate();
        gyroscope.resetDisplacement();
        gyroscope.zeroYaw();
        SmartDashboard.putData(getName(), gyroscope);
    }

    @Override public void periodic() {pose = odometry.update(getRotation(), GamePositionUtils.getInstance().encoderToMeters(leftBackMotor.getEncoder().getPosition()), GamePositionUtils.getInstance().encoderToMeters(rightBackMotor.getEncoder().getPosition()));}

    public void set_max_drive_speed(double maximumSpeed){ 
        m_drive.setMaxOutput(maximumSpeed); 
    }

    public boolean leftStickTurn;

    public void arcade_drive(){ 
        leftStickTurn = true; //SmartDashboard.getBoolean("Left stick turn", true);
        // rampRate = 0.5; //SmartDashboard.getNumber("Ramp rate", 0.5);
        m_drive.arcadeDrive(leftStickTurn ? ControllerUtils.controller.getLeftX() : ControllerUtils.controller.getRightX(), -ControllerUtils.controller.getLeftY(), true);
    }

    public void driveByDistance(double moveSpeed) {
        m_drive.arcadeDrive(moveSpeed, 0);
    }
    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(-gyroscope.getAngle());
    }

    public Pose2d getPose() {
        return pose;
    }

    public void setOutput(double left, double right) {
        leftMotors.setVoltage(left / 12);
        rightMotors.setVoltage(right / 12);
    }

    public DifferentialDriveWheelSpeeds getSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            leftBackMotor.getEncoder().getVelocity() / Constants.GEAR_RATIO * 2 * Math.PI * Units.inchesToMeters(3.0) / 60, 
            rightBackMotor.getEncoder().getVelocity() / Constants.GEAR_RATIO * 2 * Math.PI * Units.inchesToMeters(3.0) / 60
            );
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
              // Reset odometry for the first path you run during auto
              if(isFirstPath){
                odometry.resetPosition(getRotation(), kd, ka, pose);
                    odometry.resetPosition(getRotation(), GamePositionUtils.getInstance().encoderToMeters(leftBackMotor.getEncoder().getPosition()), GamePositionUtils.getInstance().encoderToMeters(rightBackMotor.getEncoder().getPosition()), traj.getInitialPose());
              }
            }),
            new PPRamseteCommand(
                traj, 
                this::getPose, // Pose supplier
                new RamseteController(),
                new SimpleMotorFeedforward(ks, kv, ka),
                this.kinematics, // DifferentialDriveKinematics
                this::getSpeeds, // DifferentialDriveWheelSpeeds supplier
                new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
                this::setOutput, // Voltage biconsumer
                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                this // Requires this drive subsystem
            )
        );
    }
}

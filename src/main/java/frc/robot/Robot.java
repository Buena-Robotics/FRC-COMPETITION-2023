// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.Arrays;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.nerds.utils.CameraUtils;
import frc.robot.nerds.utils.ControllerUtils;
import frc.robot.nerds.utils.GamePositionUtils;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  private Command m_autonomousCommand;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ControllerUtils.setBindings();
    // SmartDashboard.putBoolean("Auto disabled", Autonomous.autonomousDisabled);
		// SmartDashboard.putBoolean("Shoot disabled", Autonomous.shootDisabled);
		// SmartDashboard.putBoolean("Drive disabled", Autonomous.driveDisabled);

    // SmartDashboard.putNumber("Auto delay", Autonomous.waitTimeMS);
		// SmartDashboard.putNumber("Auto speed", Autonomous.speed);
		// SmartDashboard.putBoolean("Left stick turn", true);
		// SmartDashboard.putNumber("Auto time", Autonomous.driveTimeMS);

		SmartDashboard.putNumber("Ramp Rate", 0.5);

		SmartDashboard.putNumber("Lower Score Destination", -35);
		SmartDashboard.putNumber("Shelf Height Destination", -19);
		SmartDashboard.putNumber("High Score Height", -70);

		SmartDashboard.putNumber("WaitDelay", 0.3);
		SmartDashboard.putNumber("DriveDistance", -0.5);
		// SmartDashboard.putNumber("RotateTime", 0);
		// SmartDashboard.putNumber("DriveBackTime2", 0);

    // SmartDashboard.putNumber("Arm kP", 0.5);
    // SmartDashboard.putNumber("Arm kI", 0.5);
    // SmartDashboard.putNumber("Arm kD", 0.5);

    // SmartDashboard.putNumber("Turn kP", 0.5);
    // SmartDashboard.putNumber("Turn kI", 0.5);
    // SmartDashboard.putNumber("Turn kD", 0.5);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
    // if (CameraUtils.getCamera().getLatestResult().hasTargets()) {
    //   PhotonTrackedTarget result = CameraUtils.getCamera().getLatestResult().getBestTarget();
    //   Transform3d cameraToRobot = result.getBestCameraToTarget();
    //   AprilTagFieldLayout layout;
    //   try {
    //     layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    //     Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(result.getBestCameraToTarget(), layout.getTagPose(result.getFiducialId()).get(), cameraToRobot);
    //     System.out.println(robotPose.toString());
    //     System.out.println("it kinda worked");
    //   } catch(IOException e) {
    //     System.out.println("Something Went wrong here");
    //       // TODO decide what you want to do if the layout fails to load
    //   }
      // AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
    //   double range =
    //           PhotonUtils.calculateDistanceToTargetMeters(
    //             0.5842,
    //               0.4445,
    //               Math.toRadians(Constants.driveTrain.gyroscope.getYaw()+180),
    //               Units.degreesToRadians(result.getBestTarget().getPitch()));
    //   System.out.println(range);
    //   // System.out.println(result.getBestTarget().getYaw()); Angle , Right is positive
    //   // System.out.println(result.getBestTarget().getBestCameraToTarget().); 
    // }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    GamePositionUtils.getInstance().unsubscribe();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    if(!Constants.armSubsystem.armSensorZeroed && !Constants.armSubsystem.magnetSensor.get()){Constants.armSubsystem.armEncoder.setPosition(0); Constants.armSubsystem.armSensorZeroed = true;}

    m_autonomousCommand = GamePositionUtils.getInstance().getCommunityPos().getAutoCommand();

    //TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(2));
    //config.setKinematics(Constants.driveTrain.kinematics);
    // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      // new Pose2d(), Arrays.asList(), new Pose2d(1.0, 0.0, new Rotation2d()),
      // config
      // );

      //PathPlannerTrajectory traj = PathPlanner.loadPath("Test", new PathConstraints(4, 3));

      //m_autonomousCommand = Constants.driveTrain.followTrajectoryCommand(traj, true);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
      m_autonomousCommand.schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    final double rampRate = SmartDashboard.getNumber("Ramp Rate", Constants.D_RampRate);
    Constants.driveTrain.leftBackMotor.setClosedLoopRampRate(rampRate);
    Constants.driveTrain.leftBackMotor.setOpenLoopRampRate(rampRate);
    Constants.driveTrain.leftFrontMotor.setClosedLoopRampRate(rampRate);
    Constants.driveTrain.leftFrontMotor.setOpenLoopRampRate(rampRate);
    Constants.driveTrain.rightBackMotor.setClosedLoopRampRate(rampRate);
    Constants.driveTrain.rightBackMotor.setOpenLoopRampRate(rampRate);
    Constants.driveTrain.rightFrontMotor.setClosedLoopRampRate(rampRate);
    Constants.driveTrain.rightFrontMotor.setOpenLoopRampRate(rampRate);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

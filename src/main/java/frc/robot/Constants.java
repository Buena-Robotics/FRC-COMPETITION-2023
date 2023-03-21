// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  //Buttons
  public static final XboxController.Button A = XboxController.Button.kA
      ,B = XboxController.Button.kB
      ,X = XboxController.Button.kX
      ,Y = XboxController.Button.kY
      ,LB = XboxController.Button.kLeftBumper
      ,RB = XboxController.Button.kRightBumper
      ,LS = XboxController.Button.kLeftStick
      ,RS = XboxController.Button.kRightStick
      ,MENU = XboxController.Button.kStart
      ,NAVIGATION = XboxController.Button.kBack;
//Axis
  public static final XboxController.Axis LS_Y = XboxController.Axis.kLeftY
      ,LS_X = XboxController.Axis.kLeftX
      ,RS_Y = XboxController.Axis.kRightY
      ,RS_X = XboxController.Axis.kRightX
      ,LT = XboxController.Axis.kLeftTrigger
      ,RT = XboxController.Axis.kRightTrigger;

  //Ports
  public static final int XBOXCONTROLLERPORT = 0;
    
  //Can
  public static final int LEFTBACKMOTORPORT = 1
      ,LEFTFRONTKMOTORPORT = 3
      ,RIGHTBACKMOTORPORT = 2
      ,RIGHTFRONTMOTORPORT = 4;

  // Subsystems
  public static DrivetrainSubsystem driveTrain = new DrivetrainSubsystem();
  public static ArmSubsystem armSubsystem = new ArmSubsystem();

  public final static Relay light = new Relay(0);

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static double D_RampRate = 0.5;

  public static final int WHEEL_DIAMETER_INCH = 6;
  public static final double ENCODER_TO_METER = 70.47247 / 4;
  public static final double GEAR_RATIO = 8.45;

  public static CommandBase toggleClawCommand = Commands.runOnce(Constants.armSubsystem::togglePiston, Constants.armSubsystem);
  public static CommandBase openClawCommand = Commands.runOnce(Constants.armSubsystem::openClaw, Constants.armSubsystem);
}

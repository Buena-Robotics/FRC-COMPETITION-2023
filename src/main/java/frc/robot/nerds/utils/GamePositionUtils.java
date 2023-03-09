package frc.robot.nerds.utils;

import org.photonvision.targeting.PhotonPipelineResult;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.MoveArmToPosition;
import frc.robot.commands.OpenClawCommand;
import frc.robot.commands.TurnAngleCommand;

// Contains positions for camera cone and TODO: april tag spots
public class GamePositionUtils {
    
    private static GamePositionUtils instance;

    private NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
    // The subscribers for the cone x and cone y values
    private DoubleSubscriber cxSub, cySub;

    private AHRS gyroscope;

    private GamePositionUtils() {
        cxSub = ntInst.getDoubleTopic("cx").subscribe(0.0);
        cySub = ntInst.getDoubleTopic("cy").subscribe(0.0);
    }

    public static GamePositionUtils getInstance() {
        if (instance == null) instance = new GamePositionUtils();
        return instance;
    }

    public double getConeX() {
        return cxSub.getAsDouble();
    }

    public double getConeY() {
        return cySub.getAsDouble();
    }

    public double[] getConePos() {
        return new double[] {cxSub.getAsDouble(), cySub.getAsDouble()};
    }

    // Unsubscribes from the cx and cy topics
    public void unsubscribe() {
        cxSub.close();
        cySub.close();
    }

    public CommunityPosition getCommunityPos() {
        PhotonPipelineResult result = CameraUtils.getCamera().getLatestResult();
        
        if (result.hasTargets()) {
            for (CommunityPosition pos : CommunityPosition.values()) {
                if (pos.getId() == result.getBestTarget().getFiducialId()) return pos;
            }
        }

        return CommunityPosition.NONE;
    }

/*
5: (7.25, 6.76)
6: (7.25, 1.07)
7: (7.25, 2.75)
8: (7.25, 4.42)

1: (7.25,-1.07)
2: (7.25,-2.75)
3: (7.25,-4.42)
4: (7.96,-6.75)
 */

    public enum CommunityPosition {
        NONE(-1, null),
        RED_RIGHT(1, new SequentialCommandGroup()),
        RED_CENTER(2, new SequentialCommandGroup()),
        RED_LEFT(3, new SequentialCommandGroup()),
        RED_LOADING(4, new SequentialCommandGroup()),
        BLUE_RIGHT(8, new SequentialCommandGroup()),
        BLUE_CENTER(7, new SequentialCommandGroup()),
        BLUE_LEFT(6, new SequentialCommandGroup()),
        BLUE_LOADING(5, new SequentialCommandGroup());

        private int id;
        private Command autoCommand;

        CommunityPosition(int id, Command autoCommand) {
            this.id = id;
            this.autoCommand = new SequentialCommandGroup(
                new TurnAngleCommand(Constants.driveTrain, 30)
            );
            // this.autoCommand = new SequentialCommandGroup(
            //     new MoveArmToPosition(Constants.armSubsystem, ControllerUtils.DPadDirection.UP), 
            //     new OpenClawCommand(Constants.armSubsystem),
            //     new MoveArmToPosition(Constants.armSubsystem, ControllerUtils.DPadDirection.RIGHT)
            //     // new ParallelCommandGroup(
            //         // new RotateTimeCommand(Constants.driveTrain, 2, false, 0.6),
                
            //         // )
            //     // new DriveTimeCommand(Constants.driveTrain, 2000, true, 0.6)
            // );
        }

        public int getId() {
            return id;
        }

        public Command getAutoCommand() {
            return autoCommand;
        }
    }

    public AHRS getGryoscope() {
        if (gyroscope == null) {
            gyroscope = new AHRS(I2C.Port.kOnboard);
        }

        return gyroscope;
    }

    public double getRobotX() {
        return gyroscope.getDisplacementX();
    }

    public double getRobotY() {
        return gyroscope.getDisplacementY();
    }

    public double getRobotZ() {
        return gyroscope.getDisplacementZ();
    }

    public Vec2 getRobotPos() {
        return new Vec2(gyroscope.getDisplacementX(), gyroscope.getDisplacementZ());
    }

    public double getRobotYaw() {
        return gyroscope.getYaw();
    }

    public double getRobotPitch() {
        return gyroscope.getPitch();
    }
}

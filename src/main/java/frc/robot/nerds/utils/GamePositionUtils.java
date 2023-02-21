package frc.robot.nerds.utils;

import org.photonvision.targeting.PhotonPipelineResult;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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
            this.autoCommand = autoCommand;
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

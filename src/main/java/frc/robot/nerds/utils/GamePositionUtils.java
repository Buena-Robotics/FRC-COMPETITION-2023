package frc.robot.nerds.utils;

import org.photonvision.targeting.PhotonPipelineResult;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C;

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
        NONE(-1),
        RED_RIGHT(1),
        RED_CENTER(2),
        RED_LEFT(3),
        RED_LOADING(4),
        BLUE_RIGHT(8),
        BLUE_CENTER(7),
        BLUE_LEFT(6),
        BLUE_LOADING(5);

        private int id;

        CommunityPosition(int id) {
            this.id = id;
        }

        public int getId() {
            return id;
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

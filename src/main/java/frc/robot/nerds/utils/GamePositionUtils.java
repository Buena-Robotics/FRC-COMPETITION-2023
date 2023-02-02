package frc.robot.nerds.utils;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.nerds.utils.GamePositionUtils.CommunityPosition;

// Contains positions for camera cone and TODO: april tag spots
public class GamePositionUtils {
    
    private static GamePositionUtils instance;

    private NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
    // The subscribers for the cone x and cone y values
    private DoubleSubscriber cxSub, cySub;


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

    public CommunityPosition getCurrentPos() {
        PhotonPipelineResult result = CameraUtils.getCamera().getLatestResult();
        
        if (result.hasTargets()) {
            for (CommunityPosition pos : CommunityPosition.values()) {
                if (pos.getId() == result.getBestTarget().getFiducialId()) return pos;
            }
        }

        return CommunityPosition.NONE;
    }

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
}

package frc.robot.nerds.utils;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;

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
}

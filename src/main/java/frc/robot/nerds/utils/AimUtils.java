package frc.robot.nerds.utils;

public class AimUtils {
 
    private static double camWidth = 1280, camHeight = 720;
    private static double[] center = new double[] {camWidth / 2, camHeight / 2};

    /**
     * Aims the robot at the detected cone's position
     */
    public static void aimAtCone() {
        if (GamePositionUtils.getInstance().getConeX() < center[0]) {
            // Turn robot left
        } else if (GamePositionUtils.getInstance().getConeX() > center[0]) {
            // Turn robot right
        }
    }
}

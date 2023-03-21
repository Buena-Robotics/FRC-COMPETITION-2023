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

    public static float[] getRotationFromPosition(double x, double y, double z) {
	    double xDiff = x - GamePositionUtils.getInstance().getRobotX();
        double yDiff = y - GamePositionUtils.getInstance().getRobotY();
	    double zDiff = z - GamePositionUtils.getInstance().getRobotZ();
	
	    double dist = Math.sqrt((float) (xDiff * xDiff + zDiff * zDiff));
	    float yaw = (float) (Math.atan2(zDiff, xDiff) * 180.0D / Math.PI) - 90.0F;
	    float pitch = (float) -(Math.atan2(yDiff, dist) * 180.0D / Math.PI);
	    return new float[]{yaw, pitch};
	}
}

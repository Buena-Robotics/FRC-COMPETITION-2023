package frc.robot.nerds.drive.autonomous;

import frc.robot.nerds.utils.GamePositionUtils;

public class DrivePath {
    
    public DrivePath(int path) {
        
    }

    public void executeDrive() {

    }

    private void driveToPoint(double x, double z) {
        if (GamePositionUtils.getInstance().getRobotX() > x) {
            // Turn robot around
        }
    }
}

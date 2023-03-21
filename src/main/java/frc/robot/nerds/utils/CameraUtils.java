package frc.robot.nerds.utils;

import org.photonvision.PhotonCamera;

public class CameraUtils {
    
    private static PhotonCamera camera = new PhotonCamera("mscam");

    public static PhotonCamera getCamera() {
        return camera;
    }
}

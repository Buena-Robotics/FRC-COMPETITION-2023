package frc.robot.nerds;

public class Autonomous {
    /*
        **All Units cm precision
        PreGame:
            Upload a general Path file which contains the general starting position, rotation; points to travel to aswell as 
                commands to run at given points
        Position System: 
            Using Navx;
            Using OpenCV;
            start game by pointing robot towards any QR code
            getPosition:
                using the QR codes on the field and openCV to detect which QR code && distance from QR Code
            getRotation:
                same as position but with rotation
            Generate new path that follow all same parameters as the one given before the game
            Robot follows commands to do things
        Default System:
            Drop off a cargo
    */
}
/*
Python getPositionSudoCode:
    QRCodeImageMap.load()
    on Autonomous Start:
        frame = take picture using camera
        QRCodeIndex = opencv.findImage([code1,code2, ...]).map( QRCodeImageMap ) // An Index representing the specific QR code that the robot is at
        BoundingBox = numpy.array(QRCode.topLeft, QRCode.topRight, QRCode.bottomRight, QRCode.bottomLeft)

        rotationOffest = boundingBox[0], boundingBox[1]
 */
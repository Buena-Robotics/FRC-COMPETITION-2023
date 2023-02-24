package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.nerds.utils.ControllerUtils;

public class ArmSubsystem extends SubsystemBase {
    
    private DoubleSolenoid solenoidValves = new DoubleSolenoid(PneumaticsModuleType.REVPH,0, 1);
    private Compressor compressor = new Compressor(11, PneumaticsModuleType.REVPH);

    private final CANSparkMax armMotor = new CANSparkMax(5, MotorType.kBrushless);
    public final RelativeEncoder armEncoder = armMotor.getEncoder();

    private boolean armSensorZeroed = false;

    private DigitalInput magnetSensor = new DigitalInput(0);

    public ArmSubsystem() {
        compressor.enableDigital();
    }

    public void rotateArm(boolean isRotatingInwards){
        System.out.println("Rotate");
        if(!armSensorZeroed && !magnetSensor.get()){armEncoder.setPosition(0); armSensorZeroed = true;}

        if(isRotatingInwards && magnetSensor.get() == true){
            armMotor.set(0.1);
        } else{
            armMotor.set(-0.1);
        }
    }

    public void rotateArmToPosition(int destination){//{ -70 < destination < 0}
        if(armSensorZeroed){
            final double endPosition = -70;
            final double kP = 0.6;
            
            double normalizedDistance = kP * ((armEncoder.getPosition() + Math.abs(destination))/endPosition); //x

            double outputSpeed = normalizedDistance < 0.20 && normalizedDistance > 0 ? 0.20 : normalizedDistance;
            double newOutputSpeed = (outputSpeed > -0.20 && outputSpeed < 0) ? -0.20 : outputSpeed;
            armMotor.set(newOutputSpeed);
        }
    }

    public void togglePiston() {
        solenoidValves.toggle();
    }
}

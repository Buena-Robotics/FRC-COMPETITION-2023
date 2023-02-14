package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    
    private DoubleSolenoid solenoidValves = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,0, 1);
    private Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    private final CANSparkMax intakeMotor = new CANSparkMax(5, MotorType.kBrushless);
    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

    public ArmSubsystem() {
        compressor.enableDigital();
    }

    public void moveToPosition(int goToPosition){//{ -70 < x < 0}
        final double endPosition = -70;
        final double kP = 0.6;
        
        while(Math.round(intakeEncoder.getPosition()) != goToPosition){
            
            double normalizedDistance = kP * ((intakeEncoder.getPosition() + Math.abs(goToPosition))/endPosition); //x

            // double outputSpeed = Math.log(-normalizedDistance+1)/log1000Expr;
            double outputSpeed = normalizedDistance < 0.20 && normalizedDistance > 0 ? 0.20 : normalizedDistance;
            double newOutputSpeed = (outputSpeed > -0.20 && outputSpeed < 0) ? -0.20 : outputSpeed;
            intakeMotor.set(newOutputSpeed);
        }
    }

    public void togglePiston() {
        solenoidValves.toggle();
    }
}

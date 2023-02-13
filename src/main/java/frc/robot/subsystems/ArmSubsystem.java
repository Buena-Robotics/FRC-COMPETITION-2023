package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    
    private DoubleSolenoid solenoidValves = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,0, 1);
    private Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    public ArmSubsystem() {
        compressor.enableDigital();
    }

    public void togglePiston() {
        solenoidValves.toggle();
    }
}

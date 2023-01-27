package frc.robot.nerds.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;

public class WheelDrive {
    
    private final double MAX_VOLTS = 4.95;

    private CANSparkMax angleMotor;
    private CANSparkMax speedMotor;
    private PIDController pidController;

    public WheelDrive(int angleMotor, int speedMotor, int encoder) {
        this.angleMotor = new CANSparkMax(angleMotor, MotorType.kBrushless);
        this.speedMotor = new CANSparkMax(speedMotor, MotorType.kBrushless);
        pidController = new PIDController(1, 0, 0);
    }

    public void drive(double speed, double angle) {
        speedMotor.set(speed);
    
        double setpoint = angle * (MAX_VOLTS * 0.5) + (MAX_VOLTS * 0.5); // Optimization offset can be calculated here.
        if (setpoint < 0) {
            setpoint = MAX_VOLTS + setpoint;
        }
        if (setpoint > MAX_VOLTS) {
            setpoint = setpoint - MAX_VOLTS;
        }
    
        pidController.setSetpoint(setpoint);
    }
}
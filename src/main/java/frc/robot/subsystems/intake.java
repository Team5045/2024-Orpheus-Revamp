package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intake extends SubsystemBase{
    private TalonFX intakeL; 

    public intake(TalonFX s){
        this.intakeL = s;
    }
    public void setvolt(double volts){
        intakeL.setVoltage(volts);
    }
}

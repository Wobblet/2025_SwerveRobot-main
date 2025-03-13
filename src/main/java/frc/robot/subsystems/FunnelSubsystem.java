package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FunnelSubsystem extends SubsystemBase {
    private final SparkMax funnelMotor = new SparkMax(Constants.DriveConstants.funnelId, MotorType.kBrushless);

    SparkAbsoluteEncoder funnelEncoder = funnelMotor.getAbsoluteEncoder();

    public void setFunnel(double speed){
        funnelMotor.set(speed);
    }

    public double getFunnelEncoderValue(){
        return funnelEncoder.getPosition();
    }
    
    @Override
    public void periodic(){

    }

    @Override
    public void simulationPeriodic(){

    }
}

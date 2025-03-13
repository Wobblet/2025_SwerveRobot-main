package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax climberMotor = new SparkMax(Constants.DriveConstants.climberId, MotorType.kBrushed);

    SparkAbsoluteEncoder climberEncoder = climberMotor.getAbsoluteEncoder();

    public void setClimber(double speed){
        climberMotor.set(speed);
    }

    public double getClimberEncoderValue(){
        return climberEncoder.getPosition();
    }
    
    @Override
    public void periodic(){

    }

    @Override
    public void simulationPeriodic(){

    }
}

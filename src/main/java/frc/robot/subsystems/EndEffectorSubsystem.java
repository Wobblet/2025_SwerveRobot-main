package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffectorSubsystem extends SubsystemBase {
    private final SparkMax endEffectorMotor1 = new SparkMax(Constants.DriveConstants.EndEffectorId1, MotorType.kBrushless);
    private final SparkMax endEffectorMotor2 = new SparkMax(Constants.DriveConstants.EndEffectorId2, MotorType.kBrushless);

    public void setEndEffector(double speed){
        endEffectorMotor1.set(speed);
        endEffectorMotor2.set(-speed);
    }
    
    @Override
    public void periodic(){

    }

    @Override
    public void simulationPeriodic(){

    }
}

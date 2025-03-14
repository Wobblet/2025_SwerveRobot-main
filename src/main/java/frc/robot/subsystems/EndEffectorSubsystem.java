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

    public void triggerEndEffector(double Lspeed, double Rspeed){
        if (Lspeed == .6){
            endEffectorMotor1.set(Lspeed);
            endEffectorMotor2.set(-Lspeed);
        }
        else if(Rspeed == .6){
            endEffectorMotor1.set(Rspeed);
            endEffectorMotor2.set(Rspeed);
        }

    }
    
    @Override
    public void periodic(){

    }

    @Override
    public void simulationPeriodic(){

    }
}

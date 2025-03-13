package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    // private final DigitalInput topLimitSwitch = new DigitalInput(0);
    // private final DigitalInput bottomLimitSwitch = new DigitalInput(1);
    private final static SparkMax elevatorMotor1 = new SparkMax(Constants.DriveConstants.elevatorId, MotorType.kBrushless);
    public final SparkAbsoluteEncoder elevatorEncoder = elevatorMotor1.getAbsoluteEncoder(); /*Change to Relative Encoder*/

    public void setElevator(double speed){
        elevatorMotor1.set(speed);
    }

    public double getElevatorEncoderValue(){
        return elevatorEncoder.getPosition();
    }

    // public double resetElevatorEncoderValue(){
    //     return elevatorEncoder.resetEncoders();
    // }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Encoder Value", elevatorEncoder.getPosition());
    }

    @Override
    public void simulationPeriodic(){

    }
}

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class ElevatorSubsystem extends SubsystemBase {
        // private final DigitalInput topLimitSwitch = new DigitalInput(0);
        // private final DigitalInput bottomLimitSwitch = new DigitalInput(1);
        private SparkMax elevatorMotor1;
            //public final SparkAbsoluteEncoder elevatorEncoder = elevatorMotor1.getAbsoluteEncoder(); /*Change to Relative Encoder*/
        private RelativeEncoder elevatorRelative;
        private SparkClosedLoopController m_ElevatorPID;
        private SparkMaxConfig elevatorMotorConfig;
    
        public enum ElevatorPositions {
            GL(0),
            FL(21 + DriveConstants.h),
            L1(59 + DriveConstants.h),
            L2(99 + DriveConstants.h),
            L3(159 + DriveConstants.h),
            L4(249 + DriveConstants.h);
    
            private final double value;
            
    
            ElevatorPositions(double value){
                this.value = value;
            }

    
            public double getValue() {
                return value;
            }
    
        }


        public void setH(int value){
            if (value == 0){
                DriveConstants.h = 0;
            } else if (value == 1){
                DriveConstants.h = 1;
            } else if (value == 2){
                DriveConstants.h = -1;
            }
        }
            
        public ElevatorSubsystem(){
            elevatorMotor1 = new SparkMax(Constants.DriveConstants.elevatorId, MotorType.kBrushless);
            elevatorRelative = elevatorMotor1.getEncoder();
            m_ElevatorPID = elevatorMotor1.getClosedLoopController();
            elevatorMotorConfig = new SparkMaxConfig();
    
            // Change P(Elevator Speed) till it doesn't skip
            // p = Speed 
            // i = Correction
            // d = Dampening
            elevatorMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(.2)
                .i(0)
                .d(0.02)
                .outputRange(-1, 1);
            elevatorMotorConfig
                .inverted(true)
                .smartCurrentLimit(40)
                .idleMode(IdleMode.kBrake);
            elevatorMotor1.configure(elevatorMotorConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            elevatorRelative.setPosition(0);
        }
        
        public void setElevator(double speed){
            elevatorMotor1.set(speed);
        }
    
        public void stopElevator(){
            elevatorMotor1.stopMotor();
        }
        
        public double getElevatorEncoderValue(){
            return elevatorRelative.getPosition();
        }
    
        public double getElevatorVelocity(){
            return elevatorRelative.getVelocity();
        }
    
        public void setPosition(ElevatorPositions position){
            m_ElevatorPID.setReference(position.getValue(), ControlType.kPosition);
        }
        
            // public double resetElevatorEncoderValue(){
            //     return elevatorEncoder.resetEncoders();
            // }
        
            @Override
            public void periodic(){
                SmartDashboard.putNumber("Elevator Encoder Value", elevatorRelative.getPosition());
            }
        
            @Override
            public void simulationPeriodic(){
        
            }
    
            public void resetElevatorPose(){
                if (DriveConstants.kStartPose + 0.04 > elevatorRelative.getPosition()){
                    elevatorMotor1.set(DriveConstants.elevatorMotorSpeedSlow);
                } else if (DriveConstants.kStartPose - 0.04 < elevatorRelative.getPosition()){
                    elevatorMotor1.set(-DriveConstants.elevatorMotorSpeedSlow);
                } else{
                    elevatorMotor1.stopMotor();
                }
            }
    
    }

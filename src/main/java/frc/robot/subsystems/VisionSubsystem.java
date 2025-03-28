// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  public final NetworkTableEntry ty;
  public final NetworkTableEntry tx;
  public final NetworkTableEntry tl;
  public final NetworkTableEntry tv;
  public double distanceToGoalInches;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    ty = table.getEntry("ty");
    tx = table.getEntry("tx");
    tl = table.getEntry("ta");
    tv = table.getEntry("tv");
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
  }

  public double getX(){
    return tx.getDouble(0.0);
  }

  public void setPipeline(int pipeline){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
  }

  public double getPipeline(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").getDouble(0);
  }

  public double getY(){
    return ty.getDouble(0.0);
  }

  public double getTL(){
    return tl.getDouble(0.0);
  }

  public double getTV(){
    return tv.getDouble(0.0);
  }

  public double getDistance(){
    return distanceToGoalInches;
  }
  @Override
  public void periodic() {
    // Find distance to target
    double targetOffsetAngle_Horizontal = tx.getDouble(0.0);

    double limelightMountAngleDegrees = 0;

    double limelightLensHeightInches = 28.5;

    // Calculate horizontal distance using tan function (since camera and target are at the same height)
    double angleToTargetDegrees = limelightMountAngleDegrees + targetOffsetAngle_Horizontal;
    double angleToTargetRadians = angleToTargetDegrees * (Math.PI / 180.0);

    // Calculate the distance (horizontal distance since camera and target are at the same height)
    distanceToGoalInches = limelightLensHeightInches / Math.tan(angleToTargetRadians);

    // Show the calculated distance on the SmartDashboard
    SmartDashboard.putNumber("Distance from limelight", distanceToGoalInches);
  }
}

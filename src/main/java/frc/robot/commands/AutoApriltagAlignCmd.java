// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoApriltagAlignCmd extends Command {
  private final VisionSubsystem visionSubsystem;
  private final DriveSubsystem driveSubsystem;

  /** Creates a new AutoApriltagAlignCmd. */
  public AutoApriltagAlignCmd(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     // Check if a target is visible
        if (visionSubsystem.getTV() > 0) {  // If a valid target is found (tv > 0)
            // Get the horizontal offset from the crosshair to the target
            double tx = visionSubsystem.getX();  // Horizontal offset (left/right)
            // Get the vertical offset from the crosshair to the target
            double ty = visionSubsystem.getY();  // Vertical offset (up/down)
            // Get the distance from the Limelight to the target
            double distance = visionSubsystem.getDistance();

            // If the robot is off-center horizontally (tx != 0), turn to face the target
            if (Math.abs(tx) > 1.0) {  // Threshold to start rotating (you can adjust the tolerance)
                double turnSpeed = tx * 0.05;  // Scale the turn speed based on tx (adjust factor as needed)
                driveSubsystem.drive(0, 0, turnSpeed, false, 2);  // Rotate only (no forward/backward movement)
            }
            // If the robot is not facing the target (tx is not zero), rotate towards the target
            else {
                // If the robot is aligned with the target (tx = 0), move toward the target
                if (Math.abs(ty) > 1.0) {  // Threshold to start moving (adjust as needed)
                    double moveSpeed = 0.5;  // Constant speed toward the target (adjust as needed)
                    driveSubsystem.drive(moveSpeed, 0, 0, false, 2);  // Move forward
                } else {
                    driveSubsystem.drive(0, 0, 0, false, 2);  // Stop moving once aligned
                }
            }

            // Optionally: Display status on the SmartDashboard for debugging
            SmartDashboard.putNumber("tx (horizontal offset)", tx);
            SmartDashboard.putNumber("ty (vertical offset)", ty);
            SmartDashboard.putNumber("Distance to Target", distance);
        } else {
            // If no target is visible, stop the robot
            driveSubsystem.drive(0, 0, 0, false, 2);
        }
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

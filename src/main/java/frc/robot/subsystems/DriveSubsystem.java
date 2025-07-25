// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
//import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.pathConfig;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;


public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  //private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final Pigeon2 m_Pigeon2 = new Pigeon2(12);

  private final Field2d m_field = new Field2d();

  //private pathConfig pathConfig = new pathConfig(
  /*
  SwerveDrivePoseEstimator m_PoseEstimator = new SwerveDrivePoseEstimator(
    DriveConstants.kDriveKinematics, 
    Rotation2d.fromDegrees(-m_Pigeon2.getYaw().getValueAsDouble()), 
    new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()}, 
    getPose(),
    VecBuilder.fill(.05, .05, Units.degreesToRadians(5)),
    VecBuilder.fill(.5, .5, Units.degreesToRadians(30))
    );
  */

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(-m_Pigeon2.getYaw().getValueAsDouble()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });
  
  SwerveDriveKinematics m_kinematics;

  public static RobotConfig config;{
    try{
        config = pathConfig.fromGUISettings();
      } catch (Exception e) {
        e.printStackTrace();
      }
    }

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    zeroHeading();

    m_kinematics = new SwerveDriveKinematics(
            new Translation2d(Units.inchesToMeters(12.5), Units.inchesToMeters(12.5)), // Front Left
            new Translation2d(Units.inchesToMeters(12.5), Units.inchesToMeters(-12.5)), // Front Right
            new Translation2d(Units.inchesToMeters(-12.5), Units.inchesToMeters(12.5)), // Back Left
            new Translation2d(Units.inchesToMeters(-12.5), Units.inchesToMeters(-12.5))); // Back Right

    
    SmartDashboard.putData("Field", m_field);

    // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
    
    LimelightHelpers.setCameraPose_RobotSpace("", 
    .406,    // Forward offset (meters)
    0.050,    // Side offset (meters)
    0.330,    // Height offset (meters)
    0.0,    // Roll (degrees)
    0.0,   // Pitch (degrees)
    0.0     // Yaw (degrees)
    );
    
    // Configure AutoBuilder last
    
    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
            new PIDConstants(0.2, 0.0, 0), // Translation PID constants
            new PIDConstants(0.2, 0.0, 0) // Rotation PID constants
      ),
      config, // The robot configuration
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
     },
      this // Reference to this subsystem to set requirements
    );
    
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    /* 
    m_odometry.update(
        Rotation2d.fromDegrees(-m_Pigeon2.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    */
/* 
    if (DriverStation.isAutonomous()){
      LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      if (limelightMeasurement.tagCount >= 2) {  // Only trust measurement if we see multiple tags
        m_PoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
        m_PoseEstimator.addVisionMeasurement(
          limelightMeasurement.pose,
          limelightMeasurement.timestampSeconds
      );

        m_field.setRobotPose(m_PoseEstimator.getEstimatedPosition());
    
      }
      }
    */
  

    //SmartDashboard.putNumber("Swerve Gyro Angle", m_Pigeon2.getYaw().getValueAsDouble());

    SmartDashboard.putNumber("Elevator Offset", DriveConstants.h);

    SmartDashboard.putNumber("fRightDrive", m_frontRight.getRelativeEncoder());
    SmartDashboard.putNumber("fLeftDrive", m_frontLeft.getRelativeEncoder());
    SmartDashboard.putNumber("bRightDrive", m_rearRight.getRelativeEncoder());
    SmartDashboard.putNumber("bLeftDrive", m_rearLeft.getRelativeEncoder());
    
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
    //return m_PoseEstimator.getEstimatedPosition();
  }

  //public Pose2d getAutoPose(){
  //  return m_PoseEstimator.getEstimatedPosition();
  // }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(-m_Pigeon2.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
      }
/* 
    m_PoseEstimator.resetPosition(
        Rotation2d.fromDegrees(m_Pigeon2.getYaw().getValueAsDouble()), // This is what pathplanner uses for position
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        getAutoPose());
  
  }
*/

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(),
                                                           m_frontRight.getState(),
                                                           m_rearLeft.getState(),
                                                           m_rearRight.getState());
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }


  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, int speedMode) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = 0;
    double ySpeedDelivered = 0;
    double rotDelivered = 0;

    if(speedMode == 0){
      xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
      ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
      rotDelivered = rot * DriveConstants.kMaxAngularSpeed;
    }else if(speedMode == 1){
      xSpeedDelivered = xSpeed * (DriveConstants.kMaxSpeedMetersPerSecond*.5);
      ySpeedDelivered = ySpeed * (DriveConstants.kMaxSpeedMetersPerSecond*.5);
      rotDelivered = rot * (DriveConstants.kMaxAngularSpeed*.5);
    }else if(speedMode == 2){
      xSpeedDelivered = xSpeed * (DriveConstants.kMaxSpeedMetersPerSecond*.25);
      ySpeedDelivered = ySpeed * (DriveConstants.kMaxSpeedMetersPerSecond*.25);
      rotDelivered = rot * (DriveConstants.kMaxAngularSpeed*.25);
    }else{
      xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
      ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
      rotDelivered = rot * DriveConstants.kMaxAngularSpeed;
    }


    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(-m_Pigeon2.getYaw().getValueAsDouble()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[2]);
    m_frontRight.setDesiredState(swerveModuleStates[3]);
    m_rearLeft.setDesiredState(swerveModuleStates[0]);
    m_rearRight.setDesiredState(swerveModuleStates[1]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void changeModeValue(int value){
    DriveConstants.modeValue = value;
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    //m_gyro.reset();
    m_Pigeon2.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_Pigeon2.getYaw().getValueAsDouble()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_Pigeon2.getAngularVelocityZWorld().getValueAsDouble() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

}
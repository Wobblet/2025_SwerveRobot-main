// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ChangeSpeedModeCmd;
import frc.robot.commands.ClimbCmd;
import frc.robot.commands.SetFunnelCmd;
import frc.robot.commands.SetEndEffectorCmd;
import frc.robot.commands.elevatorEncoderCmd;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPositions;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final SendableChooser<Command> autoChooser;

  private final ElevatorSubsystem elevatorSubsystem;
  private final DriveSubsystem m_robotDrive;
  private final ClimberSubsystem climberSubsystem;
  private final FunnelSubsystem funnelSubsystem;
  public final EndEffectorSubsystem endEffectorSubsystem;

  // The operator's controller
  public final Joystick m_operatorController;

  // The driver's controller
  public final XboxController m_driverController;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
    public RobotContainer() {
        // Configure the button bindings
        elevatorSubsystem = new ElevatorSubsystem();
        m_robotDrive = new DriveSubsystem();
        climberSubsystem = new ClimberSubsystem();
        funnelSubsystem = new FunnelSubsystem();
        endEffectorSubsystem = new EndEffectorSubsystem();

        m_operatorController = new Joystick(OIConstants.kOperatorControllerPort);
        m_driverController = new XboxController(OIConstants.kDriverControllerPort);

        // Elevator Auto Commands
        NamedCommands.registerCommand("raiseFl", new InstantCommand(() -> elevatorSubsystem.setPosition(ElevatorPositions.FL)));
        NamedCommands.registerCommand("raiseL2", new InstantCommand(() -> elevatorSubsystem.setPosition(ElevatorPositions.L2)));
        NamedCommands.registerCommand("raiseL3", new InstantCommand(() -> elevatorSubsystem.setPosition(ElevatorPositions.L3)));
        NamedCommands.registerCommand("raiseL4", new InstantCommand(() -> elevatorSubsystem.setPosition(ElevatorPositions.L4)));
        
        // Score Auto Command
        NamedCommands.registerCommand("Score", new SetEndEffectorCmd(endEffectorSubsystem, -Constants.DriveConstants.scoreMotorSlowSpeed).withTimeout(1));
        NamedCommands.registerCommand("StopScore", new SetEndEffectorCmd(endEffectorSubsystem, 0).withTimeout(.5));

        // TODO Make am Auto Adjust Command Using Distance Sensor
        // TODO Make an Auto Align Command Using Limelight & AprilTags

        configureDriverBindings();

        configureOperatorBinding();

        //NamedCommands.registerCommand("[Pathplanner Name]", [Command to run]);
        //autoChooser = AutoBuilder.buildAutoChooser();

        // For convenience a programmer could change this when going to competition.
        boolean isCompetition = false;

        // Build an auto chooser. This will use Commands.none() as the default option.
        // As an example, this will only show autos that start with "comp" while at
        // competition as defined by the programmer
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
            (stream) -> isCompetition
            ? stream.filter(auto -> auto.getName().startsWith("comp"))
            : stream
        );

        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configure default commands
        m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true, DriveConstants.modeValue),
                m_robotDrive));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureDriverBindings() {
            // Speed Mode Controls
        /*L1 */ new JoystickButton(m_driverController, 5).whileTrue(new ChangeSpeedModeCmd(m_robotDrive, 1));
        /*R1 */ new JoystickButton(m_driverController, 6).whileTrue(new ChangeSpeedModeCmd(m_robotDrive, 2));
        /*X Button */ new JoystickButton(m_driverController, Button.kSquare.value).whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

            // Climber Controls  
        /*Start */ new JoystickButton(m_driverController, 8).whileTrue(new ClimbCmd(climberSubsystem, -Constants.DriveConstants.climberMotorSpeed));

            // Zero Heading
        /*Back */ new JoystickButton(m_driverController, 7).whileTrue(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));

            // Funnel Controls
        /*Y Button */ new JoystickButton(m_driverController, 4).whileTrue(new SetFunnelCmd(funnelSubsystem, Constants.DriveConstants.funnelMotorSpeed));
        /*A Button */ new JoystickButton(m_driverController, 1).whileTrue(new SetFunnelCmd(funnelSubsystem, -Constants.DriveConstants.resetFunnelMotorSpeed));

            // Score Controls
        /*B Button */ new JoystickButton(m_driverController, 2).whileTrue(new SetEndEffectorCmd(endEffectorSubsystem, -Constants.DriveConstants.retreatMotorSpeed));
        /*Right Triger*/ new Trigger(() -> m_driverController.getRawAxis(3) > .1).whileTrue(new SetEndEffectorCmd(endEffectorSubsystem, -DriveConstants.scoreMotorFullSpeed));
        /*Left Triger*/new Trigger(() -> m_driverController.getRawAxis(2) > .1).whileTrue(new SetEndEffectorCmd(endEffectorSubsystem, -DriveConstants.scoreMotorSlowSpeed));
    }

    private void configureOperatorBinding() {
        
            // Climber Controls
        /*Start Button*/ new JoystickButton(m_operatorController, 10).whileTrue(new ClimbCmd(climberSubsystem, -Constants.DriveConstants.climberMotorSpeed));
        /*Back Button */ new JoystickButton(m_operatorController, 9).whileTrue(new ClimbCmd(climberSubsystem, Constants.DriveConstants.resetClimberMotorSpeed));
        
            // Funnel Controls
        ///*L2 */ new JoystickButton(m_operatorController, 7).whileTrue(new RaiseFunnelCmd(funnelSubsystem, Constants.DriveConstants.funnelMotorSpeed));

            // Score Controls
        /*L3 */ new JoystickButton(m_operatorController, 11).whileTrue(new SetEndEffectorCmd(endEffectorSubsystem, -Constants.DriveConstants.scoreMotorFullSpeed));
        /*R3 */ new JoystickButton(m_operatorController, 12).whileTrue(new SetEndEffectorCmd(endEffectorSubsystem, Constants.DriveConstants.scoreMotorSlowSpeed));
        /*L1 */ new JoystickButton(m_operatorController, 5).whileTrue(new SetEndEffectorCmd(endEffectorSubsystem, -Constants.DriveConstants.retreatMotorSpeed));
        /*R1 */ new JoystickButton(m_operatorController, 6).whileTrue(new SetEndEffectorCmd(endEffectorSubsystem, -Constants.DriveConstants.scoreMotorSlowSpeed));

            // Elevator Controls
        /*R2 Button */ new JoystickButton(m_operatorController, 8).toggleOnTrue(new InstantCommand(() -> elevatorSubsystem.setPosition(ElevatorPositions.GL))).toggleOnFalse(new InstantCommand(() -> elevatorSubsystem.stopElevator()));
        /*X Button */ new JoystickButton(m_operatorController, 1).toggleOnTrue(new InstantCommand(() -> elevatorSubsystem.setPosition(ElevatorPositions.FL))).toggleOnFalse(new InstantCommand(() -> elevatorSubsystem.stopElevator()));
        /*Y Button */ new JoystickButton(m_operatorController, 4).toggleOnTrue(new InstantCommand(() -> elevatorSubsystem.setPosition(ElevatorPositions.L2))).toggleOnFalse(new InstantCommand(() -> elevatorSubsystem.stopElevator()));
        /*A Button */ new JoystickButton(m_operatorController, 2).toggleOnTrue(new InstantCommand(() -> elevatorSubsystem.setPosition(ElevatorPositions.L3))).toggleOnFalse(new InstantCommand(() -> elevatorSubsystem.stopElevator()));
        /*B Button */ new JoystickButton(m_operatorController, 3).toggleOnTrue(new InstantCommand(() -> elevatorSubsystem.setPosition(ElevatorPositions.L4))).toggleOnFalse(new InstantCommand(() -> elevatorSubsystem.stopElevator()));
                      new JoystickButton(m_operatorController, 3).toggleOnTrue(new InstantCommand(() -> elevatorSubsystem.resetH()));
            // Elevaotor Offsets
        /*Up Dpad */ new POVButton(m_operatorController, 0).whileTrue(new elevatorEncoderCmd(elevatorSubsystem, 1));
        /*Down Dpad */ new POVButton(m_operatorController, 180).whileTrue(new elevatorEncoderCmd(elevatorSubsystem, 2));

    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> m_robotDrive.resetPose(new Pose2d()), m_robotDrive),
        new WaitCommand(1),
        new RunCommand(() -> m_robotDrive.drive(-1, 0, 0, true, 1), m_robotDrive).withTimeout(.5),
        new WaitCommand(.5),
        new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, true, 1), m_robotDrive));
  }
/* 
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
       new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetPose(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, true, 1));
    
  }
*/
  public Command getAutonomousPathCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path\
    
    //return new PathPlannerAuto("New Auto");
    return autoChooser.getSelected();
  }
}
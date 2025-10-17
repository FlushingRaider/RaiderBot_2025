package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.io.File;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.ClimberUpCommand;
import frc.robot.Constants.ClawConstants;
// import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.miscConstants;
import frc.robot.commands.ClimberDownCommand;
import frc.robot.subsystems.Claw.Claw;
import frc.robot.subsystems.Climber.ClimbSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Swerve.Vision;
import frc.robot.subsystems.Elevator.ElevatorSubsystemSim;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  final         CommandXboxController DriveController = new CommandXboxController(0);
  final         CommandXboxController OPController = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
  public final ClimbSubsystem 
  m_climber = new ClimbSubsystem();
//======================Auton_Stuff=========================

  private final Command Leave;
  private final Command algae_Left;
  private final Command Elevator_Test;
  private final Command Middle_Coral;
  private final Command Left_Coral;
  private final Command Right_Coral;


  SendableChooser<Command> m_chooser;

  //=======================================================

  private final Claw sub_claw;
  private final ElevatorSubsystem elevator;
  private final ElevatorSubsystemSim elevatorsim;


  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                             () -> DriveController.getLeftY() * -1,
                                                             () -> DriveController.getLeftX() * -1)
                                                            //.withControllerRotationAxis(() -> DriveController.getRawAxis(2))
                                                            .withControllerRotationAxis(DriveController::getRightX)
                                                            .deadband(miscConstants.DEADBAND)
                                                            .scaleTranslation(0.20)
                                                            .scaleRotation(0.15)
                                                            .allianceRelativeControl(true);
            
 

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(DriveController::getLeftX,
                                                                                             DriveController::getLeftY)
                                                           .headingWhile(true);
                                                           

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(false)
                                                             .allianceRelativeControl(true);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                      () -> -DriveController.getLeftY(),
                                                                      () -> -DriveController.getLeftX())
                                                                    .withControllerRotationAxis(() -> DriveController.getRawAxis(
                                                                        // () -> -driverJoystick.getRawAxis(1),
                                                                        // () -> -driverJoystick.getRawAxis(0))
                                                                    // .withControllerRotationAxis(() -> driverJoystick.getRawAxis(
                                                                        2))
                                                                    .deadband(miscConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                DriveController.getRawAxis(
                                                                                                                  // driverJoystick.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                DriveController.getRawAxis(
                                                                                                                  // driverJoystick.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    sub_claw = new Claw();
    elevator = new ElevatorSubsystem();
    elevatorsim = new ElevatorSubsystemSim(elevator);




    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(false);


//========================Auton_Stuff===================================================
//=======================Nammed_Commands================================================

// NamedCommands.registerCommand("Saftey", Commands.run(() -> {
//   sub_claw.goToSetpoint(ClawConstants.Wrist.WristPositions.Elevator_Threh.getPos()); 
// }));
// NamedCommands.registerCommand("Elevator_Return", Commands.run(() -> {
//   elevator.goToSetpoint(ElevatorConstants.ElevatorConfigs.Positions.BOTTOM.getPos());
// }));
// NamedCommands.registerCommand("Algae_Level_1", Commands.run(() -> {
//   elevator.goToSetpoint(ElevatorConstants.ElevatorConfigs.Positions.Algae_1.getPos());
// }));
// NamedCommands.registerCommand("Algae_Level_2", Commands.run(() -> {
//   elevator.goToSetpoint(ElevatorConstants.ElevatorConfigs.Positions.Algae_2.getPos());
// }));

NamedCommands.registerCommand("Algae_Intake", Commands.run(() -> {
  sub_claw.goToSetpoint(ClawConstants.Wrist.WristPositions.Algae_Drive.getPos());
}));
NamedCommands.registerCommand("Algae_Intake_Rollers", Commands.run(() -> {
  sub_claw.setRollerPower(9); 
}));
NamedCommands.registerCommand("Stop", Commands.run(() -> {
  sub_claw.setRollerPower(0); 
}));
NamedCommands.registerCommand("Spit", Commands.run(() -> {
  sub_claw.setRollerPower(-8);  
}));
NamedCommands.registerCommand("Auto_Tip", Commands.run(() -> {
  sub_claw.goToSetpoint(ClawConstants.Wrist.WristPositions.Auto.getPos());
}));
NamedCommands.registerCommand("Climb_Up", Commands.run(() -> {
  OPController.back().whileTrue(new ClimberUpCommand(m_climber));
}));


    Leave = drivebase.getAutonomousCommand("Leave");
    algae_Left = drivebase.getAutonomousCommand("algae_Left");
    Elevator_Test = drivebase.getAutonomousCommand("Elevator Test");
    Middle_Coral = drivebase.getAutonomousCommand("Middle_Coral");
    Right_Coral = drivebase.getAutonomousCommand("Right Coral");
    Left_Coral = drivebase.getAutonomousCommand("Left Coral");
    

    m_chooser = new SendableChooser<Command>();

    m_chooser.addOption("Leave", Leave);
    m_chooser.addOption("algae_Left", algae_Left);
    m_chooser.addOption("Elevator Test", Elevator_Test);
    m_chooser.addOption("Middle Coral", Middle_Coral);
    m_chooser.addOption("Left Coral", Left_Coral);
    m_chooser.addOption("Right Coral", Right_Coral);


    SmartDashboard.putData(m_chooser);

//======================================================================================

  }
  private void configureBindings()
  {
        

 Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

 Command driveFieldOrientedDirectAngleKeyboard  = drivebase.driveFieldOriented(driveDirectAngleKeyboard);


   if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      DriveController.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      DriveController.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }


  
      
      //~~~~~~~~~~~~~~~~~~OPControler~~~~~~~~~~~~~~~~~~~~~~~~
      
      OPController.start().whileTrue(new ClimberUpCommand(m_climber));
      OPController.back().whileTrue(new ClimberDownCommand(m_climber));

      OPController.axisGreaterThan(2, 0.01).whileTrue(Commands.run(() -> {
        sub_claw.setRollerPower(((OPController.getRawAxis(2) * 0.85) * 12) * -1);
      }));
      OPController.axisGreaterThan(3, 0.01).whileTrue(Commands.run(() -> {
        sub_claw.setRollerPower((OPController.getRawAxis(3) * 0.85) * 12);
      }));

      OPController.axisLessThan(2, 0.01).and(OPController.axisLessThan(2,0.01)).whileTrue(Commands.run(() -> {
       sub_claw.setRollerPower(0);
      }).repeatedly());


      OPController.axisMagnitudeGreaterThan(1, 0.25).whileTrue(Commands.run(() -> {
        sub_claw.setWrist((OPController.getRawAxis(1) * (12 * 0.25)) * -1);
//        System.out.println("Wrist Bump");
      })).whileFalse(Commands.runOnce(() -> {
        sub_claw.goToSetpoint(sub_claw.getEncoderMeasurement());
      }));


      // OPController.povUp().onTrue(elevator.setHeightCommand(0.5));

      // OPController.povRight().whileTrue(Commands.run(() -> {
      //   elevator.goToSetpoint(ElevatorConstants.ElevatorConfigs.Positions.Algae_1.getPos());
      // }));
      // OPController.povUp().whileTrue(Commands.run(() -> {
      //   elevator.goToSetpoint(ElevatorConstants.ElevatorConfigs.Positions.Algae_2.getPos());
      // }));
      // OPController.povDown().whileTrue(Commands.run(() -> {
      //   sub_claw.goToSetpoint(ClawConstants.Wrist.WristPositions.Intake2.getPos());
      // }));

      // OPController.povLeft().whileTrue(Commands.run(() -> {
      //   elevator.goToSetpoint(ElevatorConstants.ElevatorConfigs.Positions.L4.getPos());
      // }));

      //Wrist Pos Control 
      // OPController.y().whileTrue(Commands.run(() -> {
      //   sub_claw.goToSetpoint(ClawConstants.Wrist.WristPositions.Home.getPos());
      //   elevator.goToSetpoint(ElevatorConstants.ElevatorConfigs.Positions.INTAKE.getPos());
      // }));

      OPController.x().whileTrue(Commands.run(() -> {

        sub_claw.goToSetpoint(ClawConstants.Wrist.WristPositions.L1_L2_Coral.getPos());
        
      }));

      // OPController.b().whileTrue(Commands.run(() -> {
      //   sub_claw.goToSetpoint(ClawConstants.Wrist.WristPositions.Floor.getPos());
      //   elevator.goToSetpoint(ElevatorConstants.ElevatorConfigs.Positions.FloorIntake.getPos());
      // }));

      OPController.a().whileTrue(Commands.run(() -> {
        sub_claw.goToSetpoint(ClawConstants.Wrist.WristPositions.Elevator_Threh.getPos());
      }));

      OPController.leftBumper().whileTrue(Commands.run(() -> {
        sub_claw.goToSetpoint(ClawConstants.Wrist.WristPositions.Algae_Drive.getPos());
      }));

      OPController.rightBumper().whileTrue(Commands.run(() -> {
        sub_claw.setRollerPower(-9);      
      }));

      //~~~~~~~~~~~~~~~~~~Drive Control~~~~~~~~~~~~~~~~~~~~~~~~
      DriveController.a().onTrue((Commands.runOnce(drivebase::zeroGyroWithAlliance)));

    


      // DriveController.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));

      //Go to pos ?
       //NOTE - Vision needs rewrite before ANY testing. Odom does not work and will throw the robot into a wall!!!!
      // DriveController.x().whileTrue(
      //   drivebase.driveToPose(new Pose2d(new Translation2d(14.4, 4.0), Rotation2d.fromDegrees(180))));

      //This is our boost control Right Trigger
      DriveController.axisGreaterThan(3, 0.01).onChange(Commands.runOnce(() -> {
        driveAngularVelocity.scaleTranslation(DriveController.getRightTriggerAxis() + 0.35);
        driveAngularVelocity.scaleRotation((DriveController.getRightTriggerAxis() * miscConstants.RotationSpeedScale) + 0.25);
      }).repeatedly()).whileFalse(Commands.runOnce(() -> { 
        driveAngularVelocity.scaleTranslation(0.25);
        driveAngularVelocity.scaleRotation(0.15);
      }).repeatedly());

       }

    
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
  }
  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}

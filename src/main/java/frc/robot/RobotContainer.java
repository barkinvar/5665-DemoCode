// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.TargetPoseSelector;
import frc.robot.subsystems.drive.TargetPoseSelector.TargetPose;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorTarget;
import frc.robot.subsystems.funnel.FunnelSubsystem;
import frc.robot.subsystems.gondik.Gondik;
import frc.robot.subsystems.gondik.Gondik.GondikTarget;
import frc.robot.subsystems.vision.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS5Controller m_driverController =
      new CommandPS5Controller(OperatorConstants.kDriverControllerPort);

  public final ElevatorSubsystem mElevator = new ElevatorSubsystem();
  public final FunnelSubsystem mFunnel = new FunnelSubsystem();
  public final Gondik mGondik = new Gondik();
  public final Superstructure mSuperstructure = new Superstructure(mElevator, mFunnel);

  private final CommandSwerveDrivetrain swerveDrivetrain = TunerConstants.createDrivetrain();
  private final Telemetry telemetry =
  new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
  
  private final TargetPoseSelector targetPoseSelector = new TargetPoseSelector(swerveDrivetrain);
  public final Vision mVision = new Vision(swerveDrivetrain);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveDrivetrain.registerTelemetry(telemetry::telemeterize);

    NamedCommands.registerCommand("L2ScoreRight", mElevator.runToSetpoint(ElevatorTarget.L2).alongWith(driveToClosestReef(TargetPose.RIGHT, targetPoseSelector, swerveDrivetrain)).andThen(mFunnel.shoot(3.5)));
    NamedCommands.registerCommand("ElevatorHome", mElevator.setForgetSetpointCommand(ElevatorTarget.HOME));
    NamedCommands.registerCommand("Intake", mFunnel.runHopperAtVoltageWithSensor(3.0));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    swerveDrivetrain.setDefaultCommand(
        swerveDrivetrain.teleopDrive(
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX()));

    m_driverController.R2().whileTrue(mFunnel.runHopperAtVoltageWithSensor(3.5));

    m_driverController.circle().onTrue(mSuperstructure.scoreToReef(ElevatorTarget.L2, m_driverController.L2()::getAsBoolean));
    m_driverController.square().onTrue(mSuperstructure.scoreToReef(ElevatorTarget.L3, m_driverController.L2()::getAsBoolean));
    m_driverController.triangle().onTrue(mSuperstructure.scoreToReef(ElevatorTarget.L4, m_driverController.L2()::getAsBoolean));

    m_driverController.cross().onFalse(mElevator.setForgetSetpointCommand(ElevatorTarget.HOME).alongWith(mGondik.setTargetCommand(GondikTarget.HOME)));
    m_driverController.L1().whileTrue(driveToClosestReef(TargetPose.LEFT, targetPoseSelector, swerveDrivetrain).onlyIf(mFunnel::getSensorState));
    m_driverController.R1().whileTrue(driveToClosestReef(TargetPose.RIGHT, targetPoseSelector, swerveDrivetrain).onlyIf(mFunnel::getSensorState));

    m_driverController.povUp().onTrue(mElevator.setForgetSetpointCommand(ElevatorTarget.VOLTAGE)).onFalse(mElevator.setForgetSetpointCommand(ElevatorTarget.HOME));
    m_driverController.povDown().whileTrue(mFunnel.setVoltageCommand(3.0));

    m_driverController.povLeft().onTrue(mElevator.runToSetpoint(ElevatorTarget.HIGH_GONDIK).andThen(mGondik.setTargetCommand(GondikTarget.GONDIK_LOW)));
    m_driverController.povRight().onTrue(mElevator.runToSetpoint(ElevatorTarget.LOW_GONDIK).andThen(mGondik.setTargetCommand(GondikTarget.GONDIK_LOW)));

  }



    public Command driveToClosestReef(TargetPose target, TargetPoseSelector selector, CommandSwerveDrivetrain drive) {
    return Commands.sequence(
        Commands.runOnce(() -> selector.updateTargetNearest(target)),
        new DriveToPose(
                () -> selector.getTargetPose().plus(new Transform2d(-0.18, 0.0, new Rotation2d())),
                true, drive)
            .unless(() -> selector.canTurnAtTime(false)),
        new DriveToPose(
            () -> selector.getTargetPose().plus(new Transform2d(-0.005, 0.0, new Rotation2d())),
            false, drive));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}

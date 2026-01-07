// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.FollowPathCommand;
// import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;


public class RobotContainer {
    public static NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    public static Field2d m_simField = new Field2d();

    private double MaxAngularRate = RotationsPerSecond.of(1.2).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest .PointWheelsAt();


    private final CommandXboxController joystick = new CommandXboxController(0);

    // public final Intake intakeSub = new Intake();

    public final Indexer indexerSub = new Indexer();

    public final Shooter shooterSub = new Shooter();

    private Swerve s_swerve = new Swerve();


    // private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // autoChooser = AutoBuilder.buildAutoChooser("Tests");
        // SmartDashboard.putData("Auto Mode", autoChooser);
        // FollowPathCommand.warmupCommand().schedule();

        configureBindings();
    }

    private void configureBindings() {
        
        // s_swerve.setDefaultCommand(
        //     new DriveCommand(
        //         s_swerve, 
        //         () -> -joystick.getLeftX(), 
        //         () -> joystick.getLeftY(),
        //         () -> joystick.getRightX(), 
        //         true
        //     )
        // );

        final var idle = new SwerveRequest.Idle();

        // joystick.leftTrigger().onTrue(new InstantCommand(() -> intakeSub.spin(0.3)));
        // joystick.leftTrigger().onFalse(new InstantCommand(() -> intakeSub.spin(0)));
 
        joystick.leftBumper().onTrue(new InstantCommand(() -> indexerSub.spinNoIntake(0.3)));
        joystick.leftBumper().onFalse(new InstantCommand(() -> indexerSub.spinNoIntake(0)));
        joystick.x().onTrue(new InstantCommand(() -> indexerSub.spinNoIntake(-0.3)));
        joystick.x().onFalse(new InstantCommand(() -> indexerSub.spinNoIntake(0)));



        // joystick.rightTrigger().onTrue(new InstantCommand(()-> shooterSub.spinWithFeed(1,1,indexerSub,intakeSub)));
        joystick.rightTrigger().onTrue(new InstantCommand(() -> shooterSub.spinNoFeed(0.5, 0.5)));
        joystick.rightTrigger().onFalse(new InstantCommand(() -> shooterSub.spinNoFeed(0, 0)));
        joystick.rightBumper().onTrue(new InstantCommand(() -> shooterSub.spinNoFeed(-0.5, -0.5)));
        joystick.rightBumper().onFalse(new InstantCommand(() -> shooterSub.spinNoFeed(0, 0)));




    }
    // public Command getAutonomousCommand() {
//     return autoChooser.getSelected(); 
// }
}

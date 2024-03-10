// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Auton;

import java.util.List;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.lib.util.SwerveModuleConstants;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.subsystems.*;
import frc.robot.Constants;

public class TwoShotAuton extends SequentialCommandGroup {
  //Creates a new TwoShotAuton.
  public static final Rotation2d rotationOffset = Rotation2d.fromDegrees(180);

    public TwoShotAuton(Swerve s_Swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
   // m_drivetrain = driveTrain;

    TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

    //Create a trajectory - like PathPlannertTrajectory
     // An example trajectory to follow.  All units in meters.
    
     //config.setReversed(true);
     Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
           // Pass through these two interior waypoints, making an 's' curve path
           List.of(new Translation2d(0.55, 0)),
           // End 3 meters straight ahead of where we started, facing forward
           new Pose2d(0.95, 0, new Rotation2d(0)),
              config);

              Trajectory exampleTrajectory2 =
              TrajectoryGenerator.generateTrajectory(
                  // Start at the origin facing the +X direction
                  new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(2, 2, new Rotation2d(0)),
                  config);

    //Create a theta controler (PID stuff)
    var thetaController =
    new ProfiledPIDController(
        Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //Create a swerve command with add command that only calls the drive train with the trajectory as a par
    SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

                
                SwerveControllerCommand swerveControllerCommand2 =
            new SwerveControllerCommand(
                exampleTrajectory2,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

                addCommands(
                  new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
                  swerveControllerCommand
                );
  }
}

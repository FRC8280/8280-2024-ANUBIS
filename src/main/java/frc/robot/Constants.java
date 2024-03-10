package frc.robot;
//import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.lib.PIDGains;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static double gyroOffset = 0;

    public static final Constraints kThetaControllerConstraints = null;

    public static class Intake {
        // Motors
        public static final int kIntakeMotorId = 12; //todo: change to ours
        public static final int kPivotMotorId = 13; //todo: change to ours
      
        // Absolute encoder offset
        public static final double k_pivotEncoderOffset = 0.166842; // Straight up, sketchy to reset to "up"
    
        
        // Pivot set point encoder values =
        public static final double k_pivotAngleGround = -55;
        public static final double k_pivotAngleSource = -30;
        public static final double k_pivotAngleAmp = k_pivotAngleSource;
        public static final double k_pivotAngleStow = 0;
    
        // Intake speeds
        public static final double k_intakeSpeed = 0.7;
        public static final double k_ejectSpeed = -0.45;
        public static final double k_feedShooterSpeed = -0.5;

        /*New commit from REV */
        public static final double kArmGearRatio = 1.0 / (100); 
        public static final double kPositionFactor = kArmGearRatio * 2.0 * Math.PI; //multiply SM value by this number and get arm position in radians
        public static final double kVelocityFactor = kArmGearRatio * 2.0 * Math.PI / 60.0;
        public static final double kArmFreeSpeed = 5676.0 * kVelocityFactor;
        public static final ArmFeedforward kArmFeedforward = new ArmFeedforward(0.0, 0.4, 12.0/kArmFreeSpeed, 0.0);
        public static final PIDGains kArmPositionGains = new PIDGains(0.6, 0.0, 0.0);
        
        public static final TrapezoidProfile.Constraints kArmMotionConstraint = new TrapezoidProfile.Constraints(2.0, 2.0);
        public static final double TrapezoidProfileMaxVel = 2.0;
        public static final double TrapezoidProfileMaxAcc = 2.0;

        public static final double kHomePosition = 0.0;
        public static final double kScoringPosition = 3.78;
        public static final double kIntakePosition = 1.82;
        public static final double kFeederPosition = 3.564;
        public static final double kHighPosition = 3.594; //3.3746;

        public static final double kSoftLimitReverse = 0; //-140;
        public static final double kSoftLimitForward = 4.77;//4.6 or 143.878

        public static final double kArmZeroCosineOffset = - Math.PI / 6; //radians to add to converted arm position to get real-world arm position (starts at ~30deg angle)
      }
      
      
      
    public static final class Swerve {  //todo: rename to Swerver Constants
        public static final int pigeonID = 9;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW- (DO NOT USE, ENABLES ROBOT-CENTRIC)

        public static final COTSFalconSwerveConstants chosenModule =  
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(24); 
        public static final double wheelBase = Units.inchesToMeters(19);
        public static final double centerToWheel = Math.sqrt(Math.pow(wheelBase / 2.0, 2) + Math.pow(trackWidth / 2.0, 2));
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Voltage Compensation */
        public static final double voltageComp = 12.0;

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.6; 
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.16861 / 12);
        public static final double driveKV = (2.6686 / 12);
        public static final double driveKA = (0.34757 / 12);

        /* Drive Motor Conversion Factors */
        public static final double driveConversionPositionFactor =
        wheelCircumference / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.1; 
        public static final double maxAccel = 4.1; 

        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0;

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kCoast;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 22;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(358.5); //127.3 original value
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 26;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(196.084);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 3; 
            public static final int angleMotorID = 4;
            public static final int canCoderID = 28;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(237.35);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID =7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 24;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(280.986);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants {
        public static final double slowVel = 1.5;
        public static final double slowAccel = 1.5;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final double kArmManualDeadband = 0.05;
        public static final double kArmManualScale = 0.5;
    }

    public static final class Climber{
        public static final int leftClimberID = 30;
        public static final int rightClimberID = 11;
        public static final double maxClimberHeight = 230;
        public static final double minClimberHeight = 0;
    }

    public static final class Shooter {
        public static final int kTopCanId = 16;
        public static final int kBottomCanId = 17;
        public static final int kIndexCanId = 14;
        public static final int kPivotShooterCanId = 15;
    
        public static final int kCurrentLimit = 80;
        public static final int kIndexCurrentLimit = 35;
        public static final int kSPivotCurrentLimit = 40;
    
        public static final double kTopPower = 0.7;
        public static final double kBottomPower = 0.8;
        public static final double kIndexPower = 0.9;
        public static final double kInputPower = 0.3;

        public static final double k_ShooterPivotLoaded = 25.9;

        public static final double k_ShooterRPM = 4000;//5000;
        public static final double k_ShooterTimer = 500;

        public static final double k_PivotMargin = 2;
        public static final double k_PivotZero = 2;
        public static final double k_PivotMiddle = 20;
        public static final double k_PivotNinety = 34;
        public static final double k_SubWoofer = 12.5;
        
        public static final double kNoteDetectionDistance = 110;
        
      }
}

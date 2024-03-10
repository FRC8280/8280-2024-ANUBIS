package frc.robot;
import edu.wpi.first.wpilibj2.command.*;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.Auton.TwoShotAuton;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climber;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.photonvision.targeting.PhotonTrackedTarget;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is aoperator
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

  
  public boolean singleDriver = true;

  //PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
  /* Controllers */

  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController operator = new CommandXboxController(1);
 
  public  double TargetYaw;

  /* Subsystems */
  
  public final Swerve s_Swerve = new Swerve();
  /*public final Climber m_Climber = new Climber();*/
  public final Shooter m_Shooter = new Shooter();
  public final Intake m_Intake = new Intake(this); 
  
  //LoadMakeReady m_LoadingCommand = new LoadMakeReady(m_Shooter, m_Intake);

  public double intakeVec = 0;
  public Command autoCode = Commands.sequence(new PrintCommand("no auto selected"));

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // Auto chooser
  SendableChooser<Command> chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, IO devices, and commands.
   */
  public RobotContainer() {

    SmartDashboard.putData("Auto Mode", chooser);
    chooser.addOption("Two Shots",new TwoShotAuton(s_Swerve));
    
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> driver.getRawAxis(translationAxis),
            () -> driver.getRawAxis(strafeAxis),
            () -> driver.getRawAxis(rotationAxis),
            () -> driver.leftBumper().getAsBoolean(),//() -> robotCentric.getAsBoolean(),
            () -> driver.rightBumper().getAsBoolean(),//() -> deadCat.getAsBoolean(),
            () -> driver.a().getAsBoolean(),//lockOn,
            this)
    );
   // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {

    CommandXboxController pointer;
    if(singleDriver)
      pointer = driver;
    else
      pointer = operator;
     
    //Operator Commands
    pointer.leftTrigger()
      .onTrue(Commands.parallel(new InstantCommand(() -> m_Shooter.pivotLoaded()),new InstantCommand(() -> m_Intake.goToGround())))
      .onFalse(new InstantCommand(() -> m_Intake.goToStow()));

      pointer.rightTrigger()  
      .onTrue(new BasicShoot(m_Shooter, m_Intake,Constants.Shooter.k_ShooterRPM,Constants.Shooter.k_PivotMiddle));

    pointer.b()
      .onTrue(new InstantCommand(()-> m_Shooter.SubWooferShot()));

    pointer.a()
      .onTrue(new InstantCommand(()-> m_Shooter.pivotFortyFive()));
    //climber commands

    /*
    pointer.povDown()
      .onTrue(new InstantCommand(() -> m_Climber.DecreasePower()))
      .onFalse(new InstantCommand(() -> m_Climber.NoPower()));
    pointer.povUp()
      .onTrue(new InstantCommand(() -> m_Climber.IncreasePower()))
      .onFalse(new InstantCommand(() -> m_Climber.NoPower()));   
    
    pointer.povRight()
      .onTrue(new InstantCommand(() -> m_Shooter.IncreasePivot(.25)));
    pointer.povLeft()
      .onFalse(new InstantCommand(() -> m_Shooter.DecreasePivot(.25)));
     
*/
 
    driver.y()
      .onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
  }

   public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  public void scheduleLoadingCommand(){
    //m_LoadingCommand.schedule();
    new LoadMakeReady(m_Shooter, m_Intake).schedule();
  }
  public void runRobotPeriodic(){

    /*if(m_Intake.getIntakeHasNote() && m_Intake.isPivotStowed())
      m_LoadingCommand.schedule();*/ 

      /*var result = camera.getLatestResult();
      PhotonTrackedTarget target = result.getBestTarget();

      if(target != null)
      {
          double yaw = target.getYaw();
          double pitch = target.getPitch();
          double area = target.getArea();
          double skew = target.getSkew();
          //Transform2d pose = target.getCameraToTarget();
          //List<TargetCorner> corners = target.getCorners();

          SmartDashboard.putNumber("Target yaw", target.getYaw());
          SmartDashboard.putNumber("Target pitch", target.getPitch());
          SmartDashboard.putNumber("Target area", target.getArea());
          SmartDashboard.putNumber("Target skew", target.getSkew());

          TargetYaw = target.getYaw();
      } 
      else
      { 
          TargetYaw = -5000;
      }
      
      SmartDashboard.putNumber("Pass Yaw to Drive system", TargetYaw);
*/
  }

  public void disablePIDSubsystems() {
    m_Shooter.StopAllMotors();
    m_Intake.StopAllMotors(); 
  }

  public void printValues() { // all of the SmartDashboard prints:
    //SmartDashboard.putBoolean("Pov pressed", e_presButton_0.getAsBoolean());
  
   /* SmartDashboard.putNumber("yaw", s_Swerve.gyro.getYaw());
    //m_Intake.outputTelemetry();

    SmartDashboard.putNumber("pitch", s_Swerve.gyro.getPitch());
    SmartDashboard.putNumber("roll", s_Swerve.gyro.getRoll());

    SmartDashboard.putNumber("odometry x", s_Swerve.getPose().getX());
    SmartDashboard.putNumber("odometry y", s_Swerve.getPose().getY()); */
    /* 
    SmartDashboard.putNumber("bpftX", limelight.botPoseX);
    SmartDashboard.putNumber("bpftZ", limelight.botPoseZ);
    SmartDashboard.putNumber("Limelight updates", limelight.updates);
    SmartDashboard.putNumber("Elevator Position", s_Elevator.getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator Target", s_Elevator.getTarget());
    SmartDashboard.putBoolean("elevator limit triggered?", s_Elevator.elevatorSwitchTriggered());
    SmartDashboard.putNumber("Wrist Position", s_Wrist.getEncoder().getPosition());
    SmartDashboard.putNumber("Wrist Target", s_Wrist.getTarget());
    SmartDashboard.putBoolean("cube beam broken?: ", s_Intake.cubeBeamBroken());
    SmartDashboard.putBoolean("cone beam broken?", s_Intake.coneBeamBroken());
    SmartDashboard.putNumber("intake speed", s_Intake.getSpeed());
    SmartDashboard.putNumber("yaw", s_Swerve.gyro.getYaw());
    SmartDashboard.putNumber("pitch", s_Swerve.gyro.getPitch());
    SmartDashboard.putNumber("roll", s_Swerve.gyro.getRoll());
    SmartDashboard.putNumber("elevatorPower", s_Elevator.elevatorPower());
    SmartDashboard.putNumber("rollLimelight", limelight.getRoll());
    SmartDashboard.putNumber("yawLimelight", limelight.getYaw());
    SmartDashboard.putNumber("pitchLimelight", limelight.getPitch());
    SmartDashboard.putNumber("odometry x", s_Swerve.getPose().getX());
    SmartDashboard.putNumber("odometry y", s_Swerve.getPose().getY());
    SmartDashboard.putData("Field", s_Swerve.getField2d());
    SmartDashboard.putNumber("RX", limelight.getRX());
    SmartDashboard.putNumber("RY", limelight.getRY());
    SmartDashboard.putNumber("RZ", limelight.getRZ());*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */


}

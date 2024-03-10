package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    //m_robotContainer.printValues();
    m_robotContainer.runRobotPeriodic();


   // m_allSubsystems.forEach(subsystem -> subsystem.outputTelemetry());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    //Lights.showLights("green");
    m_robotContainer.disablePIDSubsystems();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}


 
  private CANSparkMax m_testLeftClimber;
  private CANSparkMax m_testRightClimber;
  private CommandXboxController operator;
  

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
/*
    operator = new CommandXboxController(1);

    m_testLeftClimber =
      new CANSparkMax(Constants.Climber.leftClimberID, CANSparkLowLevel.MotorType.kBrushless);
    m_testRightClimber =
      new CANSparkMax(Constants.Climber.rightClimberID, CANSparkLowLevel.MotorType.kBrushless);
    
    m_testLeftClimber.enableSoftLimit(SoftLimitDirection.kForward, false);
    m_testRightClimber.enableSoftLimit(SoftLimitDirection.kForward, false);
    m_testLeftClimber.enableSoftLimit(SoftLimitDirection.kReverse, false);
    m_testRightClimber.enableSoftLimit(SoftLimitDirection.kReverse, false);
  */
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    /*
    if(Math.abs(operator.getLeftY()) > 0.25)
      m_testLeftClimber.set(operator.getLeftY());

    if(Math.abs(operator.getRightY()) > 0.25)
      m_testRightClimber.set(operator.getRightY()); */
  }
}

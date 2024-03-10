package frc.robot.commands;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class BasicShoot extends Command {

    private Shooter m_Shooter;
    private Intake m_Intake;
    private double m_Angle;
    private double m_Power;
    private Boolean abort;
    private Timer m_EmergencyTimer;

    public BasicShoot(Shooter shooterParam,Intake intake, double power, double angle){

        m_Shooter = shooterParam;
        m_Intake = intake;
        m_Angle = angle;
        m_Power = power;
        addRequirements(m_Shooter);
        addRequirements(m_Intake);
    }

    //Setup the 
    @Override public void initialize()
    {
        m_EmergencyTimer = new Timer();
        m_EmergencyTimer.start();
    
        System.out.printf("******Attempting to Fire******\n");
        if(!m_Shooter.getShooterHasNote())
        {
            System.out.printf("******No Note Detected******\n");
            abort = true;
        }
        else 
        {
            System.out.printf("******Setting motor power (Initialize)******\n");
             m_Shooter.m_shooterMotorSpeed = m_Power;
             abort = false;
        }
    }

    //periodic update
    @Override public void execute()
    {
        if(abort)   //Don't execute if we're aborting. 
          return;

        //Fire when it's reached the angle and the velocity is good
        if(m_Shooter.AtTargetSpeed() )//&& m_Shooter.AtTargetPivot())
        {
            m_Shooter.StartIndexMotor();
            m_Intake.ReverseIntake();
        }   
    }

    @Override public void end(boolean interrupted)
    {
        //System.out.printf("******Shutting Command Down******\n");
        m_Shooter.StopAllShooterMotors();
        m_Intake.StopIntakeMotor();
    }

    //quiting condition = look 
    @Override public boolean isFinished()
    {   
       //Emergency Timer
        if(m_EmergencyTimer.hasElapsed(5))
        {    
            System.out.printf("******Shoot command emergency timer******\n");
            return true; 
            //Todo: Is this the right situation in low battery?
            /*m_Shooter.StartIndexMotor();
            m_Intake.ReverseIntake();
            return false; */
        }
            
       //No note abort 
        if(abort)
        {   
            //System.out.printf("******Is finished calling abort******\n");
            return true;
        }
        //Note has fired shut down the system
        if(!m_Shooter.getShooterHasNote())
        {
            //System.out.printf("******No Note calling abort******\n");
            m_Shooter.pivotLoaded(); 
            return true;
        }
        return false;
    }
    
}

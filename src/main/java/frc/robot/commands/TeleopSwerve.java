package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier rightShoulderSup;
    private RobotContainer mRobotContainer;
    private BooleanSupplier mTargetButton;
    

    private boolean ControlsReleased()
    {       
        if((Math.abs(translationSup.getAsDouble()) < .1) && (Math.abs(strafeSup.getAsDouble()) < .1) && (Math.abs(rotationSup.getAsDouble()) < .1)  )
            return true;

        return false;
    }

    
    public TeleopSwerve(Swerve s_Swerve, 
    DoubleSupplier translationSup, 
    DoubleSupplier strafeSup, 
    DoubleSupplier rotationSup, 
    BooleanSupplier robotCentricSup, 
    BooleanSupplier rightShoulder,
        BooleanSupplier targetButton, 
        RobotContainer robotContainer)
        
        
        {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.rightShoulderSup = rightShoulder;
        this.mTargetButton = targetButton;
        this.mRobotContainer = robotContainer;

    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);


/*
        if(mTargetButton.getAsBoolean() && (mRobotContainer.TargetYaw!=-5000))
        {
            double Kp = -0.1f;
            double min_command = 0.05f;

            double heading_error = -mRobotContainer.TargetYaw;
            double steering_adjust = 0.0f;
            if (Math.abs(heading_error) < 1.0) 
            {
                if (heading_error < 0) 
                {
                    steering_adjust = Kp*heading_error + min_command;
                } 
                else 
                {
                    steering_adjust = Kp*heading_error - min_command;
                }
            } 
            rotationVal = steering_adjust;
            SmartDashboard.putNumber("rotationVal", rotationVal);
            //System.out.printf("******Try to Strafe******\n");
        }
 */
       /* if(ControlsReleased())
            s_Swerve.deadCat();*/
      /*  else if((target != null) && aSup.getAsBoolean()){
            System.out.printf("******Try to Strafe******\n");
        } */
        //else {
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        ); 
    }
}
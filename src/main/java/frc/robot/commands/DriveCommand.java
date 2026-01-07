package frc.robot.commands;
import static frc.robot.Constants.Joystick.*;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;



public class DriveCommand extends Command{
    private Swerve s_swerveBase;
    private DoubleSupplier m_translationSup, m_strafeSup, m_rotationSup;
    private boolean isFieldCentric;

    public DriveCommand(Swerve swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, boolean isFieldCentric){
        this.s_swerveBase = swerve;
        this.m_translationSup = translationSup;
        this.m_strafeSup = strafeSup;
        this.m_rotationSup = rotationSup;
        this.isFieldCentric = isFieldCentric;
        addRequirements(swerve);
    }
    @Override
    public void execute(){
        SmartDashboard.putBoolean("swerve", true);
        double translation = MathUtil.applyDeadband(this.m_translationSup.getAsDouble(), kStickDeadband);
        double strafe = MathUtil.applyDeadband(this.m_strafeSup.getAsDouble(), kStickDeadband);
        double rotation = MathUtil.applyDeadband(this.m_rotationSup.getAsDouble(), kRotationDeadband);
        SmartDashboard.putNumber("x", translation);
        SmartDashboard.putNumber("y", strafe);
        SmartDashboard.putBoolean("x == y", translation == strafe);
        
        s_swerveBase.drive(
            new Translation2d(translation, strafe).times(Constants.Swerve.maxDriveSpeed),
            rotation * Constants.Swerve.kMaxAngularVelocityRad,
            this.isFieldCentric
        );
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
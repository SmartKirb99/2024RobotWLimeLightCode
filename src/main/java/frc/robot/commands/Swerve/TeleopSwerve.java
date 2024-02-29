package frc.robot.commands.Swerve;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private DoubleSupplier m_translation;
    private DoubleSupplier m_strafe;
    private DoubleSupplier m_rotation;
    private BooleanSupplier m_robotCentric;
    private Swerve m_swerve;    

    /**
     * The default drive command.
     * @param translation Forward or back (-1 - 1).
     * @param strafe Side to side (-1 - 1).
     * @param rotation Spining (-1 - 1).
     * @param robotCentric Ignore the gyro or not.
     * @param swerve The Swerve subsystem.
     */
    public TeleopSwerve(DoubleSupplier translation, DoubleSupplier strafe, DoubleSupplier rotation, BooleanSupplier robotCentric, Swerve swerve) {
        m_swerve = swerve;
        addRequirements(m_swerve);

        m_translation = translation;
        m_strafe = strafe;
        m_rotation = rotation;
        m_robotCentric = robotCentric;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(m_translation.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(m_strafe.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(m_rotation.getAsDouble(), Constants.stickDeadband);

        rotationVal = Math.copySign(rotationVal * rotationVal, rotationVal);
        double linearMag = Math.hypot(translationVal, strafeVal);
        linearMag = Math.copySign(linearMag * linearMag, linearMag);
        Rotation2d angle = new Translation2d(translationVal, strafeVal).getAngle();

        /* Drive */
        m_swerve.drive(
            new Translation2d(linearMag, angle).times(Constants.SwerveConstants.maxSpeed), 
            rotationVal * Constants.SwerveConstants.maxAngularVelocity, 
            !m_robotCentric.getAsBoolean(), 
            false
        );
    }
}
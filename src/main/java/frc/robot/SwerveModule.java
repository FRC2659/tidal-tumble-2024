package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;
    private CANSparkMax mAngleMotor;
    private SparkPIDController mAnglePID;
    private TalonFX mDriveMotor;
    private RelativeEncoder mSteerEncoder;
    private CANcoder angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        // Angle Encoder Config 
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();

        // Angle Motor Config 
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        mAnglePID = mAngleMotor.getPIDController();
        mSteerEncoder = mAngleMotor.getEncoder();
        configAngleMotor();

        // Drive Motor Config 
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        // This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not 
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.setControl(new VelocityDutyCycle(velocity));
            //mDrivePID.setReference(velocity,ControlType.kVelocity); //BVN 1-24-24
          }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        lastAngle = angle;
        double steerOutput = (angle.getDegrees()*Constants.Swerve.angleGearRatio)/360;

        mAnglePID.setReference(steerOutput, ControlType.kPosition, 0);
        
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(mSteerEncoder.getPosition()/Constants.Swerve.angleGearRatio*360);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble()*360);
    }

    public void resetToAbsolute(){
        double absolutePositionNeo = -((getCanCoder().getDegrees() - angleOffset.getDegrees()) * Constants.Swerve.angleGearRatio) / 360;
        mSteerEncoder.setPosition(absolutePositionNeo);
    }

    private void configAngleEncoder(){        
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.clearFaults();
        mAngleMotor.restoreFactoryDefaults();
        mAngleMotor.setIdleMode(IdleMode.kCoast);
        mAnglePID.setP(0.2, 0);
        mAnglePID.setD(0.1, 0);
        mAngleMotor.setInverted(true);
        mAngleMotor.burnFlash();
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            mDriveMotor.getVelocity().getValue()/(Constants.Swerve.driveGearRatio/Constants.Swerve.wheelCircumference), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            -mDriveMotor.getPosition().getValue()/(Constants.Swerve.driveGearRatio/Constants.Swerve.wheelCircumference), 
            getAngle()
        );
    }
}
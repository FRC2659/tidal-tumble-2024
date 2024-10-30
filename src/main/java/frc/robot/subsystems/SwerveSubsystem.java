package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Vision;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
//import com.ctre.phoenix.sensors.PigeonIMU;
//import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.ctre.phoenix6.hardware.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.SwerveSubsystem;


public class SwerveSubsystem extends SubsystemBase {
    public static SwerveDriveOdometry swerveOdometry;
    public static SwerveModule[] mSwerveMods;
    public static Pigeon2 gyro;
    //public static PigeonIMU gyro; //2-3-24 quick fix
    private static double balanceDeadBand = 1.0;
    private static double balanceK = 22; //TUNE
    private static double output;
    private static boolean photonHasTarget = false;
    private Vision vision;
    private final SwerveDrivePoseEstimator poseEstimator;
    //private boolean init = true;
    public static Pose2d poseEstimatorPose; //static reference fix 2-29-2024

    private final Field2d m_field = new Field2d();
    
    public SwerveSubsystem() {
        //if(init){
        //    makeVision();
        //    init = false;
        //}
        //makeVision();

        vision = new Vision();
        SmartDashboard.putData("FIELD", m_field); 

        gyro = new Pigeon2(Constants.Swerve.pigeonID);  //2-3-24 temp fix
        gyro.getConfigurator().apply(new Pigeon2Configuration()); //2-3-24 temp fix

        //gyro = new PigeonIMU(Constants.Swerve.pigeonID);
        
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        // Define the standard deviations for the pose estimator, which determine how fast the pose
        // estimate converges to the vision measurement. This should depend on the vision measurement
        // noise
        // and how many or how frequently vision measurements are applied to the pose estimator.
        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        var visionStdDevs = VecBuilder.fill(.2, .2, .2);
        poseEstimator =
            new SwerveDrivePoseEstimator(
                    Constants.Swerve.swerveKinematics,
                    //getGyroYaw(),
                    getYaw(),
                    getModulePositions(), //causing crash on 1-27?
//                    new SwerveModulePosition[] {
//                        mSwerveMods[0].getPosition(),
//                        mSwerveMods[1].getPosition(),
//                        mSwerveMods[2].getPosition(),
//                        mSwerveMods[3].getPosition()
//                      },
                    new Pose2d(),
                    stateStdDevs,
                    visionStdDevs);

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, Rotation2d.fromDegrees(-getYaw().getDegrees()), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(new SwerveModuleState(-desiredStates[mod.moduleNumber].speedMetersPerSecond,desiredStates[mod.moduleNumber].angle), true);
        
        //mod.setDesiredState(desiredStates[mod.moduleNumber], true);
    }
    }    
    
    public Pose2d getPose() {
        return new Pose2d(swerveOdometry.getPoseMeters().getX(), swerveOdometry.getPoseMeters().getY(), swerveOdometry.getPoseMeters().getRotation());
    }

    public void stopMods() {
        SwerveModuleState[] zero = new SwerveModuleState[] {
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)),
        };
        setModuleStates(zero);
    }

    public static Pose2d getPoseSwerveOdometry() {
        return new Pose2d(swerveOdometry.getPoseMeters().getX(), swerveOdometry.getPoseMeters().getY(), swerveOdometry.getPoseMeters().getRotation());
    }

    public static Pose2d getPoseFromPoseEstimator() {
        return poseEstimatorPose;
    }
    
    public Pose2d getPoseFromPoseEstimator1() {
        return poseEstimatorPose;
    }
    
    public void resetOdometry(Pose2d pose) {
        Pose2d resetPose = new Pose2d( pose.getX(), pose.getY(), pose.getRotation());
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), resetPose);
    }

    public static void resetOdometryStatic(Pose2d pose) {
        Pose2d resetPose = new Pose2d( pose.getX(), pose.getY(), pose.getRotation());
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), resetPose);
    }

    public static double getAvgSwerveSpeeds(){
        double avgSpeed = 
           (Math.abs(mSwerveMods[0].getState().speedMetersPerSecond) +
            Math.abs(mSwerveMods[1].getState().speedMetersPerSecond) +
            Math.abs(mSwerveMods[2].getState().speedMetersPerSecond) +
            Math.abs(mSwerveMods[3].getState().speedMetersPerSecond) ) / 4;
        return avgSpeed;
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public static SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        if (RobotContainer.alliance == "RED") {
            gyro.setYaw(180);
        } else {
            gyro.setYaw(0);
        }
    }

    public static void gyroOffset(double deg){
        gyro.setYaw(deg);
    }

    public static Rotation2d getYaw() {
       SmartDashboard.putNumber("gyro angle", gyro.getAngle());
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(gyro.getAngle()) : Rotation2d.fromDegrees(-gyro.getAngle());  //2-19-24 inverted gyro
        //return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getAngle()) : Rotation2d.fromDegrees(gyro.getAngle());
//        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw()); //2-3-24 quick fix
    }

    public static void resetModulesToAbsolute(){
        Timer.delay(0.1);
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
            //Timer.delay(0.1);
        }
    }

//    public void makeVision(){
//        vision = new Vision();
//        //SmartDashboard.putNumber("debug2", 1);}

    /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}. */
    public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
    }

    /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}. */
    public void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        //SmartDashboard.putNumber("add Vision Meas", Timer.getFPGATimestamp());
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }

    public void resetPose(Pose2d pose) { //doesnt get used as of 1-27-24
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    /** Get the estimated pose of the swerve drive on the field. */
    public Pose2d getPoseEst() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void periodic(){

        //SmartDashboard.putNumber("debugSwerve-Periodic", Timer.getFPGATimestamp());

        var visionEst = vision.getEstimatedGlobalPose();
        visionEst.ifPresentOrElse(
        est -> {
            var estPose = est.estimatedPose.toPose2d();
            // Change our trust in the measurement based on the tags we can see 
            var estStdDevs = vision.getEstimationStdDevs(estPose);
            addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            SmartDashboard.putBoolean("Target in Sight", true);
            photonHasTarget = true;
            //SmartDashboard.putString("est pos",est.toString());
            //SmartDashboard.putNumber("debug1", Timer.getFPGATimestamp());
            if (!DriverStation.isAutonomous()) resetOdometry(getPoseEst());//1-27-24 BVN - if the estimate isnt null, then update swerve odom
        },() -> {
            SmartDashboard.putBoolean("Target in Sight", false);
            photonHasTarget = false;}
        );
        
        

        swerveOdometry.update(Rotation2d.fromDegrees(getYaw().getDegrees()), getModulePositions());  
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putString("Est Robot Location", getPoseEst().getTranslation().toString());
        m_field.setRobotPose(getPose());
        SmartDashboard.putData("FIELD", m_field); 


        // Update the odometry of the swerve drive using the wheel encoders and gyro.
        //poseEstimator.update(getGyroYaw(), getModulePositions());
        poseEstimator.update(getYaw(), getModulePositions());
        poseEstimatorPose = poseEstimator.getEstimatedPosition();

        //Prevents gyro from exceeding 360 degrees in either direction
        if (getYaw().getDegrees() > 360) {
            gyro.setYaw(getYaw().getDegrees() - 360);
        } else if (getYaw().getDegrees() < -360) {
            gyro.setYaw(getYaw().getDegrees() + 360);
        }

        for(SwerveModule mod : mSwerveMods){
           SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
           SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CanCoder", mod.getCanCoder().getDegrees());   
        }

        SmartDashboard.putNumber("Speed", mSwerveMods[0].getState().speedMetersPerSecond);
    }

}
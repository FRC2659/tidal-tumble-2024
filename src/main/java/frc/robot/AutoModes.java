package frc.robot;

import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SuperStructure;

public class AutoModes {

    public static boolean target = false;

    public static double sped = 63;
    public static double angle = 1.5;

    public SequentialCommandGroup chosenAuto(int id) {

        TrajectoryConfig trajectoryConfigTravel = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics).setReversed(false).setStartVelocity(0.5);

//            trajectoryConfigTravel.setReversed(true);
        
        TrajectoryConfig trajectoryConfigSlow = new TrajectoryConfig( 
                AutoConstants.kMaxSpeedMetersPerSecond*0.8,//*0.5
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics).setReversed(false).setEndVelocity(0);
        
        TrajectoryConfig trajectoryConfigSlowMid = new TrajectoryConfig( 
                AutoConstants.kMaxSpeedMetersPerSecond*0.4,//*0.5
                AutoConstants.kMaxAccelerationMetersPerSecondSquared*.85)
            .setKinematics(Constants.Swerve.swerveKinematics).setReversed(false).setEndVelocity(0);
        
        TrajectoryConfig trajectoryConfigIntake = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics).setReversed(false).setEndVelocity(2.0);
        
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(Constants.Swerve.swerveKinematics).setReversed(false);
            
        PIDController xController = new PIDController(AutoConstants.kPController, 0, AutoConstants.kDController);
        PIDController yController = new PIDController(AutoConstants.kPController, 0, AutoConstants.kDController);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        switch (id) {

            case 1: //AUTOOOOOOOOOOOOOOOOOOOOOO
                Trajectory driveBack = TrajectoryGenerator.generateTrajectory(new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
                    List.of(
                        new Translation2d(0.2, 0.2)),
                    new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(45.0)),
                    trajectoryConfigTravel);

                SwerveControllerCommand driveBackCommand = new SwerveControllerCommand(
                    driveBack,
                    RobotContainer.s_Swerve::getPoseEst,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

            return new SequentialCommandGroup(
                
                new InstantCommand(()-> SuperStructure.subShot()),
                new InstantCommand(()-> Timer.delay(0.25)),
                new InstantCommand(()-> SuperStructure.releaseShot()),
                new InstantCommand(()-> Timer.delay(0.15)),
                new InstantCommand(()-> SuperStructure.goHome()),
                new InstantCommand(()-> Timer.delay(0.3)), //added .05
                //new InstantCommand(()-> SuperStructure.intake()),
                new InstantCommand(()-> RobotContainer.s_Swerve.stopMods())
            );

            case 2: //4 piece Blue Middle
                Trajectory iB1 = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(54), Units.inchesToMeters(215), Rotation2d.fromDegrees(0)),
                    List.of(
                        new Translation2d(Units.inchesToMeters(58), Units.inchesToMeters(169)),
                        new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(169))),
                    new Pose2d(Units.inchesToMeters(63), Units.inchesToMeters(215), Rotation2d.fromDegrees(0)),
                    trajectoryConfigSlowMid);

                
                Trajectory BCenterNote3= TrajectoryGenerator.generateTrajectory(new Pose2d(Units.inchesToMeters(63), Units.inchesToMeters(215), Rotation2d.fromDegrees(0.0)),
                    List.of(
                        new Translation2d(16.54-12.5,5.25),
                        new Translation2d(16.54-10.75,4.0),
                        new Translation2d(16.54-8.8,4.1-.2)),
                    new Pose2d(16.54-8,4.1-.2, Rotation2d.fromDegrees(180)),
                    trajectoryConfigTravel);

                /*Trajectory sB1 = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(58), Units.inchesToMeters(215), Rotation2d.fromDegrees(0)),
                    List.of(

                        new Translation2d(Units.inchesToMeters(114), Units.inchesToMeters(223))), //adjusted -.2 after pmatch 10
                    new Pose2d(Units.inchesToMeters(58), Units.inchesToMeters(215), Rotation2d.fromDegrees(180)),
                    trajectoryConfig); */

                Trajectory iB2 = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(54), Units.inchesToMeters(215), Rotation2d.fromDegrees(0)),
                    List.of(
                        new Translation2d(Units.inchesToMeters(114-3), Units.inchesToMeters(225))), //adjusted -.2 after pmatch 10
                    new Pose2d(Units.inchesToMeters(63), Units.inchesToMeters(210), Rotation2d.fromDegrees(0)),
                    trajectoryConfigSlowMid);

                /*Trajectory sB2 = TrajectoryGenerator.generateTrajectory(new Pose2d(Units.inchesToMeters(291.611), Units.inchesToMeters(293.64), Rotation2d.fromDegrees(0.0)),
                    List.of(
                        new Translation2d(Units.inchesToMeters(80), Units.inchesToMeters(275.638)),
                        new Translation2d(Units.inchesToMeters(70), Units.inchesToMeters(218.638))),
                    new Pose2d(Units.inchesToMeters(41.76), Units.inchesToMeters(264.108), Rotation2d.fromDegrees(31.4)),
                    trajectoryConfig);*/

                Trajectory iB3 = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(54), Units.inchesToMeters(215), Rotation2d.fromDegrees(0)),
                    List.of(
                        new Translation2d(Units.inchesToMeters(70), Units.inchesToMeters(265)),
                        new Translation2d(Units.inchesToMeters(114), Units.inchesToMeters(265))),
                    new Pose2d(Units.inchesToMeters(63), Units.inchesToMeters(215), Rotation2d.fromDegrees(0)),
                    trajectoryConfigSlowMid);
                
                    
                /*Trajectory iB12 = iB1.concatenate(sB1);
                SwerveControllerCommand iB1Command = new SwerveControllerCommand(
                    iB12,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand sB1Command = new SwerveControllerCommand(
                    sB1,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);*/
                SwerveControllerCommand BCenterNote3Command = new SwerveControllerCommand(
                    BCenterNote3,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand iB2Command = new SwerveControllerCommand(
                    iB2,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);
                
                SwerveControllerCommand iB1Command = new SwerveControllerCommand(
                    iB1,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand iB3Command = new SwerveControllerCommand(
                    iB3,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                /*SwerveControllerCommand BlueCenterNote1Command = new SwerveControllerCommand(
                    BlueCenterNote1,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand BlueCenterNote2Command = new SwerveControllerCommand(
                    BlueCenterNote2,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand BlueCenterNote3Command = new SwerveControllerCommand(
                    BlueCenterNote3,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand BlueCenterNote4Command = new SwerveControllerCommand(
                    BlueCenterNote4,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);*/

            return new SequentialCommandGroup(
                
                new InstantCommand(()-> RobotContainer.s_Swerve.resetOdometry(iB1.getInitialPose())),
                new InstantCommand(()-> SuperStructure.goHome()), // added 10-19
                new InstantCommand(()-> Timer.delay(0.05)), // added 10-19     
                new InstantCommand(()-> SuperStructure.subShot()), 
                new InstantCommand(()-> Timer.delay(.3)),//added .2 10-19
                new InstantCommand(()-> SuperStructure.releaseShot()),
                new InstantCommand(()-> Timer.delay(0.15)),
                new InstantCommand(()-> SuperStructure.goHome()),
                new InstantCommand(()-> Timer.delay(0.25)), //added .05
                new InstantCommand(()-> SuperStructure.intake()),
                new InstantCommand(()-> Timer.delay(0.25+.25)), //added .25 10-19
                iB2Command,
                new InstantCommand(()-> SuperStructure.releaseShot()),
                new InstantCommand(()-> Timer.delay(0.1)),
                new InstantCommand(()-> SuperStructure.goHome()),
                new InstantCommand(()-> Timer.delay(0.25)),
                new InstantCommand(()-> SuperStructure.intake()),
                iB3Command,
                new InstantCommand(()-> SuperStructure.releaseShot()),
                new InstantCommand(()-> Timer.delay(0.1)),
                new InstantCommand(()-> SuperStructure.goHome()),
                new InstantCommand(()-> Timer.delay(0.25)),
                //new InstantCommand(()-> SuperStructure.intake()),
                //iB1Command,
                BCenterNote3Command,
                new InstantCommand(()-> SuperStructure.releaseShot()),
                new InstantCommand(()-> Timer.delay(0.15)),
                new InstantCommand(()-> SuperStructure.goHome())
            );

        case 3: //4 piece Red middle start
                /*Trajectory iR1 = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(597.043500), Units.inchesToMeters(215), Rotation2d.fromDegrees(180)),
                    List.of(
                        new Translation2d(Units.inchesToMeters(580), Units.inchesToMeters(174+3)),
                        new Translation2d(Units.inchesToMeters(537.222500), Units.inchesToMeters(174+3))),
                    new Pose2d(Units.inchesToMeters(590), Units.inchesToMeters(225), Rotation2d.fromDegrees(180)),
                    trajectoryConfigSlowMid);
                */
                Trajectory iR1 = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(597.043500), Units.inchesToMeters(215), Rotation2d.fromDegrees(180)),
                    List.of(
                        new Translation2d(Units.inchesToMeters(580), Units.inchesToMeters(174-3)),
                        new Translation2d(Units.inchesToMeters(537.222500), Units.inchesToMeters(174-3))),
                    new Pose2d(Units.inchesToMeters(590), Units.inchesToMeters(225), Rotation2d.fromDegrees(180)),
                    trajectoryConfigSlowMid);
                Trajectory sR1 = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(597.043500), Units.inchesToMeters(215), Rotation2d.fromDegrees(180)),
                    List.of(
                        new Translation2d(Units.inchesToMeters(537.222500), Units.inchesToMeters(218.638410))),
                    new Pose2d(Units.inchesToMeters(597.043500), Units.inchesToMeters(208), Rotation2d.fromDegrees(180)),
                    trajectoryConfig);

                Trajectory iR2 = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(597.043500), Units.inchesToMeters(215), Rotation2d.fromDegrees(180.0)),
                    List.of(
                        new Translation2d(Units.inchesToMeters(540), Units.inchesToMeters(223))), //adjusted -.2 after pmatch 10
                    new Pose2d(Units.inchesToMeters(597.043500), Units.inchesToMeters(215), Rotation2d.fromDegrees(180)),
                    trajectoryConfig);

                Trajectory iR3 = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(597.043500), Units.inchesToMeters(215), Rotation2d.fromDegrees(180.0)),
                    List.of(
                        new Translation2d(Units.inchesToMeters(580), Units.inchesToMeters(265)),
                        new Translation2d(Units.inchesToMeters(543), Units.inchesToMeters(265))),
                    new Pose2d(Units.inchesToMeters(597.043500), Units.inchesToMeters(225), Rotation2d.fromDegrees(180)),
                    trajectoryConfigSlowMid);

                Trajectory RCenterNote1= TrajectoryGenerator.generateTrajectory(new Pose2d(15.5,5.64, Rotation2d.fromDegrees(180.0)),
                    List.of(
                        new Translation2d(14,6.5),
                        new Translation2d(11,6.75),
                        new Translation2d(8.8,7.5-.2),
                        new Translation2d(9.5,7.5),
                        new Translation2d(12,6.75),
                        new Translation2d(14,6.5)),
                    new Pose2d(15.5,5.64, Rotation2d.fromDegrees(180-28.6)),
                    trajectoryConfigTravel);

                Trajectory RCenterNote2= TrajectoryGenerator.generateTrajectory(new Pose2d(15.5,5.64, Rotation2d.fromDegrees(180.0)),
                    List.of(
                        new Translation2d(14,6.5),
                        new Translation2d(11,6.75),
                        new Translation2d(8.8,5.85-0.2), //adjusted after pmatch 10
                        new Translation2d(9.5,6.2),
                        new Translation2d(11,6.75),
                        new Translation2d(14,6.5)),
                    new Pose2d(15.5,5.64, Rotation2d.fromDegrees(180-28.6)),
                    trajectoryConfigTravel);

                /*Trajectory RCenterNote3= TrajectoryGenerator.generateTrajectory(new Pose2d(15.5,5.64, Rotation2d.fromDegrees(180.0)),
                    List.of(
                        new Translation2d(12.5,5.25),
                        new Translation2d(10.75,4.0),
                        new Translation2d(8.8,4.1-.2),
                        new Translation2d(9.5,4.15),
                        new Translation2d(10.75,4.0),
                        new Translation2d(12.5,5.25)),
                    new Pose2d(15.5,5.64, Rotation2d.fromDegrees(180-28.6)),
                    trajectoryConfigTravel);*/
                Trajectory RCenterNote3= TrajectoryGenerator.generateTrajectory(new Pose2d(Units.inchesToMeters(597.043500), Units.inchesToMeters(225), Rotation2d.fromDegrees(180.0)),
                    List.of(
                        new Translation2d(12.5,5.25),
                        new Translation2d(10.75,4.0),
                        new Translation2d(8.8,4.1-.2)),
                    new Pose2d(8.8-.8,4.1-.2, Rotation2d.fromDegrees(0)),
                    trajectoryConfigTravel);

                Trajectory RCenterNote4= TrajectoryGenerator.generateTrajectory(new Pose2d(15.5,5.64, Rotation2d.fromDegrees(180.0)),
                    List.of(
                        new Translation2d(12.5,5.25),
                        new Translation2d(10.75,4.0),
                        new Translation2d(9.0,2.7),
                        new Translation2d(8.8,2.55-.2),
                        new Translation2d(9.0,2.75),
                        new Translation2d(10.75,4.0),
                        new Translation2d(12.5,5.25)),
                    new Pose2d(15.5,5.64, Rotation2d.fromDegrees(180-28.6)),
                    trajectoryConfigTravel);

                //Trajectory iR12 = iR1.concatenate(sR1);
                SwerveControllerCommand iR1Command = new SwerveControllerCommand(
                    iR1,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand iR2Command = new SwerveControllerCommand(
                    iR2,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand iR3Command = new SwerveControllerCommand(
                    iR3,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);
                    
                SwerveControllerCommand RCenterNote1Command = new SwerveControllerCommand(
                    RCenterNote1,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand RCenterNote2Command = new SwerveControllerCommand(
                    RCenterNote2,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand RCenterNote3Command = new SwerveControllerCommand(
                    RCenterNote3,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);
                    
                SwerveControllerCommand RCenterNote4Command = new SwerveControllerCommand(
                    RCenterNote4,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

            return new SequentialCommandGroup(
                new InstantCommand(()-> RobotContainer.s_Swerve.resetOdometry(iR1.getInitialPose())),
                /*new InstantCommand(()-> SuperStructure.subShot()),
                new InstantCommand(()-> Timer.delay(0.25)),
                new InstantCommand(()-> SuperStructure.releaseShot()),
                new InstantCommand(()-> Timer.delay(0.15)),
                new InstantCommand(()-> SuperStructure.goHome()),
                new InstantCommand(()-> Timer.delay(0.3)), //added .05
                new InstantCommand(()-> SuperStructure.intake()),*/

                new InstantCommand(()-> SuperStructure.goHome()), // added 10-19
                new InstantCommand(()-> Timer.delay(0.05)), // added 10-19     
                new InstantCommand(()-> SuperStructure.subShot()), 
                new InstantCommand(()-> Timer.delay(.3)),//added .2 10-19
                new InstantCommand(()-> SuperStructure.releaseShot()),
                new InstantCommand(()-> Timer.delay(0.15)),
                new InstantCommand(()-> SuperStructure.goHome()),
                new InstantCommand(()-> Timer.delay(0.25)), //added .05
                new InstantCommand(()-> SuperStructure.intake()),
                new InstantCommand(()-> Timer.delay(0.25+.25)), //added .25 10-19

                iR2Command,
                //sR1Command,
                //new InstantCommand(()-> SuperStructure.subShot()),
                //new InstantCommand(()-> Timer.delay(0.5)),
                new InstantCommand(()-> SuperStructure.releaseShot()),
                new InstantCommand(()-> Timer.delay(0.15)),
                new InstantCommand(()-> SuperStructure.goHome()),
                new InstantCommand(()-> Timer.delay(0.3)),
                new InstantCommand(()-> SuperStructure.intake()),
                iR3Command,
                new InstantCommand(()-> SuperStructure.releaseShot()),
                new InstantCommand(()-> Timer.delay(0.15)),
                new InstantCommand(()-> SuperStructure.goHome()),
                new InstantCommand(()-> Timer.delay(0.3)),
                //new InstantCommand(()-> SuperStructure.intake()),
                RCenterNote3Command,
                new InstantCommand(()-> SuperStructure.releaseShot()),
                new InstantCommand(()-> Timer.delay(0.15)),
                new InstantCommand(()-> SuperStructure.goHome()),
                new InstantCommand(()-> Timer.delay(0.3)),
                new InstantCommand(()-> SuperStructure.intake()),
//                RCenterNote4Command,
                new InstantCommand(()-> SuperStructure.goHome())
            );
            
            case 4: // Blue Ampside
                Trajectory blueAmpsideCloseNote1 = TrajectoryGenerator.generateTrajectory(new Pose2d(0.6,6.7, Rotation2d.fromDegrees(60)),
                    List.of(
                        new Translation2d(Units.inchesToMeters(70), Units.inchesToMeters(265)),
                        new Translation2d(Units.inchesToMeters(114), Units.inchesToMeters(265))),
                    new Pose2d(Units.inchesToMeters(114), Units.inchesToMeters(265), Rotation2d.fromDegrees(0)),
                    trajectoryConfigSlow);

                Trajectory blueAmpsideCloseNote1Return = TrajectoryGenerator.generateTrajectory(new Pose2d(Units.inchesToMeters(114), Units.inchesToMeters(265), Rotation2d.fromDegrees(0)),
                    List.of(
                        new Translation2d(Units.inchesToMeters(70), Units.inchesToMeters(265)),
                        new Translation2d(Units.inchesToMeters(114), Units.inchesToMeters(265))),
                        new Pose2d(0.6,6.7, Rotation2d.fromDegrees(60)),
                    trajectoryConfig);

                Trajectory blueAmpsideCloseNote2 = TrajectoryGenerator.generateTrajectory(new Pose2d(0.6,6.7, Rotation2d.fromDegrees(60)),
                    List.of(
                        new Translation2d(2, 7),
                        new Translation2d(2.3, 7),
                        new Translation2d(2.2, 6.5)),
                    new Pose2d(0.6, 6.7, Rotation2d.fromDegrees(60)),
                    trajectoryConfig);


                Trajectory BlueAmpsideCenterNote1 = TrajectoryGenerator.generateTrajectory(new Pose2d(0.6, 6.7, Rotation2d.fromDegrees(60.0-28.6)),
                    List.of(
                        new Translation2d(2.5 ,6.5),
                        new Translation2d(5.5 ,6.75),
                        new Translation2d(7.7 ,7.5 - .25),
                        new Translation2d(7,7.5),
                        new Translation2d(4.5 ,6.75),
                        new Translation2d(2.5 ,6.5)),
                    new Pose2d(0.6, 6.74, Rotation2d.fromDegrees(60-28.6)),
                    trajectoryConfig);

                Trajectory BlueAmpsideCenterNote2 = TrajectoryGenerator.generateTrajectory(new Pose2d(0.6, 6.7, Rotation2d.fromDegrees(60.0-28.6)),
                    List.of(
                        new Translation2d(2.5 ,6.5),
                        new Translation2d(5.5 ,6.75),
                        new Translation2d(7.7 ,5.85),
                        new Translation2d(7,6.2),
                        new Translation2d(5.5 ,6.75),
                        new Translation2d(2.5 ,6.5)),
                    new Pose2d(0.6, 6.7, Rotation2d.fromDegrees(60-28.6)),
                    trajectoryConfig);

                Trajectory BlueAmpsideCenterNote3 = TrajectoryGenerator.generateTrajectory(new Pose2d(0.6, 6.7, Rotation2d.fromDegrees(60.0-28.6)),
                    List.of(
                        new Translation2d(3 ,6.5),
                        new Translation2d(4 ,5.25),
                        new Translation2d(5.75 ,4),
                        new Translation2d(7.7 ,4.1),
                        new Translation2d(7,4.15),
                        new Translation2d(5.75 ,4),
                        new Translation2d(4.0 ,5.25),
                        new Translation2d(3.0 ,6.5)),
                    new Pose2d(1, 5.64, Rotation2d.fromDegrees(60-28.6)),
                    trajectoryConfig);

                Trajectory BlueAmpsideCenterNote4 = TrajectoryGenerator.generateTrajectory(new Pose2d(0.6, 6.7, Rotation2d.fromDegrees(60.0-28.6)),
                    List.of(
                        new Translation2d(3 ,6.5),
                        new Translation2d(4 ,5.25),
                        new Translation2d(5.75 ,4),
                        new Translation2d(7.5 ,2.7),
                        new Translation2d(7.7 ,2.55),
                        new Translation2d(7.5 ,2.75),
                        new Translation2d(5.75 ,4),
                        new Translation2d(4.0 ,5.25),
                        new Translation2d(3 ,6.5)),
                    new Pose2d(0.6, 6.7, Rotation2d.fromDegrees(60-28.6)),
                    trajectoryConfig);
                    

                SwerveControllerCommand blueAmpsideCloseNote1Command = new SwerveControllerCommand(
                    blueAmpsideCloseNote1,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand blueAmpsideCloseNote1ReturnCommand = new SwerveControllerCommand(
                    blueAmpsideCloseNote1Return,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand blueAmpsideCloseNote2Command = new SwerveControllerCommand(
                    blueAmpsideCloseNote2,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);


                SwerveControllerCommand BlueAmpsideCenterNote1Command = new SwerveControllerCommand(
                    BlueAmpsideCenterNote1,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand BlueAmpsideCenterNote2Command = new SwerveControllerCommand(
                    BlueAmpsideCenterNote2,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand BlueAmpsideCenterNote3Command = new SwerveControllerCommand(
                    BlueAmpsideCenterNote3,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand BlueAmpsideCenterNote4Command = new SwerveControllerCommand(
                    BlueAmpsideCenterNote4,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

            return new SequentialCommandGroup(
                new InstantCommand(()-> RobotContainer.s_Swerve.resetOdometry(blueAmpsideCloseNote1.getInitialPose())), //center start
                new InstantCommand(()-> SuperStructure.subShot()),
                new InstantCommand(()-> Timer.delay(0.25)),
                new InstantCommand(()-> SuperStructure.releaseShot()),
                new InstantCommand(()-> Timer.delay(0.15)),
                new InstantCommand(()-> SuperStructure.goHome())
                //new InstantCommand(()-> Timer.delay(0.3)), //added .05
                //new InstantCommand(()-> SuperStructure.intake()),
                //blueAmpsideCloseNote1Command,
                //new InstantCommand(()-> SuperStructure.subShot()),
                //blueAmpsideCloseNote1ReturnCommand,
                //blueAmpsideCloseNote2Command,
                //sB2Command,
                //new InstantCommand(()-> SuperStructure.subShot()),
                //new InstantCommand(()-> Timer.delay(0.5)),
                //new InstantCommand(()-> SuperStructure.releaseShot()),
                //new InstantCommand(()-> Timer.delay(0.15)),
                //new InstantCommand(()-> SuperStructure.goHome())
            );

        case 5: //4 piece Red
                Trajectory redAmpsideCloseNote1 = TrajectoryGenerator.generateTrajectory(new Pose2d(15.9, 6.7, Rotation2d.fromDegrees(180-60)),
                    List.of(
                        new Translation2d(Units.inchesToMeters(580), Units.inchesToMeters(265)),
                        new Translation2d(Units.inchesToMeters(543), Units.inchesToMeters(265))),
                    new Pose2d(Units.inchesToMeters(543), Units.inchesToMeters(265), Rotation2d.fromDegrees(180)),
                    trajectoryConfigSlowMid);

                Trajectory redAmpsideCloseNote1Return = TrajectoryGenerator.generateTrajectory(new Pose2d(15.9, 6.7, Rotation2d.fromDegrees(180)),
                    List.of(
                        new Translation2d(Units.inchesToMeters(580), Units.inchesToMeters(265)),
                        new Translation2d(Units.inchesToMeters(543), Units.inchesToMeters(265))),
                    new Pose2d(Units.inchesToMeters(543), Units.inchesToMeters(265), Rotation2d.fromDegrees(180-60)),
                    trajectoryConfigSlowMid);

                Trajectory redAmpsideCloseNote2 = TrajectoryGenerator.generateTrajectory(new Pose2d(15.9, 6.7, Rotation2d.fromDegrees(180-60-28.6)),
                    List.of(
                        new Translation2d(14.5, 7),
                        new Translation2d(14.2, 7),
                        new Translation2d(14.3, 6.5)),
                    new Pose2d(15.9, 6.7, Rotation2d.fromDegrees(180-60-28.6)),
                    trajectoryConfig);

                Trajectory redAmpsideCenterNote1= TrajectoryGenerator.generateTrajectory(new Pose2d(15.9,6.7, Rotation2d.fromDegrees(180.0-60-28.6)),
                    List.of(
                        new Translation2d(14,6.5),
                        new Translation2d(11,6.75),
                        new Translation2d(8.8,7.5),
                        new Translation2d(9.5,7.5),
                        new Translation2d(12,6.75),
                        new Translation2d(14,6.5)),
                    new Pose2d(15.9,6.7, Rotation2d.fromDegrees(180-60-28.6)),
                    trajectoryConfig);

                Trajectory redAmpsideCenterNote2= TrajectoryGenerator.generateTrajectory(new Pose2d(15.9,6.7, Rotation2d.fromDegrees(180.0-60-28.6)),
                    List.of(
                        new Translation2d(14,6.5),
                        new Translation2d(11,6.75),
                        new Translation2d(8.8,5.85),
                        new Translation2d(9.5,6.2),
                        new Translation2d(11,6.75),
                        new Translation2d(14,6.5)),
                    new Pose2d(15.9,6.7, Rotation2d.fromDegrees(180-60-28.6)),
                    trajectoryConfig);

                Trajectory redAmpsideCenterNote3= TrajectoryGenerator.generateTrajectory(new Pose2d(15.9,6.7, Rotation2d.fromDegrees(180.0-60-28.6)),
                    List.of(
                        new Translation2d(13.5,6.5),
                        new Translation2d(12.5,5.25),
                        new Translation2d(10.75,4),
                        new Translation2d(8.8,4.1),
                        new Translation2d(9.5,4.15),
                        new Translation2d(10.75,4.0),
                        new Translation2d(12.5,5.25),
                        new Translation2d(13.5,6.5)),
                    new Pose2d(15.9,6.7, Rotation2d.fromDegrees(180-60-28.6)),
                    trajectoryConfigTravel);

                Trajectory redAmpsideCenterNote4= TrajectoryGenerator.generateTrajectory(new Pose2d(15.9,6.7, Rotation2d.fromDegrees(180.0-60-28.6)),
                    List.of(
                        new Translation2d(13.5,6.5),
                        new Translation2d(12.5,5.25),
                        new Translation2d(10.75,4.0),
                        new Translation2d(9.0,2.7),
                        new Translation2d(8.8,2.55),
                        new Translation2d(9.0,2.75),
                        new Translation2d(10.75,4.0),
                        new Translation2d(12.5,5.25),
                        new Translation2d(13.5,6.5)),
                    new Pose2d(15.9,6.7, Rotation2d.fromDegrees(180-60-28.6)),
                    trajectoryConfigTravel);

                SwerveControllerCommand redAmpsideCloseNote1Command = new SwerveControllerCommand(
                    redAmpsideCloseNote1,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand redAmpsideCloseNote1ReturnCommand = new SwerveControllerCommand(
                    redAmpsideCloseNote1,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand redAmpsideCloseNote2Command = new SwerveControllerCommand(
                    redAmpsideCloseNote2,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand redAmpsideCenterNote1Command = new SwerveControllerCommand(
                    redAmpsideCenterNote1,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);
                
                SwerveControllerCommand redAmpsideCenterNote2Command = new SwerveControllerCommand(
                    redAmpsideCenterNote2,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand redAmpsideCenterNote3Command = new SwerveControllerCommand(
                    redAmpsideCenterNote3,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);
                    
                SwerveControllerCommand redAmpsideCenterNote4Command = new SwerveControllerCommand(
                    redAmpsideCenterNote4,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

            return new SequentialCommandGroup(
                new InstantCommand(()-> RobotContainer.s_Swerve.resetOdometry(redAmpsideCloseNote1.getInitialPose())), //center start
                new InstantCommand(()-> SuperStructure.subShot()),
                new InstantCommand(()-> Timer.delay(0.25)),
                new InstantCommand(()-> SuperStructure.releaseShot()),
                new InstantCommand(()-> Timer.delay(0.15)),
                new InstantCommand(()-> SuperStructure.goHome()),
                new InstantCommand(()-> Timer.delay(0.3)) //added .05
                //new InstantCommand(()-> SuperStructure.intake()),
                //redAmpsideCloseNote1Command,
                //new InstantCommand(()-> SuperStructure.subShot()),
                //redAmpsideCloseNote1ReturnCommand,
                //new InstantCommand(()-> SuperStructure.subShot()),
                //new InstantCommand(()-> Timer.delay(0.5)),
                //new InstantCommand(()-> SuperStructure.releaseShot()),
                //new InstantCommand(()-> Timer.delay(0.15)),
                //new InstantCommand(()-> SuperStructure.goHome())
            );
            
            case 6: // Blue Source Side
                Trajectory blueSourceSideCenterNote1Drive = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0.87,4.26, Rotation2d.fromDegrees(-60)),
                    List.of(
                        new Translation2d(3.5,2)),
                    new Pose2d(7.62 + .8,0.9 - .25-.2, Rotation2d.fromDegrees(0)),
                    trajectoryConfigSlow);

                Trajectory blueSourceSideCenterNote1Return = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(7.62 + .8,0.75, Rotation2d.fromDegrees(0)),
                    List.of(
                        new Translation2d(4,2)),
                    new Pose2d(0.87,4.26, Rotation2d.fromDegrees(-60)),//3.18,3.0,-42
                    trajectoryConfigSlow);


                Trajectory blueSourceSideCenterNote2Drive = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0.87,4.26, Rotation2d.fromDegrees(-60)),
                    List.of(
                        new Translation2d(3.5,1),
                        new Translation2d(6,1),
                        new Translation2d(7.24,2.6)), 
                    new Pose2d(8.3,2.5, Rotation2d.fromDegrees(0)),
                    trajectoryConfigSlow);

                Trajectory blueSourceSideCenterNote2Return = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(8.3,2.5, Rotation2d.fromDegrees(0)),
                    List.of(
                        new Translation2d(7,2),
                        new Translation2d(3.5,2)),
                    new Pose2d(0.87,4.26, Rotation2d.fromDegrees(-60)),
                    trajectoryConfigSlow);

                Trajectory blueSourceSideCenterNote3Drive = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(1, 4.5, Rotation2d.fromDegrees(-60.0-28.6)),
                    List.of(
                        new Translation2d(2 ,3),
                        new Translation2d(4 ,3),
                        new Translation2d(6 ,4.1),
                        new Translation2d(6.9 ,3.25)),
                    new Pose2d(7.3, 4.1, Rotation2d.fromDegrees(30)),
                    trajectoryConfigSlow);
                    
                Trajectory blueSourceSideTEST = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0.87,4.26, Rotation2d.fromDegrees(-60)),
                    List.of(
                        new Translation2d(1,5)),
                    new Pose2d(1,5.5, Rotation2d.fromDegrees(60)),
                    trajectoryConfigSlow);

                Trajectory blueSourceSideCenterNote3Return = TrajectoryGenerator.generateTrajectory(new Pose2d(7.3, 4.1, Rotation2d.fromDegrees(30)),
                    List.of(
                        new Translation2d(6, 4.1),
                        new Translation2d(4, 3),
                        new Translation2d(2, 3)),
                    new Pose2d(1, 4.5, Rotation2d.fromDegrees(-60-28.6)),
                    trajectoryConfig);
                
                SwerveControllerCommand blueSourceSideCenterNote1DriveCommand = new SwerveControllerCommand(
                    blueSourceSideCenterNote1Drive,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand blueSourceSideCenterNote1ReturnCommand = new SwerveControllerCommand(
                    blueSourceSideCenterNote1Return,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);


                SwerveControllerCommand blueSourceSideCenterNote2DriveCommand = new SwerveControllerCommand(
                    blueSourceSideCenterNote2Drive,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand blueSourceSideCenterNote2ReturnCommand = new SwerveControllerCommand(
                    blueSourceSideCenterNote2Return,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand blueSourceSideCenterNote3DriveCommand = new SwerveControllerCommand(
                    blueSourceSideCenterNote3Drive,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand blueSourceSideCenterNote3ReturnCommand = new SwerveControllerCommand(
                    blueSourceSideCenterNote3Return,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);
                
                SwerveControllerCommand blueSourceSideTESTCommand = new SwerveControllerCommand(
                    blueSourceSideTEST,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

            return new SequentialCommandGroup(
                new InstantCommand(() -> sped = 86),
                new InstantCommand(() -> angle = -6.7),
                new InstantCommand(()-> RobotContainer.s_Swerve.resetOdometry(blueSourceSideCenterNote1Drive.getInitialPose())),
                //blueSourceSideTESTCommand
                new InstantCommand(()-> SuperStructure.subShot()),
                new InstantCommand(()-> Timer.delay(0.25)),
                new InstantCommand(()-> SuperStructure.releaseShot()),
                new InstantCommand(()-> Timer.delay(0.15)),
                new InstantCommand(()-> SuperStructure.goHome()),
                new InstantCommand(()-> Timer.delay(0.3)), //added .05
                new InstantCommand(()-> SuperStructure.intake()),

                /* new InstantCommand(()-> SuperStructure.subShot()),
                new InstantCommand(()-> Timer.delay(0.3)),   
                new InstantCommand(()-> SuperStructure.releaseShot()),
                new InstantCommand(()-> Timer.delay(0.15)),
                new InstantCommand(()-> SuperStructure.goHome()),
                new InstantCommand(()-> Timer.delay(0.25)), //added .05
                new InstantCommand(()-> SuperStructure.intake()),
                new InstantCommand(()-> Timer.delay(0.25)), */

                blueSourceSideCenterNote1DriveCommand,  
                new InstantCommand(()-> SuperStructure.subShot()),              
                blueSourceSideCenterNote1ReturnCommand,
                new InstantCommand(()-> SuperStructure.releaseShot()),
                new InstantCommand(()-> Timer.delay(0.15)),
                new InstantCommand(()-> SuperStructure.goHome()),
                new InstantCommand(()-> Timer.delay(0.3)),
                new InstantCommand(()-> SuperStructure.intake()),
                blueSourceSideCenterNote2DriveCommand,
                new InstantCommand(()-> SuperStructure.subShot()),
                blueSourceSideCenterNote2ReturnCommand,
                new InstantCommand(()-> SuperStructure.releaseShot()),
                new InstantCommand(()-> Timer.delay(0.15)),
                new InstantCommand(()-> SuperStructure.goHome())
            );

            case 7: // Red Source Side
                Trajectory redSourceSideCenterNote1Drive = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(15.63, 4.26, Rotation2d.fromDegrees(180+60)),
                    List.of(
                        new Translation2d(13.2, 2)),
                    new Pose2d(8.88 - .8, 0.5, Rotation2d.fromDegrees(180)),
                    trajectoryConfigSlow);

                Trajectory redSourceSideCenterNote1Return = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(8.88 - .8, 0.62, Rotation2d.fromDegrees(180)),
                    List.of(
                        new Translation2d(12.5, 2)),
                    new Pose2d(13.32,3, Rotation2d.fromDegrees(180 + 42)),
                    trajectoryConfigSlow);
                    
                Trajectory redSourceSideCenterNote2Drive = TrajectoryGenerator.generateTrajectory(//* */
                    new Pose2d(13.32,3., Rotation2d.fromDegrees(180 + 42)),
                    List.of(
                        new Translation2d(13,1.),
                        new Translation2d(10.5,2.),//10.5,1 Time 12:19 TT
                        new Translation2d(9.26,2.6)),
                    new Pose2d(8.2-.2,2.5, Rotation2d.fromDegrees(180-180)), // TT 10-20 go further (-.2) and swing around anti-Tide
                    trajectoryConfigSlow);

                Trajectory redSourceSideCenterNote2Return = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(8.2,2.5, Rotation2d.fromDegrees(180)),
                    List.of(
                        new Translation2d(9.5,2),
                        new Translation2d(13,2)),
                    new Pose2d(12.82,3., Rotation2d.fromDegrees(180+42)),
                    trajectoryConfigSlow);

                Trajectory redSourceSideCenterNote3Drive = TrajectoryGenerator.generateTrajectory(new Pose2d(15.5, 4.5, Rotation2d.fromDegrees(180+60.0-28.6)),
                    List.of(
                        new Translation2d(14.5, 3),
                        new Translation2d(12.5 ,3),
                        new Translation2d(10.5 ,4.1),
                        new Translation2d(9.6 ,3.25)),
                    new Pose2d(9.2, 4.1, Rotation2d.fromDegrees(180-30)),
                    trajectoryConfigSlow);

                Trajectory redSourceSideCenterNote3Return = TrajectoryGenerator.generateTrajectory(new Pose2d(9.2, 4.1, Rotation2d.fromDegrees(180-30)),
                    List.of(
                        new Translation2d(10.5, 4.1),
                        new Translation2d(12.5, 3),
                        new Translation2d(14.5, 3)),
                    new Pose2d(15.5, 4.5, Rotation2d.fromDegrees(180+60-28.6)),
                    trajectoryConfig);
                
                SwerveControllerCommand redSourceSideCenterNote1DriveCommand = new SwerveControllerCommand(
                    redSourceSideCenterNote1Drive,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand redSourceSideCenterNote1ReturnCommand = new SwerveControllerCommand(
                    redSourceSideCenterNote1Return,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);


                SwerveControllerCommand redSourceSideCenterNote2DriveCommand = new SwerveControllerCommand(
                    redSourceSideCenterNote2Drive,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand redSourceSideCenterNote2ReturnCommand = new SwerveControllerCommand(
                    redSourceSideCenterNote2Return,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand redSourceSideCenterNote3DriveCommand = new SwerveControllerCommand(
                    redSourceSideCenterNote3Drive,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand redSourceSideCenterNote3ReturnCommand = new SwerveControllerCommand(
                    redSourceSideCenterNote3Return,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

            return new SequentialCommandGroup(
                new InstantCommand(() -> sped = 86),
                new InstantCommand(() -> angle = -5.75),
                new InstantCommand(()-> RobotContainer.s_Swerve.resetOdometry(redSourceSideCenterNote1Drive.getInitialPose())),
                new InstantCommand(()-> SuperStructure.subShot()),
                new InstantCommand(()-> Timer.delay(0.25)),
                new InstantCommand(()-> SuperStructure.releaseShot()),
                new InstantCommand(()-> Timer.delay(0.15)),
                new InstantCommand(()-> SuperStructure.goHome()),
                /*new InstantCommand(()-> Timer.delay(0.3)), //added .05
                new InstantCommand(()-> SuperStructure.intake()),

                redSourceSideCenterNote1DriveCommand,             
                redSourceSideCenterNote1ReturnCommand,

                new InstantCommand(()-> SuperStructure.releaseShot()),
                new InstantCommand(()-> Timer.delay(0.15)),
                new InstantCommand(()-> SuperStructure.goHome()),
                new InstantCommand(()-> Timer.delay(0.3)),
                new InstantCommand(()-> SuperStructure.intake()),*/ // 
                redSourceSideCenterNote2DriveCommand/*,
                redSourceSideCenterNote2ReturnCommand,
                new InstantCommand(()-> SuperStructure.releaseShot()),
                new InstantCommand(()-> Timer.delay(0.15)),
                new InstantCommand(()-> SuperStructure.goHome())*/
            );

        case 9: //5 piece Red middle start
                Trajectory iR15 = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(597), Units.inchesToMeters(225), Rotation2d.fromDegrees(180)),
                    List.of(
                        new Translation2d(Units.inchesToMeters(595-5), Units.inchesToMeters(220))),
                    new Pose2d(Units.inchesToMeters(537+12), Units.inchesToMeters(177-5), Rotation2d.fromDegrees(210)),
                    trajectoryConfigSlow);

                Trajectory iR25 = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(537+12), Units.inchesToMeters(177), Rotation2d.fromDegrees(210)),
                    List.of(
                        new Translation2d(Units.inchesToMeters(570+3), Units.inchesToMeters(200)),
                        new Translation2d(Units.inchesToMeters(570+3), Units.inchesToMeters(225)),
                        new Translation2d(Units.inchesToMeters(537+12), Units.inchesToMeters(225))), //adjusted -.2 after pmatch 10
                    new Pose2d(Units.inchesToMeters(550), Units.inchesToMeters(225), Rotation2d.fromDegrees(180)),
                    trajectoryConfigSlow);

                Trajectory iR35 = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(550), Units.inchesToMeters(225), Rotation2d.fromDegrees(180.0)),
                    List.of(
                        new Translation2d(Units.inchesToMeters(555), Units.inchesToMeters(250)),
                        new Translation2d(Units.inchesToMeters(552), Units.inchesToMeters(267))),
                    new Pose2d(Units.inchesToMeters(547), Units.inchesToMeters(268), Rotation2d.fromDegrees(150)),
                    trajectoryConfigSlow);

                    //Previous iR35 below
                /*Trajectory iR35 = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(550), Units.inchesToMeters(225), Rotation2d.fromDegrees(180.0)),
                    List.of(
                        new Translation2d(Units.inchesToMeters(580), Units.inchesToMeters(265)),
                        new Translation2d(Units.inchesToMeters(537), Units.inchesToMeters(265))),
                    new Pose2d(Units.inchesToMeters(537), Units.inchesToMeters(225), Rotation2d.fromDegrees(180)),
                    trajectoryConfigSlow);

                Trajectory iR45 = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(597.043500), Units.inchesToMeters(215), Rotation2d.fromDegrees(180.0)),
                    List.of(
                        new Translation2d(Units.inchesToMeters(580), Units.inchesToMeters(265)),
                        new Translation2d(Units.inchesToMeters(537.222500), Units.inchesToMeters(265))),
                    new Pose2d(Units.inchesToMeters(597.043500), Units.inchesToMeters(225), Rotation2d.fromDegrees(180)),
                    trajectoryConfig);*/
                    
                Trajectory iR45 = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(547), Units.inchesToMeters(268), Rotation2d.fromDegrees(150)),
                    List.of(
                        new Translation2d(12.5, 5.25),
                        new Translation2d(10.75, 4.0),
                        new Translation2d(9.2,  4.15),//noteGrab
                        new Translation2d(9.5,  4.15),
                        new Translation2d(10.75, 4.0),
                        new Translation2d(12.5, 5.25)),
                    new Pose2d(Units.inchesToMeters(550), Units.inchesToMeters(225), Rotation2d.fromDegrees(180)),
                    trajectoryConfigTravel);

                //Trajectory iR12 = iR1.concatenate(sR1);
                SwerveControllerCommand iR1Command5 = new SwerveControllerCommand(
                    iR15,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand iR2Command5 = new SwerveControllerCommand(
                    iR25,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

                SwerveControllerCommand iR3Command5 = new SwerveControllerCommand(
                    iR35,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);
                    
                SwerveControllerCommand iR4Command5 = new SwerveControllerCommand(
                    iR45,
                    RobotContainer.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    xController,
                    yController,
                    thetaController,
                    RobotContainer.s_Swerve::setModuleStates,
                    RobotContainer.s_Swerve);

            return new SequentialCommandGroup(
                new InstantCommand(()-> RobotContainer.s_Swerve.resetOdometry(iR15.getInitialPose())),
                new InstantCommand(()-> SuperStructure.subShot()),
                new InstantCommand(()-> Timer.delay(0.25)),
                new InstantCommand(()-> SuperStructure.releaseShot()),
                new InstantCommand(()-> Timer.delay(0.15)),
                new InstantCommand(()-> SuperStructure.goHome()),
                new InstantCommand(()-> Timer.delay(0.3)), //added .05
                new InstantCommand(()-> SuperStructure.intake()),
                new InstantCommand(() -> sped = 78),
                new InstantCommand(() -> angle = -4),
                iR1Command5,
                new InstantCommand(()-> Timer.delay(0.5)),
                new InstantCommand(()-> SuperStructure.releaseShot()),
                new InstantCommand(()-> Timer.delay(0.15)),
                new InstantCommand(()-> SuperStructure.goHome()),
                new InstantCommand(()-> Timer.delay(0.3)),
                new InstantCommand(()-> SuperStructure.intake()),
                new InstantCommand(() -> sped = 63),
                new InstantCommand(() -> angle = 0-1),
                iR2Command5,
                new InstantCommand(()-> Timer.delay(0.3)),
                new InstantCommand(()-> SuperStructure.releaseShot()),
                new InstantCommand(()-> Timer.delay(0.15)),
                new InstantCommand(()-> SuperStructure.goHome()),
                new InstantCommand(()-> Timer.delay(0.3)),
                new InstantCommand(() -> sped = 80),
                new InstantCommand(() -> angle = -4.25),
                new InstantCommand(()-> SuperStructure.intake()),
                iR3Command5,
                new InstantCommand(()-> Timer.delay(0.5)),
                new InstantCommand(()-> SuperStructure.releaseShot()),
                new InstantCommand(()-> Timer.delay(0.15)),
                new InstantCommand(()-> SuperStructure.goHome()),
                new InstantCommand(()-> Timer.delay(0.3)),
                new InstantCommand(()-> SuperStructure.intake()),
                iR4Command5,
                new InstantCommand(()-> SuperStructure.releaseShot()),
                new InstantCommand(()-> Timer.delay(0.15)),
                new InstantCommand(()-> SuperStructure.goHome())
            );

        }

        xController.close();
        yController.close();
        return null;
    }

}
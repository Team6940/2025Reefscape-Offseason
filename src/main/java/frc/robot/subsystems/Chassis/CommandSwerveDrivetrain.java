package frc.robot.subsystems.Chassis;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
// import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Controller.ImprovedCommandXboxController;
// import frc.robot.subsystems.Vision.ImprovedLL;
import frc.robot.subsystems.Vision.LimelightHelpers;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.GeneralConstants.*;
import frc.robot.constants.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.containers.RobotContainer;
import frc.robot.library.MUtils;
import frc.robot.library.MUtils.SegmentOnTheField;
import frc.robot.library.team1706.MathUtils;
import frc.robot.library.team5516.utils.simulation.MapleSimSwerveDrivetrain;;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static CommandSwerveDrivetrain m_Instance;

    public static CommandSwerveDrivetrain getInstance() {
        return m_Instance == null ? m_Instance = TunerConstants.createDrivetrain() : m_Instance;
    }

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity);

    /** Swerve request to apply during field-centric move to Pose */
    private static final SwerveRequest.ApplyFieldSpeeds m_moveToPoseDrive = new SwerveRequest.ApplyFieldSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity);
    PIDController m_translationXController, m_translationYController, m_rotationController;
    Pose2d m_targetPose2d = new Pose2d();

    /** Swerve request to apply during Manual drivemode */
    private static double manual_MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts
                                                                                                // desired top speed
    private static double manual_MaxAngularRate = RotationsPerSecond.of(1.).in(RadiansPerSecond); // 3/4 of a rotation
                                                                                                  // per second max
                                                                                                  // angular velocity
    private static final SwerveRequest.FieldCentric m_manualDrive = new SwerveRequest.FieldCentric().withDeadband(0.15)
            .withRotationalDeadband(0.05 * manual_MaxAngularRate).withDriveRequestType(DriveRequestType.Velocity);

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    // null, // Use default ramp rate (1 V/s)
                    Volts.of(2).per(Second), // Use Faster ramprate
                    Volts.of(6), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(
                drivetrainConstants,
                odometryUpdateFrequency,
                MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules) {
    super(
            drivetrainConstants,
            odometryUpdateFrequency,
            odometryStandardDeviation,
            visionStandardDeviation,
            MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
    if (Utils.isSimulation()) {
        startSimThread();
    }
    configureAutoBuilder();
}

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> getState().Pose, // Supplier of current robot pose
                    this::resetPose, // Consumer for seeding pose against auto
                    () -> getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds.withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(AutoConstants.followPathTranslationkP,
                                    AutoConstants.followPathTranslationkI, AutoConstants.followPathTranslationkD),
                            // PID constants for rotation
                            new PIDConstants(AutoConstants.followPathRotationkP, AutoConstants.followPathRotationkI,
                                    AutoConstants.followPathRotationkD)),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the
                    // case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    private void configureMoveToPose() {
        m_translationXController = new PIDController(AutoConstants.moveToPoseTranslationkP,
                AutoConstants.moveToPoseTranslationkI, AutoConstants.moveToPoseTranslationkD);
        m_translationYController = new PIDController(AutoConstants.moveToPoseTranslationkP,
                AutoConstants.moveToPoseTranslationkI, AutoConstants.moveToPoseTranslationkD);
        m_rotationController = new PIDController(AutoConstants.moveToPoseRotationkP, AutoConstants.moveToPoseRotationkI,
                AutoConstants.moveToPoseRotationkD);
        m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
        m_rotationController.setTolerance(AutoConstants.moveToPoseRotationToleranceRadians);
        m_translationXController.setTolerance(AutoConstants.moveToPoseTranslationToleranceMeters);
        m_translationYController.setTolerance(AutoConstants.moveToPoseTranslationToleranceMeters);
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }
        Logger.recordOutput("Chassis/RealPose", getPose());
        updateOdometry();
    }

    private MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;
    @SuppressWarnings("unchecked")
    private void startSimThread() {
        mapleSimSwerveDrivetrain = new MapleSimSwerveDrivetrain(
            Seconds.of(kSimLoopPeriod),
            Pounds.of(115), // robot weight
            Inches.of(30), // bumper length
            Inches.of(30), // bumper width
            DCMotor.getKrakenX60(1), // drive motor type
            DCMotor.getKrakenX60(1), // steer motor type
            1.2, // wheel COF
            getModuleLocations(),
            getPigeon2(),
            getModules(),
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);
    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
    m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void resetPose(Pose2d pose) {
        if (this.mapleSimSwerveDrivetrain != null)
            mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
        Timer.delay(0.05); // Wait for simulation to update
        super.resetPose(pose);
    }
    
    public Pose2d getPose() {
        return this.getState().Pose;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return this.getState().Speeds;
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), getPose().getRotation());
    }

    public void driveFieldCentric(ChassisSpeeds speeds) {
        driveFieldCentric(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    public void driveFieldCentric(double vx, double vy, double omegaRadsPerSec) {
        this.setControl(m_manualDrive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omegaRadsPerSec));
    }

    public void driveFieldCentric(ImprovedCommandXboxController controller, double power) {
        driveFieldCentric(
                -MathUtils.signedPow(controller.getLeftY(), power) * manual_MaxSpeed,
                -MathUtils.signedPow(controller.getLeftX(), power) * manual_MaxSpeed,
                -controller.getRightX() * manual_MaxAngularRate);
    }

    public void driveFieldCentric(ImprovedCommandXboxController controller, double power, double maxSpeed) {
        driveFieldCentric(
                -MathUtils.signedPow(controller.getLeftY(), power) * maxSpeed,
                -MathUtils.signedPow(controller.getLeftX(), power) * maxSpeed,
                -controller.getRightX() * manual_MaxAngularRate);
    }

    public void updateOdometry() {
        LimelightHelpers.SetRobotOrientation(RobotContainer.m_Limelight, getPose().getRotation().getDegrees(), 0, 0, 0,
                0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2(RobotContainer.m_Limelight);
        if (mt2 == null) {
            DriverStation.reportWarning(RobotContainer.m_Limelight + " Diconnected!", false);
            return;
        }

        if (Math.abs(getRobotRelativeSpeeds().omegaRadiansPerSecond) <= 4 * Math.PI
                && mt2.tagCount > 0 // ll fetch over one target
                && mt2.avgTagDist < 4 // within 4 meters
                && Math.hypot(getRobotRelativeSpeeds().vxMetersPerSecond,
                        getRobotRelativeSpeeds().vyMetersPerSecond) < 2) {// not moving too fast
            // Use vision measurement
            addVisionMeasurement(mt2.pose,
                    Utils.fpgaToCurrentTime(mt2.timestampSeconds),
                    VecBuilder.fill(PoseEstimatorConstants.tAtoDev.get(mt2.avgTagArea),
                            PoseEstimatorConstants.tAtoDev.get(mt2.avgTagArea), 100000000)
            );// with very high rotation certainty

            // Logger.recordOutput("Chassis/VisionPose", mt2.pose);

            SmartDashboard.putNumber("tA",
                                        mt2.avgTagArea);
            SmartDashboard.putNumber("Dev",
                    PoseEstimatorConstants.tAtoDev.get(mt2.avgTagArea));
        }

    }

    public boolean isAtTranslation(Translation2d t) {
        return getPose().getTranslation().minus(t).getNorm() <= AutoConstants.moveToPoseTranslationToleranceMeters;
    }

    public boolean isAtRotation(Rotation2d r) {
        return Math
                .abs(getPose().getRotation().minus(r).getRadians()) <= AutoConstants.moveToPoseRotationToleranceRadians;
    }

    public boolean isAtPose(Pose2d pose) {
        return isAtTranslation(pose.getTranslation()) && isAtRotation(pose.getRotation());
    }

    public boolean isAtTargetPose() {
        return isAtPose(m_targetPose2d);
    }

    /** Caution this would reset your targetPose to this pose */
    public ChassisSpeeds getAutoMoveToPoseSpeeds(Pose2d pose) {
        m_targetPose2d = pose;
        Pose2d currentPose = getPose();
        m_rotationController.setSetpoint(pose.getRotation().getRadians());
        m_translationXController.setSetpoint(pose.getX());
        m_translationYController.setSetpoint(pose.getY());

        double targetAngularVelocity = m_rotationController.calculate(currentPose.getRotation().getRadians());
        double targetTranslationXVelocity = m_translationXController.calculate(currentPose.getX());
        double targetTranslationYVelocity = m_translationYController.calculate(currentPose.getY());

        return optimizeMoveToSpeeds(optimizeMoveToSpeeds(
                new ChassisSpeeds(targetTranslationXVelocity, targetTranslationYVelocity, targetAngularVelocity)));
    }

    public Command autoMoveToPoseCommand(Pose2d pose) {
        return this.applyRequest(() -> m_moveToPoseDrive.withSpeeds(getAutoMoveToPoseSpeeds(pose)));
    }

    public void autoMoveToPose(Pose2d pose) {

        this.setControl(m_moveToPoseDrive.withSpeeds(getAutoMoveToPoseSpeeds(pose)));
    }

    private ChassisSpeeds optimizeMoveToSpeeds(ChassisSpeeds speeds) {
        ChassisSpeeds optimizedSpd = speeds;

        // If too small, apply deadband;
        // optimizedSpd.vxMetersPerSecond =
        // MathUtil.applyDeadband(optimizedSpd.vxMetersPerSecond,
        // AutoConstants.moveToVelocityDeadband, 1e6);
        // optimizedSpd.vyMetersPerSecond =
        // MathUtil.applyDeadband(optimizedSpd.vyMetersPerSecond,
        // AutoConstants.moveToVelocityDeadband, 1e6);
        // optimizedSpd.omegaRadiansPerSecond =
        // MathUtil.applyDeadband(optimizedSpd.omegaRadiansPerSecond,
        // AutoConstants.moveToAnguVeloDeadband, 1e6);

        // if too big, apply limit;
        if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) > AutoConstants.maxMoveToSpeed) {
            optimizedSpd = optimizedSpd.times(
                    AutoConstants.maxMoveToSpeed / Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
        }
        if (Math.abs(speeds.omegaRadiansPerSecond) > AutoConstants.maxMoveToAngularVelocity) {
            optimizedSpd.omegaRadiansPerSecond = Math.copySign(AutoConstants.maxMoveToAngularVelocity,
                    speeds.omegaRadiansPerSecond);
        }
        return optimizedSpd;
    }

    public double getToPoseDistance(Pose2d pose) {
        return getPose().getTranslation().getDistance(pose.getTranslation());
    }

    public double getToReefCenterDistance() {
        return getFromReefCentreTranslation().getNorm();
    }

    public double getHybridMoveToposeDistance() {
        // return
        // Math.sqrt(m_translationXController.getError()*m_translationXController.getError()+m_translationYController.getError()*m_translationYController.getError());
        return getToPoseDistance(m_targetPose2d);
    }

    /** Caution this would reset your targetPose to this pose */
    public ChassisSpeeds getHybridMoveToPoseSpeeds(Pose2d pose, ImprovedCommandXboxController driverController,
            double translationAdjustmentRange, double rotationAdjustmentRangeDegs) {
        Pose2d currentPose = getPose();

        // Hybrid Control, read the lines carefully before you tune it
        Transform2d adjustment = new Transform2d(
                driverController.getLeftY() * translationAdjustmentRange
                        * (DriverStation.getAlliance().get() == Alliance.Blue ? -1. : +1.),
                driverController.getLeftX() * translationAdjustmentRange
                        * (DriverStation.getAlliance().get() == Alliance.Blue ? -1. : +1.),
                Rotation2d.fromDegrees(-driverController.getRightX() * rotationAdjustmentRangeDegs));

        pose = MUtils.transformFieldRelative(pose, adjustment);
        m_targetPose2d = pose;

        m_rotationController.setSetpoint(pose.getRotation().getRadians());
        m_translationXController.setSetpoint(pose.getX());
        m_translationYController.setSetpoint(pose.getY());

        double targetAngularVelocity = m_rotationController.calculate(currentPose.getRotation().getRadians());
        double targetTranslationXVelocity = m_translationXController.calculate(currentPose.getX());
        double targetTranslationYVelocity = m_translationYController.calculate(currentPose.getY());

        if (isAtTranslation(pose.getTranslation())) {
            targetTranslationXVelocity = 0.;
            targetTranslationYVelocity = 0.;
        }
        if (isAtRotation(pose.getRotation())) {
            targetAngularVelocity = 0.;
        }

        SmartDashboard.putNumber("targetXVeloUnoptimized", targetTranslationXVelocity);
        SmartDashboard.putNumber("targetYVeloUnoptimized", targetTranslationYVelocity);
        SmartDashboard.putNumber("targetAngularVeloUnoptimized", targetAngularVelocity);

        return optimizeMoveToSpeeds(
                new ChassisSpeeds(targetTranslationXVelocity, targetTranslationYVelocity, targetAngularVelocity));
    }

    public Command hybridMoveToPoseCommand(Supplier<Pose2d> pose, ImprovedCommandXboxController driverController,
            double translationAdjustmentRange, double rotationAdjustmentRangeDegs) { // TODO
        return this.applyRequest(() -> m_moveToPoseDrive.withSpeeds(getHybridMoveToPoseSpeeds(pose.get(),
                driverController, translationAdjustmentRange, rotationAdjustmentRangeDegs)));
    }

    public void hybridMoveToPose(Pose2d pose, ImprovedCommandXboxController driverController,
            double translationAdjustmentRange, double rotationAdjustmentRangeDegs) {
        this.setControl(m_moveToPoseDrive.withSpeeds(getHybridMoveToPoseSpeeds(pose, driverController,
                translationAdjustmentRange, rotationAdjustmentRangeDegs)));
    }

    public PathPlannerPath generateChoreoPath(String pathName) {
        try {
            return PathPlannerPath.fromChoreoTrajectory(pathName);
        } catch (Exception e) {
            e.printStackTrace();
        }
        return null;
    }

    public PathPlannerPath generatePPPath(String pathName) {
        try {
            return PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public Command followChoreoPath(String pathName) {
        return AutoBuilder.followPath(generateChoreoPath(pathName));
    }

    public Command followPPPath(String pathName) {
        return AutoBuilder.followPath(generatePPPath(pathName));
    }

    public Pose2d generateReefPose(int index, int level) {
        Translation2d t = FieldConstants.BlueReefCenterPos;
        Translation2d dt = FieldConstants.DReefTranslation12;

        if (level <= 2) {
            dt = new Translation2d(dt.getX() + FieldConstants.L2Fix, dt.getY());
        }
        if (index % 2 == 1) {
            dt = new Translation2d(dt.getX(), -dt.getY());
        }
        t = t.plus(dt);

        Rotation2d r = new Rotation2d(Math.PI);
        Rotation2d dr = Rotation2d.fromDegrees(
                (double) ((index + 1) / 2) * 60.);

        t = t.rotateAround(FieldConstants.BlueReefCenterPos, dr);
        r = r.plus(dr);

        Pose2d res = new Pose2d(t, r);

        if (DriverStation.getAlliance().get() == Alliance.Red) {
            // t = t.rotateAround(FieldConstants.FieldCenter, Rotation2d.fromDegrees(180));
            // r = r.plus(Rotation2d.fromDegrees(180));
            res = FieldConstants.rotateAroundCenter(res, FieldConstants.FieldCenter, Rotation2d.k180deg);
        }

        if (res == null) {
            DriverStation.reportWarning("1111111111111111111", false);
        }

        return res;
    }

    public Pose2d generateReefPoseReversed(int index, int Level) {
        Translation2d t = FieldConstants.BlueReefCenterPos;
        Translation2d dt = FieldConstants.DReefTranslation12Reversed;
        if (Level == 2) {
            dt = new Translation2d(dt.getX() + FieldConstants.L2Fix, dt.getY());
        }
        if (index % 2 == 1) {
            dt = new Translation2d(dt.getX(), -dt.getY()+0.05);
        }
        t = t.plus(dt);

        Rotation2d r = new Rotation2d(0);// changed from PI to 0 here
        Rotation2d dr = Rotation2d.fromDegrees(
                (double) ((index + 1) / 2) * 60.);

        t = t.rotateAround(FieldConstants.BlueReefCenterPos, dr);
        r = r.plus(dr);

        Pose2d res = new Pose2d(t, r);

        if (DriverStation.getAlliance().get() == Alliance.Red) {
            // t = t.rotateAround(FieldConstants.FieldCenter, Rotation2d.fromDegrees(180));
            // r = r.plus(Rotation2d.fromDegrees(180));
            res = FieldConstants.rotateAroundCenter(res, FieldConstants.FieldCenter, Rotation2d.k180deg);
        }

        if (res == null) {
            DriverStation.reportWarning("1111111111111111111", false);
        }

        return res;
    }

    public Pose2d generateAlgaeIntakePose(int index) {
        Translation2d t = FieldConstants.BlueReefCenterPos;
        Translation2d dt = FieldConstants.DAlgaeTranslation6;
        t = t.plus(dt);// translation here

        Rotation2d r = new Rotation2d(0);
        Rotation2d dr = Rotation2d.fromDegrees(
                (double) ((index + 2) % 6) * 60.// TODO here
        );

        t = t.rotateAround(FieldConstants.BlueReefCenterPos, dr);
        r = r.plus(dr);// constructing the final pose

        Pose2d res = new Pose2d(t, r);

        if (DriverStation.getAlliance().get() == Alliance.Red) {
            res = FieldConstants.rotateAroundCenter(res, FieldConstants.FieldCenter, Rotation2d.k180deg);
        } // position shifts based on red&blue

        return res;
    }

    public Pose2d generateAlgaeScorePose() {
        Pose2d currentPose = getPose();
        Translation2d t = new Translation2d(FieldConstants.AlgaeScoreTransalationX, currentPose.getY());
        Pose2d res = new Pose2d(t, new Rotation2d(Math.PI));// todo, decide which direction the robot is facing.
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            res = FieldConstants.rotateAroundCenter(res, new Translation2d(FieldConstants.FieldCenter.getX(), t.getY()),
                    Rotation2d.k180deg);
        }
        return res;
    }

    public Translation2d getFromReefCentreTranslation() {
        // Translation2d t =
        // FieldConstants.BlueReefCenterPos.minus(getPose().getTranslation());
        // if(DriverStation.getAlliance().get() == Alliance.Red){
        // t.rotateBy(new Rotation2d(Math.PI));
        // }
        // return t;

        return DriverStation.getAlliance().get() == Alliance.Blue ?
        // FieldConstants.BlueReefCenterPos.minus(getPose().getTranslation()) :
                getPose().getTranslation().minus(FieldConstants.BlueReefCenterPos)
                : FieldConstants.BlueReefCenterPos.rotateAround(FieldConstants.FieldCenter, Rotation2d.k180deg)
                        .minus(getPose().getTranslation());
    }

    public boolean isFacingReefCenter() {
        Translation2d t = getFromReefCentreTranslation();
        double positionDegrees = t.getAngle().getDegrees();
        double facingDegrees = getPose().getRotation().getDegrees();
        if (Math.abs(positionDegrees - facingDegrees) <= 90.) {
            return true;// TODO here
        } else {
            return false;
        }
    }

    /**
     * Automatically generates a pose for station based on the current pose.
     * <p>
     * Only processes blue originated inputs! If you're on Red, rotate the input
     * around the center.
     * <p>
     * The algorithm is as the following:
     * <p>
     * 1. Define 2 segments in the field representing the Left and Right Station,
     * naming L and R.
     * <p>
     * 2. Determine which segment on which we should land, based on the current
     * pose.
     * <p>
     * 3. Finds the nearest point on the segment to the current pose, and generates
     * a pose that is at that point.
     * <p>
     * - If the projection of the current pose onto the segment is on the segment,
     * then the pose is at that point.
     * <p>
     * - If the projection of the current pose onto the segment is outside the
     * segment, then the pose is at the closest point on the segment to the current
     * pose.
     * 
     * @param currentPose the current pose of the robot, blue origin
     * @return the
     */

    public Pose2d generateStationPose(Pose2d currentPose) {
        SegmentOnTheField targetSegment = currentPose.getY() >= FieldConstants.FieldCenter.getY()
                ? FieldConstants.BlueLeftStation
                : FieldConstants.BlueRghtStation;
        Translation2d t = targetSegment.getNearestPointTo(currentPose.getTranslation());
        Rotation2d r = Rotation2d.fromRadians(Math.atan(-1. / targetSegment.k));
        return new Pose2d(t, r);
    }

    /**
     * Only accepts blue originated inputs! If you're on Red, rotate the input
     * around the center.
     * For more information, see {@link #generateStationPose(Pose2d)}
     * 
     * @param currentPose
     * @param distance
     * @return
     */
    public Pose2d generateStationPose(Pose2d currentPose, double distance) {
        SegmentOnTheField targetSegment = currentPose.getY() >= FieldConstants.FieldCenter.getY()
                ? FieldConstants.BlueLeftStation
                : FieldConstants.BlueRghtStation;
        distance = MUtils.numberLimit(0, targetSegment.getLength(), distance);
        Translation2d t = targetSegment
                .calculate(targetSegment.m + distance / Math.sqrt(targetSegment.k * targetSegment.k + 1.));
        Rotation2d r = Rotation2d.fromRadians(Math.atan(-1. / targetSegment.k));
        return new Pose2d(t, r);
    }

    public Pose2d generateStationPose(double distance) {
        return generateStationPose(getPose(), distance);
    }

    public Pose2d generateStationPose() {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return FieldConstants.rotateAroundCenter(
                    generateStationPose(FieldConstants.rotateAroundCenter(getPose(), FieldConstants.FieldCenter,
                            Rotation2d.k180deg)),
                    FieldConstants.FieldCenter,
                    Rotation2d.k180deg);
        }
        return generateStationPose(getPose());
    }

    /**
     * Only accepts blue originated inputs! If you're on Red, use
     * {@link #generateAlignedStationPoseRed(Pose2d)} instead.
     * <p>
     * Generates a pose that is aligned with one of the chutes.
     * For more information, see {@link #generateStationPose(Pose2d)} and
     * {@link #generateStationPose(double)}
     * 
     * @param currentPose
     * @return target pose
     */
    public Pose2d generateAlignedStationPoseBlue(Pose2d currentPose) {
        Pose2d projection = generateStationPose(currentPose);
        int lowerIndex = (int) Math
                .floor((projection.getX() - FieldConstants.BlueFirstChuteTranslationX) / FieldConstants.DChuteX);
        int upperIndex = lowerIndex + 1;
        Translation2d t1, t2;
        if (projection.getY() >= FieldConstants.FieldCenter.getY()) {
            t1 = FieldConstants.BlueLeftStation
                    .calculate(lowerIndex * FieldConstants.DChuteX + FieldConstants.BlueFirstChuteTranslationX);
            t2 = FieldConstants.BlueLeftStation
                    .calculate(upperIndex * FieldConstants.DChuteX + FieldConstants.BlueFirstChuteTranslationX);
        } else {
            t1 = FieldConstants.BlueRghtStation
                    .calculate(lowerIndex * FieldConstants.DChuteX + FieldConstants.BlueFirstChuteTranslationX);
            t2 = FieldConstants.BlueRghtStation
                    .calculate(upperIndex * FieldConstants.DChuteX + FieldConstants.BlueFirstChuteTranslationX);
        }
        Translation2d t = t1.getDistance(projection.getTranslation()) < t2.getDistance(projection.getTranslation()) ? t1
                : t2;
        return new Pose2d(t, projection.getRotation()); // TODO
    }

    /**
     * Red version of {@link #generateAlignedStationPoseBlue(Pose2d)}
     * 
     * @param currentPose
     * @return target pose
     */
    public Pose2d generateAlignedStationPoseRed(Pose2d currentPose) {
        return FieldConstants.rotateAroundCenter(
                generateAlignedStationPoseBlue(
                        FieldConstants.rotateAroundCenter(getPose(), FieldConstants.FieldCenter, Rotation2d.k180deg)),
                FieldConstants.FieldCenter,
                Rotation2d.k180deg);
    }

    public Pose2d generateAlignedStationPose(Pose2d currentPose) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return generateAlignedStationPoseRed(currentPose);
        }
        return generateAlignedStationPoseBlue(currentPose);
    }

    public Pose2d generateAlignedStationPose() {
        return generateAlignedStationPose(getPose());
    }

    public int generateReefIndex() {
        Translation2d t = getFromReefCentreTranslation();
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            t.rotateBy(new Rotation2d(Math.PI));
        }
        double angle = t.getAngle().getDegrees();
        int res = (int) Math.floor(angle / 30.) + 11;
        return res % 12 + 1;
    }

    public int generateAlgaeIntakeIndex() {
        Translation2d t = getFromReefCentreTranslation();
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            t.rotateBy(new Rotation2d(Math.PI));
        }
        double angle = t.getAngle().getDegrees();
        int res = (int) Math.floor((angle -30) / 60.) + 5;
        return res % 6;
    }

    public enum AutonomousTargetType {
        BARGE, // Should not be called unless you're returning to barge at teammates' request.
        STATION,
        REEF,
    }

    /**
     * Checks if the robot is near the given translation.
     * 
     * @param t        in meters, blue origin
     * @param distance in meters
     * @return
     */
    public boolean isNearTranslation(Translation2d t, double distance) {
        return getPose().getTranslation().getDistance(t) <= distance;
    }

    public boolean isNearStation() {
        return generateStationPose().getTranslation()
                .getDistance(getPose().getTranslation()) <= FieldConstants.StationDetectionArea;
    }

    public void brake() {
        setControl(new SwerveRequest.SwerveDriveBrake());
    }

    /**
     * Returns the predicted pose of the robot at the given time.
     * 
     * @param time in seconds
     * @return the predicted pose
     */
    public Pose2d getPredictedPose(double time) {
        return getPose().exp(getRobotRelativeSpeeds().toTwist2d(time));
    }

}

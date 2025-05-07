package org.chillout1778.subsystems

import choreo.trajectory.SwerveSample
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.chillout1778.Robot
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import org.chillout1778.Constants
import kotlin.math.abs

object Swerve: SubsystemBase() {
    private val gyro = Pigeon2(Constants.CanIds.GYRO).apply {
        configurator.apply(
            Pigeon2Configuration()
        )
    }

    // Read yaw from the gyro (radians, counterclockwise from forward).
    var gyroAngle: Double
        get() = MathUtil.angleModulus(gyro.rotation2d.radians)
        set(desiredAngle) {
            gyro.setYaw(MathUtil.angleModulus(desiredAngle)/Math.PI*180)
        }

    // The order doesn't matter to us, but PathPlannerLib prefers FL, FR, BL, BR.
    private val modules = arrayOf(
        SwerveModule(
            name = "Front Left",
            driveMotorID = Constants.CanIds.SWERVE_FRONT_LEFT_DRIVE,
            turnMotorID = Constants.CanIds.SWERVE_FRONT_LEFT_TURN,
            canCoderID = Constants.CanIds.SWERVE_FRONT_LEFT_CANCODER,
            driveInverted = InvertedValue.CounterClockwise_Positive,
            turnInverted  = InvertedValue.Clockwise_Positive,
            encoderOffset = Constants.Swerve.FRONT_LEFT_ENCODER_OFFSET
        ),
        SwerveModule(
            name = "Front Right",
            driveMotorID = Constants.CanIds.SWERVE_FRONT_RIGHT_DRIVE,
            turnMotorID = Constants.CanIds.SWERVE_FRONT_RIGHT_TURN,
            canCoderID = Constants.CanIds.SWERVE_FRONT_RIGHT_CANCODER,
            driveInverted = InvertedValue.Clockwise_Positive,
            turnInverted  = InvertedValue.Clockwise_Positive,
            encoderOffset = Constants.Swerve.FRONT_RIGHT_ENCODER_OFFSET
        ),
        SwerveModule(
            name = "Back Left",
            driveMotorID = Constants.CanIds.SWERVE_BACK_LEFT_DRIVE,
            turnMotorID = Constants.CanIds.SWERVE_BACK_LEFT_TURN,
            canCoderID = Constants.CanIds.SWERVE_BACK_LEFT_CANCODER,
            driveInverted = InvertedValue.CounterClockwise_Positive,
            turnInverted  = InvertedValue.Clockwise_Positive,
            encoderOffset = Constants.Swerve.BACK_LEFT_ENCODER_OFFSET
        ),
        SwerveModule(
            name = "Back Right",
            driveMotorID = Constants.CanIds.SWERVE_BACK_RIGHT_DRIVE,
            turnMotorID = Constants.CanIds.SWERVE_BACK_RIGHT_TURN,
            canCoderID = Constants.CanIds.SWERVE_BACK_RIGHT_CANCODER,
            driveInverted = InvertedValue.Clockwise_Positive,
            turnInverted  = InvertedValue.Clockwise_Positive,
            encoderOffset = Constants.Swerve.BACK_RIGHT_ENCODER_OFFSET
        )
    )

    private val fieldEstimate = Field2d()
//    val goalField = Field2d().apply { name = "goalPose" }

    var isAligned = false

    init {
        for (module in modules) {
            Shuffleboard.getTab("Swerve").add(module.name, module)
        }
        Shuffleboard.getTab("Swerve").add("swerve actual object", this)
        Shuffleboard.getTab("Swerve").add("Swerve Estimated Pose Field", fieldEstimate)
//        Shuffleboard.getTab("Swerve").add("Goal Pose", goalField)
    }

    private val kinematics = SwerveDriveKinematics(
        Translation2d(Constants.Swerve.XY_DISTANCE, Constants.Swerve.XY_DISTANCE), // FL
        Translation2d(Constants.Swerve.XY_DISTANCE, -Constants.Swerve.XY_DISTANCE), // FR
        Translation2d(-Constants.Swerve.XY_DISTANCE, Constants.Swerve.XY_DISTANCE), // BL
        Translation2d(-Constants.Swerve.XY_DISTANCE, -Constants.Swerve.XY_DISTANCE), // BR
    )

    private val poseEstimator = SwerveDrivePoseEstimator(
        kinematics, // positions of modules
        Rotation2d(gyroAngle), // initial robot yaw (converted to Rotation2d)
        modulePositions, // initial "positions" (how far the wheels have moved and in what direction)
        Pose2d(0.0, 0.0, Rotation2d(gyroAngle)),
        VecBuilder.fill(0.1, 0.1, 0.1), // odometry
        VecBuilder.fill(.9, .9, 2.0) // vision
    )

    override fun periodic() {
        Vision.periodicAddMeasurements(poseEstimator)
        poseEstimator.update(
            Rotation2d(gyroAngle),
            modulePositions
        )
        fieldEstimate.robotPose = poseEstimator.estimatedPosition
    }

    fun withinTolerance(t: Translation2d) = estimatedPose.translation.getDistance(t) < Constants.Swerve.ALIGNMENT_TOLERANCE

    private val modulePositions get() = modules.map { it.position }.toTypedArray()
    val moduleStates get() = modules.map { it.state }.toTypedArray()

    fun driveFieldRelative(speeds: ChassisSpeeds) {
        driveRobotRelative(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                estimatedPose.rotation
            )
        )
    }

    fun driveRobotRelative(speeds: ChassisSpeeds) {
        val discreteSpeeds = ChassisSpeeds.discretize(speeds, Robot.period)
        val moduleStates = kinematics.toSwerveModuleStates(discreteSpeeds)
        for ((mod, state) in modules.zip(moduleStates)) {
            mod.driveState(state)
        }
        poseEstimator.update(Rotation2d(gyroAngle), modulePositions)
    }

    fun stop() {
        driveRobotRelative(ChassisSpeeds())
    }

    private val xController = PIDController(12.0, 0.0, 0.0)
    private val yController = PIDController(12.0, 0.0, 0.0)
    private val headingController = PIDController(5.0, 0.0, 0.0).apply {
            enableContinuousInput(-Math.PI, Math.PI) }

    fun followSample(sample: SwerveSample) {
        val pose = poseEstimator.estimatedPosition
//        goalField.robotPose = sample.pose

        val speeds = ChassisSpeeds(
            sample.vx + xController.calculate(pose.x, sample.x),
            sample.vy + yController.calculate(pose.y, sample.y),
            sample.omega + headingController.calculate(pose.rotation.radians, sample.heading)
        )
        driveFieldRelative(speeds)
    }
    fun followPose(goalPose: Pose2d) {
        val currentPose = poseEstimator.estimatedPosition
        val speeds = ChassisSpeeds(
            xController.calculate(currentPose.x, goalPose.x),
            yController.calculate(currentPose.y, goalPose.y),
            headingController.calculate(currentPose.rotation.radians, goalPose.rotation.radians)
        )
        driveFieldRelative(speeds)
    }

    var estimatedPose
        get() = poseEstimator.estimatedPosition
        set(p) { poseEstimator.resetPosition(Rotation2d(gyroAngle), modulePositions, p) }

    fun score(p: Pose2d): Double {
        val translation = p.translation.getDistance(estimatedPose.translation)
        val rotation = abs((p.rotation - estimatedPose.rotation).radians)
        return Constants.Swerve.ALIGN_TRANSLATION_WEIGHT * translation +
                Constants.Swerve.ALIGN_ANGLE_WEIGHT * rotation
    }

    var probablyScoredPoses = BooleanArray(24*4)
    fun markPoseScored() {
        val idx = closestFudgedScoringPose?.index ?: return
        probablyScoredPoses[idx*4 + Superstructure.inputs.wantedScoringLevel.index] = true
    }
    fun wasPoseScored(idx: Int): Boolean {
        return probablyScoredPoses[idx*4 + Superstructure.inputs.wantedScoringLevel.index]
    }

//    var fudgingMakesADifference = false
    val closestFudgedScoringPose: IndexedValue<Pose2d>? get() {
        return (if (Robot.isRedAlliance) Constants.Field.redScoringPoses else Constants.Field.blueScoringPoses)
            .withIndex()
            .filter { (_: Int, pose: Pose2d) ->
                pose.translation.getDistance(estimatedPose.translation) < Constants.Swerve.MAX_NODE_DISTANCE
            }.minByOrNull { (idx: Int, pose: Pose2d) ->
                val fudge: Double = if (wasPoseScored(idx)) Constants.Swerve.ALREADY_SCORED_BADNESS else 0.0
                val poseBadness = score(pose)
                fudge + poseBadness
            }
    }

//    val closestCoralScoringPose: Pose2d? get() {
//        var poses = if (Robot.isRedAlliance) Constants.Field.redScoringPoses else Constants.Field.blueScoringPoses
//        poses = poses.filter {
//            it.translation.getDistance(estimatedPose.translation) < Constants.Swerve.MIN_NODE_DISTANCE
//        }
//        return poses.minByOrNull { score(it) }
//    }

    val closestAlgaeGrabPose: Pose2d? get() {
        var poses = if (Robot.isRedAlliance) Constants.Field.redAlgaeGrabbingPose else Constants.Field.blueAlgaeGrabbingPose
        poses = poses.filter {
            it.translation.getDistance(estimatedPose.translation) < Constants.Swerve.MAX_NODE_DISTANCE
        }
        return poses.minByOrNull { score(it) }
    }

    val closestTroughScoringPose: Pose2d? get() {
        var poses = if (Robot.isRedAlliance) Constants.Field.redTroughScoringPoses else Constants.Field.blueTroughScoringPoses
        poses = poses.filter {
            it.translation.getDistance(estimatedPose.translation) < Constants.Swerve.MAX_NODE_DISTANCE
        }
        return poses.minByOrNull { score(it) }
    }

    val BARGE_SCORING_POSITION_TOLERANCE = .1
    val atGoodScoringDistance: Boolean get() {
        return if(!Robot.isOnRedSide)
            estimatedPose.x > Constants.Field.BLUE_BARGE_SCORING_X - BARGE_SCORING_POSITION_TOLERANCE
                && estimatedPose.x < Constants.Field.BLUE_BARGE_SCORING_X + BARGE_SCORING_POSITION_TOLERANCE
        else estimatedPose.x > Constants.Field.RED_BARGE_SCORING_X - BARGE_SCORING_POSITION_TOLERANCE
                && estimatedPose.x < Constants.Field
                    .RED_BARGE_SCORING_X + BARGE_SCORING_POSITION_TOLERANCE
    }

    override fun initSendable(builder: SendableBuilder?) {
        builder!!
        builder.addDoubleProperty("gyro angle", {Math.toDegrees(gyroAngle)}, {})
        builder.addDoubleProperty("vision angle", {estimatedPose.rotation.degrees}, {})
        builder.addBooleanProperty("is red?", {Robot.isRedAlliance}, {})
        builder.addDoubleProperty("raw (not wrapped) gyro yaw", {gyro.rotation2d.radians}, {})
        builder.addStringProperty("odometry pose", {poseEstimator.estimatedPosition.toString()}, {})
//        builder.addDoubleProperty("distance from starting pose", { stupidStartingTranslation.getDistance(estimatedPose.translation) }, {})
    }
}

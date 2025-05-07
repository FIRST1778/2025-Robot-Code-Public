package org.chillout1778

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.util.Units
import org.chillout1778.Utils.mirror
import kotlin.math.PI
import kotlin.math.sqrt


object Constants{
    object CanIds{
        //Don't put anything on ID 0, when a new motor is recognized in Phoenix Tuner X it causes issues
        const val SWERVE_FRONT_LEFT_DRIVE = 1
        const val SWERVE_FRONT_LEFT_TURN = 2
        const val SWERVE_FRONT_RIGHT_DRIVE = 3
        const val SWERVE_FRONT_RIGHT_TURN = 4
        const val SWERVE_BACK_RIGHT_DRIVE = 5
        const val SWERVE_BACK_RIGHT_TURN = 6
        const val SWERVE_BACK_LEFT_DRIVE = 7
        const val SWERVE_BACK_LEFT_TURN = 8
        const val SWERVE_FRONT_LEFT_CANCODER = 9
        const val SWERVE_FRONT_RIGHT_CANCODER = 10
        const val SWERVE_BACK_LEFT_CANCODER = 11
        const val SWERVE_BACK_RIGHT_CANCODER = 12

        const val ELEVATOR_MAIN_MOTOR = 13
        const val ELEVATOR_FOLLOWER_MOTOR = 14

        const val ARM_PIVOT_MOTOR = 15
        const val ARM_ROLLER_MOTOR = 16
        const val ARM_ENCODER = 20

        const val INTAKE_PIVOT_MOTOR = 17
        const val INTAKE_ROLLER_MOTOR = 18
        const val INTAKE_CENTERING_MOTOR = 19

        const val GYRO = 30
    }
    object DioIds{
        const val ARM_ABSOLUTE_ENCODER = 0
        const val INTAKE_LINEBREAK = 1
        const val DISABLE_BREAK_MODE = 2
    }
    object Vision{
        val FIELD_TYPE = AprilTagFields.k2025ReefscapeWelded

        const val FRONT_RIGHT_NAME = "Front Right"
        const val FRONT_LEFT_NAME = "Front Left"
        const val BACK_RIGHT_NAME = "Back Right"
        const val BACK_LEFT_NAME = "Back Left"
        // positive x is to the front (intake side), positive y is to the left, and counterclockwise (when looking down at the robot) is positive theta
        val FRONT_RIGHT_TRANSFORM = Transform3d(Translation3d(-0.012552, -0.319809, 0.191168), Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(-70.0)))
        val BACK_RIGHT_TRANSFORM = Transform3d(Translation3d(-0.081165, -0.322330, 0.191168), Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(-(180.0 - 55.0))))
        val FRONT_LEFT_TRANSFORM = Transform3d(Translation3d(-0.012552, 0.319809, 0.191168), Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(70.0)))
        val BACK_LEFT_TRANSFORM = Transform3d(Translation3d(-0.081165, 0.322330, 0.191168), Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(180.0 - 55.0)))
    }
    object Field{
        const val FIELD_X_SIZE = 17.548249
        const val FIELD_Y_SIZE = 8.051800

        const val BLUE_BARGE_SCORING_X = 7.6
        const val RED_BARGE_SCORING_X = FIELD_X_SIZE - BLUE_BARGE_SCORING_X

        const val SAFE_WALL_DISTANCE = 1.0
        /**
         * Coordinates of the center of the reef on the blue side of the field
         */
        val BLUE_REEF_CENTER = Translation2d(Units.inchesToMeters(176.75), Units.inchesToMeters(158.5))

        /** Distance between the center of the robot and the center of the reef
        when the robot is at the center of one of the reef's edges.*/
        val ROBOT_REEF_CENTER_DISTANCE = Units.inchesToMeters(32.25 + 18.0)
        
        /**
         * Branch offset from the center of one of the reef's edges
         */
        val REEF_BRANCH_OFFSET_DISTANCE = Units.inchesToMeters(12.9375/2)

        val TROUGH_OFFSET_DISTANCE = Units.inchesToMeters(14.5/2)

        val blueScoringPoses = (0 until 360 step 60).flatMap { angle ->
            arrayOf(1, -1).flatMap { direction ->
                arrayOf(1, -1).map { side ->
                    Pose2d(BLUE_REEF_CENTER, Rotation2d(0.0)) +
                        Transform2d(
                            Translation2d(
                                -ROBOT_REEF_CENTER_DISTANCE,
                                Rotation2d(Math.toRadians(angle.toDouble()))
                            ), Rotation2d(0.0)
                        ) +
                        Transform2d(
                            Translation2d(
                                direction * REEF_BRANCH_OFFSET_DISTANCE,
                                Rotation2d(Math.toRadians(angle.toDouble() + 90.0))
                            ),
                            Rotation2d(Math.toRadians(angle.toDouble()))
                        ) +
                        Transform2d(
                            0.0,
                            side * Arm.CORAL_CENTER_OFFSET,
                            Rotation2d(-side * Math.PI / 2)
                        )
                }
            }
        }

        val blueTroughScoringPoses = (0 until 360 step 60).flatMap { angle ->
            arrayOf(1, -1).map { direction ->
                Pose2d(BLUE_REEF_CENTER, Rotation2d(0.0)) +
                        Transform2d(
                            Translation2d(
                                -ROBOT_REEF_CENTER_DISTANCE,
                                Rotation2d(Math.toRadians(angle.toDouble()))
                            ), Rotation2d(0.0)
                        ) +
                        Transform2d(
                            Translation2d(
                                direction * TROUGH_OFFSET_DISTANCE,
                                Rotation2d(Math.toRadians(angle.toDouble() + 90.0))
                            ),
                            Rotation2d(Math.toRadians(angle.toDouble()))
                        )
            }
        }

        val blueAlgaeGrabbingPose =  (0 until 360 step 60).flatMap { angle ->
                arrayOf(1, -1).map { side ->
                    Pose2d(BLUE_REEF_CENTER, Rotation2d()) +
                            Transform2d(
                                Translation2d(
                                    -ROBOT_REEF_CENTER_DISTANCE + .05,
                                    Rotation2d(Math.toRadians(angle.toDouble()))
                                ), Rotation2d(Math.toRadians(angle.toDouble()))
                            ) + Transform2d(
                                0.0,
                                side * Arm.CORAL_CENTER_OFFSET,
                                Rotation2d(-side * Math.PI / 2)
                            )
            }
        }

        val redScoringPoses = blueScoringPoses.map{pose -> pose.mirror()}
        val redTroughScoringPoses = blueTroughScoringPoses.map{pose -> pose.mirror()}
        val redAlgaeGrabbingPose = blueAlgaeGrabbingPose.map{pose -> pose.mirror()}
    }
    // InterpolatingDoubleTreeMap
    // This is the amount of angular freedom that the arm has from the upright position when at an elevator height
    var armElevatorPairs = listOf(
        //arm freedom, elevator height
        Math.toRadians(7.0) to 0.0, // This is the arm up elevator down
        Math.toRadians(18.775917) to 0.044542,
        Math.toRadians(26.241907) to 0.068086,
        Math.toRadians(34.899764) to 0.100567,
        Math.toRadians(44.171701) to 0.137335,
        Math.toRadians(55.042173) to 0.184139,
        Math.toRadians(68.556032) to 0.253736,
        Math.toRadians(180.0 - 96.880) to 0.324134,
        Math.toRadians(180.0 - 87.206214) to 0.371332,
        Math.toRadians(180.0 - 82.416832) to 0.396130,
        Math.toRadians(180.0 - 71.851303) to 0.441318,
        Math.toRadians(180.0 - 68.941494) to 0.483176,
        Math.toRadians(180.0 - 65.996990) to 0.521174,
        Math.toRadians(180.0 - 64.161654) to 0.558276,
        Math.toRadians(180.0 - 62.064537) to 0.619367,
        Math.toRadians(180.0 - 58.290246) to 0.654566,
        Math.toRadians(180.0 - 55.419787) to 0.701323 - .0254,
        Math.toRadians(180.0 - 47.462013) to 0.750120 - .0254,
        Math.toRadians(180.0 - 40.585547) to 0.802161 - .0254,
        Math.toRadians(180.0) to Elevator.SAFE_HEIGHT, // This is the arm down elevator up
    )

    var armInterpolationIntakeDown = listOf(
        // elevator is up before changing setpoint for intake up/down
        // arm switch interpolation tables based on location of intake
        // TODO
        Math.toRadians(88.0) to 0.0, // This is the elevator up so intake can move
        Math.toRadians(103.296) to 0.12,
        Math.toRadians(113.75) to 0.202,
        Math.toRadians(117.77) to 0.333,
        Math.toRadians(131.77) to 0.456,
        Math.toRadians(137.593) to 0.533,
        Math.toRadians(180.0) to 0.660, // elevator safe height
    )

    object Swerve{
        val ALIGNMENT_TOLERANCE: Double = 0.04
        val STARTING_TOLERANCE: Double = 0.15

        // TODO: we should do motion magic here too
        //Module constants
        val WHEEL_RADIUS = Units.inchesToMeters(3.9/2)
        const val DRIVE_RATIO = 1.0 / ((16.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0))
        const val TURN_RATIO =  150.0 / 7.0

        // These turn PID values are borrowed straight from Zappy.
        fun makeTurnPID() = PIDController(7.0, 0.0, 0.01).apply {
            enableContinuousInput(-Math.PI, Math.PI)
        }
        fun makeDriveFeedforward() = SimpleMotorFeedforward(
            0.2199442,
            2.18943902193,
            0.0
        )
        fun makeAlignTurnPID() = PIDController(6.0, 0.0, 0.04).apply {
            enableContinuousInput(-Math.PI, Math.PI)
        }
        fun makeAlignDrivePID() = PIDController(5.0, 0.0, 0.01)
        fun makeBargeAlignDrivePID() = PIDController(6.0, 0.0, 0.0)
        val maxAlignTranslationSpeed = 1.5
        val maxAlignRotationSpeed = 2.5
        val maxBargeAlignTranslationSpeed = 1.5
        val maxBargeAlignRotationSpeed = 1.5
        const val MAX_NODE_DISTANCE = 3.0 // meters
        // How fast the robot can move in a straight line (meters/sec).
        val MAX_VELOCITY = (5800.0 / 60) / DRIVE_RATIO * WHEEL_RADIUS * 2*PI
        // How far the swerve modules are from (0,0).
        val XY_DISTANCE = Units.inchesToMeters(13.393747)
        // How fast the robot can rotate (radians/sec).
        val MAX_ANGULAR_VELOCITY = MAX_VELOCITY / (XY_DISTANCE * sqrt(2.0))
        val CHASSIS_RADIUS = (XY_DISTANCE * sqrt(2.0))

        val ALIGN_ANGLE_WEIGHT = 2.7
        val ALIGN_TRANSLATION_WEIGHT = 5.0
        val ALREADY_SCORED_BADNESS = 0.5 + Units.inchesToMeters(12.9375) * ALIGN_TRANSLATION_WEIGHT * (1 - 2*0.3)

        const val FRONT_LEFT_ENCODER_OFFSET = 0.44 // ROTATIONS
        const val FRONT_RIGHT_ENCODER_OFFSET = 0.276 // ROTATIONS
        const val BACK_RIGHT_ENCODER_OFFSET = 0.372 // ROTATIONS
        const val BACK_LEFT_ENCODER_OFFSET = 0.298 // ROTATIONS
    }

    object Arm{
        val PIVOT_CONFIG = TalonFXConfiguration().apply {
            MotorOutput.NeutralMode = NeutralModeValue.Brake
            MotorOutput.Inverted = InvertedValue.Clockwise_Positive
            Feedback.SensorToMechanismRatio = 1.0 / PIVOT_GEAR_RATIO
            Slot0.kS = 0.0
            Slot0.kV = 0.1
            Slot0.kA = 0.0
            Slot0.kG = 0.0
            Slot0.kP = 80.0 // volts per rotation
            MotionMagic.MotionMagicJerk = 9999.0
            MotionMagic.MotionMagicAcceleration = 4.5
            MotionMagic.MotionMagicCruiseVelocity = 2.0 // rps
            CurrentLimits.StatorCurrentLimit = 70.0
            CurrentLimits.SupplyCurrentLimit = 50.0
        }

        const val POSITION_DEPENDENT_KG: Double = 0.33

        val ALLOWED_OPERATING_RANGE = Math.toRadians(-350.0) .. Math.toRadians(350.0)
        const val PIVOT_ENCODER_RATIO = (36.0 / 18.0) * (36.0 / 18.0) * (60.0 / 24.0) * (12.0 / 54.0)

        const val PIVOT_GEAR_RATIO = (12.0 / 60.0) * (20.0 / 60.0) * (12.0 / 54.0)
        const val PIVOT_ABS_ENCODER_OFFSET_ENCODER_ROTATIONS = .7209

        val CORAL_CENTER_OFFSET = Units.inchesToMeters(9.5)
        val SAFE_DISTANCE_FROM_REEF_CENTER = Units.inchesToMeters(70.0)
        val SAFE_PLACEMENT_DISTANCE = Units.inchesToMeters(60.0)
        val SAFE_BARGE_DISTANCE = Units.inchesToMeters(50.0)
        val SAFE_INSIDE_ROBOT_ANGLE = Math.toRadians(40.0)

        // Distance from center of robot to stored coral position (in the forward-backward direction)
        // for aligning with pose estimation


        const val SETPOINT_THRESHOLD = 0.1

        const val CURRENT_DRAW = 20.0
        const val IDLE_CURRENT_DRAW = 10.0
    }
    object Elevator{
        val SPOOL_RADIUS: Double = Units.inchesToMeters(0.75)
        const val GEAR_RATIO: Double = 4.0

        const val ZERO_VOLTAGE = -0.2
        const val ZERO_MIN_CURRENT = 1.7 //amps

        const val SETPOINT_THRESHOLD = 0.01
        const val LAZIER_SETPOINT_THRESHOLD = 0.03

        const val COLLISION_AVOIDANCE_MARGIN = 1.0

        val MAX_EXTENSION = Units.inchesToMeters(55.0)

        const val SAFE_HEIGHT = 0.837198 - .01

        val MOTOR_CONFIG = TalonFXConfiguration().apply {
            Feedback.SensorToMechanismRatio = GEAR_RATIO / (SPOOL_RADIUS * 2*Math.PI)
            MotorOutput.Inverted = InvertedValue.Clockwise_Positive
            MotorOutput.NeutralMode = NeutralModeValue.Brake

            Slot0.kS = 0.0
            Slot0.kV = 0.0
            Slot0.kA = 0.0
            Slot0.kG = 0.37
            Slot0.kP = 70.0
//            MotionMagic.MotionMagicJerk = 9999.0
            MotionMagic.MotionMagicAcceleration = 14.0
            MotionMagic.MotionMagicCruiseVelocity = 3.0
        }
    }

    object Intake{

        val PIVOT_CONFIG = TalonFXConfiguration().apply {
            MotorOutput.Inverted = InvertedValue.Clockwise_Positive
            Feedback.SensorToMechanismRatio = GEAR_RATIO

            Slot0.kS = 0.0
            Slot0.kV = 0.0
            Slot0.kA = 0.0
            Slot0.kG = 0.0
            Slot0.kP = 160.0
            MotionMagic.MotionMagicJerk = 2000.0
            MotionMagic.MotionMagicAcceleration = 200.0
            MotionMagic.MotionMagicCruiseVelocity = 2.0
            MotorOutput.NeutralMode = NeutralModeValue.Coast
        }

        private const val GEAR_RATIO = 1.0 / ((12.0/40.0) * (18.0/46.0) * (18.0/60.0) * (12.0/32.0))
        const val ZERO_VOLTAGE = -0.7
        const val ZERO_MIN_CURRENT = 20.0 // amps

        val SETPOINT_THRESHOLD = Math.toRadians(7.0)

        const val ULTRASONIC_SENSOR_THRESHOLD = 0.02

    }
}
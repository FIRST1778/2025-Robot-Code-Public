package org.chillout1778.subsystems

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import org.chillout1778.Constants
import kotlin.math.PI
import kotlin.math.cos

class SwerveModule(
    val name: String,
    driveMotorID: Int,
    turnMotorID: Int,
    canCoderID: Int,
    encoderOffset: Double,
    driveInverted: InvertedValue,
    turnInverted: InvertedValue,
) : Sendable {
    private val driveMotor: TalonFX = TalonFX(driveMotorID)
    private val turnMotor: TalonFX = TalonFX(turnMotorID)
    private val canCoder: CANcoder = CANcoder(canCoderID)
    init {
        driveMotor.configurator.apply(
            TalonFXConfiguration().apply {
                Feedback = FeedbackConfigs().withSensorToMechanismRatio(Constants.Swerve.DRIVE_RATIO)
                MotorOutput = MotorOutputConfigs().withInverted(driveInverted)
                CurrentLimits = CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(45.0)
//                .withSupplyCurrentThreshold(70.0)
//                .withSupplyTimeThreshold(0.1)
                    // TODO: what did CTRE do with these?
                    .withStatorCurrentLimit(80.0)
                    .withStatorCurrentLimitEnable(true)
            }
        )
        turnMotor.configurator.apply(
            TalonFXConfiguration().apply {
                Feedback = FeedbackConfigs().withSensorToMechanismRatio(Constants.Swerve.TURN_RATIO)
                MotorOutput = MotorOutputConfigs().withInverted(turnInverted)
                CurrentLimits = CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(45.0)
//                .withSupplyCurrentThreshold(70.0)
//                .withSupplyTimeThreshold(0.1)
                    .withStatorCurrentLimit(80.0)
                    .withStatorCurrentLimitEnable(true)
            }
        )
        canCoder.configurator.apply(
            MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(0.5)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        )
        turnMotor.setPosition(canCoder.absolutePosition.valueAsDouble - encoderOffset)
        driveMotor.setNeutralMode(NeutralModeValue.Brake)
        turnMotor.setNeutralMode(NeutralModeValue.Coast)
    }

    private val turnPID = Constants.Swerve.makeTurnPID()
    private val driveFeedforward = Constants.Swerve.makeDriveFeedforward()

    val turnPosition: Double
        get() = MathUtil.angleModulus(turnMotor.position.valueAsDouble * 2*PI)
    private val driveVelocity: Double
        get() = driveMotor.velocity.valueAsDouble * 2*PI * Constants.Swerve.WHEEL_RADIUS
    private val driveAcceleration: Double
        get() = driveMotor.acceleration.valueAsDouble * 2*PI * Constants.Swerve.WHEEL_RADIUS
    private val drivePosition: Double
        get() = driveMotor.position.valueAsDouble * 2*PI * Constants.Swerve.WHEEL_RADIUS

    // Accessors for "overall" statistics, basically combines drive and turn information.
    // State is about drive velocity, position is about drive position.
    val position get() = SwerveModulePosition(
        drivePosition, Rotation2d(turnPosition)
    )
    val state get() = SwerveModuleState(
        driveVelocity, Rotation2d(turnPosition),
    )

    var commandedVolts = 0.0
    fun driveState(state: SwerveModuleState) {
        val optimizedState = SwerveModuleState.optimize(state, Rotation2d.fromRadians(turnPosition))
        val goalTurnPosition = optimizedState.angle.radians
        val goalDriveVelocity = optimizedState.speedMetersPerSecond * cos(turnPosition - goalTurnPosition)
        commandedVelocity = goalDriveVelocity
        turnMotor.setVoltage(turnPID.calculate(turnPosition, goalTurnPosition))
        commandedVolts = driveFeedforward.calculate(goalDriveVelocity, driveAcceleration)
        driveMotor.setVoltage(commandedVolts)
    }

    var commandedVelocity = 0.0

    override fun initSendable(builder: SendableBuilder?) {
        builder!!
        builder.clearProperties()
//        builder.addDoubleProperty("raw turn position", {turnMotor.position.valueAsDouble}, {})
        builder.addDoubleProperty("raw cancoder position", {canCoder.absolutePosition.valueAsDouble}, {})
//        builder.addDoubleProperty("raw drive position (rev)", {driveMotor.position.valueAsDouble}, {})
        builder.addDoubleProperty("turn position (deg)", {Math.toDegrees(turnPosition)}, {})
//        builder.addDoubleProperty("raw drive position (rotations)", {driveMotor.position.valueAsDouble}, {})
        builder.addDoubleProperty("drive velocity (mps)", {driveVelocity}, {})
        builder.addDoubleProperty("drive stator current", {driveMotor.statorCurrent.valueAsDouble}, {})
        builder.addDoubleProperty("command drive voltage", {commandedVolts}, {})
        builder.addDoubleProperty("Commanded Drive Velocity", {commandedVelocity}, {})
//        builder.addDoubleProperty("drive acceleration (mps^2)", {driveAcceleration}, {})
//        builder.addDoubleProperty("raw drive velocity", {driveMotor.velocity.valueAsDouble}, {})

    }
}

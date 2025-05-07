package org.chillout1778.subsystems

import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.units.measure.Current
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.chillout1778.Constants
import org.chillout1778.Constants.Field.BLUE_REEF_CENTER
import org.chillout1778.Controls
import org.chillout1778.Robot
import org.chillout1778.Utils
import org.chillout1778.Utils.mirrorIfRed
import org.chillout1778.Utils.wrapTo0_2PI
import kotlin.math.PI
import kotlin.math.abs
import kotlin.system.measureTimeMillis

object Arm : SubsystemBase() {
    enum class RollerState(val dutyCycle: Double) {
        Off(0.0),
        SlowIdle(-.035),
        FastIdle(-.1),
        Idle(-0.035),
        AlgaeIdle(-.225),
        In(-1.0),
        SlowOut(.075),
        Out(1.0),
        Descore(.8)
    }

    enum class MirrorType { FixedAngle, ActuallyFixedAngle, ClosestToReef, ClosestToPosition, AlgaeScore, ProcessorScore}
    enum class PivotState(private val rawAngle: Double,
            val mirrorType: MirrorType)
    {
        Up(Math.PI, MirrorType.FixedAngle),
        AlgaeUp(Math.PI, MirrorType.FixedAngle),
        Down(0.0, MirrorType.ActuallyFixedAngle),
        ScoreCoral(Math.toRadians(130.0), MirrorType.ClosestToReef),
        FinishScoreCoral(Math.toRadians(105.0), MirrorType.ClosestToReef),
        AboveScoreCoral(Math.toRadians(160.0), MirrorType.ClosestToReef),
        L4ScoreCoral(Math.toRadians(135.0), MirrorType.ClosestToReef),
        L4FinishScoreCoral(Math.toRadians(100.0), MirrorType.ClosestToReef),
        GetAlgae(Math.toRadians(100.0), MirrorType.ClosestToReef),
        PostAlgae(Math.toRadians(110.0), MirrorType.ClosestToReef),
        DescoreAlgae(Math.toRadians(110.0), MirrorType.ClosestToReef),
        SafeInsideRobotAngle(Math.PI - Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE, MirrorType.ClosestToReef),
        PreBarge(Math.toRadians(160.0), MirrorType.AlgaeScore),
        BargeScore(Math.toRadians(160.0), MirrorType.AlgaeScore),
        Processor(Math.toRadians(70.0), MirrorType.ProcessorScore),
        AlgaeGroundPickup(Math.toRadians(-78.0), MirrorType.ActuallyFixedAngle), // out the left
        ExitAlgaeGroundPickup(Math.toRadians(-95.0), MirrorType.ActuallyFixedAngle), // out the left
        PopsiclePickup(Math.toRadians(-80.0), MirrorType.ActuallyFixedAngle),

        ;  // This semicolon is important, it separates the list of states from the methods on the states

        val desiredAngle: Double get() {
            return when (mirrorType) {
                MirrorType.ActuallyFixedAngle, MirrorType.FixedAngle -> rawAngle
                MirrorType.ClosestToPosition -> if (position > 0.0) rawAngle else -rawAngle // TODO: should this handle very small positions differently?
                MirrorType.ClosestToReef -> when (sideCloserToReef) {
                    Side.Left -> -rawAngle
                    Side.Right -> rawAngle
                    // If we can't decide to rotate out Left or Right, then go to Up or Down, whichever is closer to current arm angle.
                    // This could be an error, maybe.
                    Side.Neither -> if (abs(rawAngle) < Math.PI/2) 0.0 else Math.PI
                }
                MirrorType.AlgaeScore -> when(sideCloserToBarge){
                    Side.Left -> -rawAngle
                    Side.Right -> rawAngle
                    // If we can't decide to rotate out Left or Right, then go to Up or Down, whichever is closer to current arm angle.
                    // This could be an error, maybe.
                    Side.Neither -> Math.PI
                }
                MirrorType.ProcessorScore -> when(sideCloserToProcessor){
                    Side.Left -> -rawAngle
                    Side.Right -> rawAngle
                    Side.Neither -> Math.PI
                }
            }
        }
    }

    private val absoluteEncoder = CANcoder(Constants.CanIds.ARM_ENCODER)
    val armPivotMotor = TalonFX(Constants.CanIds.ARM_PIVOT_MOTOR).apply {
        configurator.apply(Constants.Arm.PIVOT_CONFIG)
    }

    val rollerMotor = TalonFX(Constants.CanIds.ARM_ROLLER_MOTOR).apply{
        configurator.apply(CurrentLimitsConfigs().withStatorCurrentLimit(80.0))
    }

    private val statorCurrentSignal: StatusSignal<Current> = rollerMotor.statorCurrent.apply {
        setUpdateFrequency(100.0)
    }

    enum class Side { Left, Right, Neither }

    val deadzoneAngle = Math.toRadians(20.0)
    val sideCloserToReef: Side get() {
        val directionTowardReefCenter: Rotation2d =
            (BLUE_REEF_CENTER.mirrorIfRed() - Swerve.estimatedPose.translation).angle
        val directionTowardRight = Swerve.estimatedPose.rotation.rotateBy(Rotation2d.kCW_90deg)

        // Uses dot product formula (for unit vectors).  This is always between 0 and 180 degrees (0 and pi radians).
        fun angleBetween(r1: Rotation2d, r2: Rotation2d) = Math.acos(r1.cos*r2.cos + r1.sin*r2.sin)
        val ang = angleBetween(directionTowardReefCenter, directionTowardRight)
        assert(ang >= 0.0) // the math works out this way

        return if (PI/2 - deadzoneAngle < ang && ang < PI/2 + deadzoneAngle) {
            Side.Neither
        } else if (ang < Math.PI/2) {
            Side.Right
        } else {
            Side.Left
        }
    }

    val sideCloserToBarge: Side get() {
        val isOnBlue = !Robot.isOnRedSide
        val rotation = Swerve.estimatedPose.rotation.radians

        return if ((rotation < PI &&  rotation > PI - deadzoneAngle) || (rotation > -PI && rotation < -PI + deadzoneAngle) || (rotation > -deadzoneAngle && rotation < deadzoneAngle)) Side.Neither
            else if((isOnBlue && rotation > 0.0) || (!isOnBlue && rotation < 0.0)) Side.Right
            else Side.Left
    }

    val sideCloserToProcessor: Side get() {
        val isOnBlue = !Robot.isOnRedSide
        val rotation = Swerve.estimatedPose.rotation.radians

        return if ((rotation < (PI/2 + deadzoneAngle) && rotation > (PI/2 - deadzoneAngle)) || (rotation > (-PI/2 - deadzoneAngle) && rotation < (-PI/2 + deadzoneAngle))) Side.Neither
        else if((isOnBlue && rotation < PI/2 && rotation > -PI/2) || (!isOnBlue && (rotation > PI/2 || rotation < -PI/2))) Side.Right
        else Side.Left
    }

    fun atSafeReefDistance() =
        Swerve.estimatedPose.translation.getDistance(BLUE_REEF_CENTER.mirrorIfRed()) > Constants.Arm.SAFE_DISTANCE_FROM_REEF_CENTER

    fun atSafePlacementDistance() =
        Swerve.estimatedPose.translation.getDistance(BLUE_REEF_CENTER.mirrorIfRed()) > Constants.Arm.SAFE_PLACEMENT_DISTANCE

    fun atSafeBargeDistance() =
        Swerve.estimatedPose.x < Constants.Field.FIELD_X_SIZE/2 - Constants.Arm.SAFE_BARGE_DISTANCE
                || Swerve.estimatedPose.x > Constants.Field.FIELD_X_SIZE/2 + Constants.Arm.SAFE_BARGE_DISTANCE

    fun atSafeProcessorDistance() =
        Swerve.estimatedPose.y > Constants.Field.SAFE_WALL_DISTANCE && Swerve.estimatedPose.y < (Constants.Field.FIELD_Y_SIZE - Constants.Field.SAFE_WALL_DISTANCE)

    val insideFrame: Boolean get(){
        return abs(MathUtil.angleModulus(position)) < Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE || abs(MathUtil.angleModulus(position)) > (PI - Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE)
    }
    
    val elevatorToArm = InterpolatingDoubleTreeMap().apply {
        for ((armAngle, elevatorHeight) in Constants.armElevatorPairs)
            put(elevatorHeight, armAngle)
    }

    val elevatorToArmWhenIntakeDown = InterpolatingDoubleTreeMap().apply {
        for ((armAngle, elevatorHeight) in Constants.armInterpolationIntakeDown)
            put(elevatorHeight, armAngle)
    }


    var isArmStuck = false
    fun positionFromAngle(angle: Double, respectReef: Boolean): Double {
        val positions: List<Double> = listOf(wrapTo0_2PI(angle), wrapTo0_2PI(angle) - 2*Math.PI)
            .filter { it in Constants.Arm.ALLOWED_OPERATING_RANGE }

        val actualArmPosition = position
        val closeSide = sideCloserToReef

        // OBSTRUCTIONS
        var p: Double = if (positions.size == 1) {
            positions[0]
        } else if (respectReef) {
            when (closeSide) {
                Side.Neither -> {
                    // Choose whichever position is closest to angle
                    positions.minBy { abs(it - actualArmPosition) }
                }
                Side.Right -> {
                    if(actualArmPosition < Math.PI/2) positions[1] // negative position aka rotate out left
                    else positions[0] // for if it is already rotated up to a safe position, rotate to the right angle
                }
                Side.Left -> {
                    if(actualArmPosition > -Math.PI/2) positions[0] // positive position
                    else positions[1]
                }
            }
        } else {
            positions.minBy { abs(it - actualArmPosition) }
        }

        // Keep arm inside robot if the setpoint requires pivoting towards the reef
        // This works because the only time we should be pivoting to a position below 90 degrees is when going to rest state.
        val notAtSafeReefDistance = !atSafeReefDistance()
        p = if(actualArmPosition > Math.PI/2 && p < Math.PI/2 && closeSide == Side.Right && notAtSafeReefDistance){
            isArmStuck = true
            p.coerceAtLeast(Math.PI - Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE)
        }else if(actualArmPosition < -Math.PI/2 && p > -Math.PI/2 && closeSide == Side.Left && notAtSafeReefDistance){
            isArmStuck = true
            p.coerceAtMost(-Math.PI + Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE)
        }else{
            isArmStuck = false
            p
        }

        // CLAMPING, DO NOT MOVE OR REMOVE!
        val actualElevatorHeight = Elevator.height

        val limit = if (Intake.effectivePivotState == Intake.PivotState.Down && Intake.atSetpoint) elevatorToArmWhenIntakeDown.get(actualElevatorHeight)
            else elevatorToArm.get(actualElevatorHeight) // This value is actually measured in radians DOWN from straight-up
        return when{
            MathUtil.isNear(Math.PI, limit, 0.0001) -> p // mitigates IEEE754 issues
            actualArmPosition < 0.0 -> p.coerceIn(-Math.PI - limit, -Math.PI + limit)
            else -> p.coerceIn(Math.PI - limit, Math.PI + limit)
        }
    }

    var lastUpdatedTick: Long = -1
    var lastCachedValue: Double = 0.0
    val desiredPosition: Double get() {
        if (lastUpdatedTick == Robot.tickNumber) {
            return lastCachedValue
        }
        var answer: Double
        val milliseconds: Long = measureTimeMillis {
            answer = positionFromAngle(pivotState.desiredAngle,
                respectReef = pivotState.mirrorType != MirrorType.ActuallyFixedAngle)
        }
        lastUpdatedTick = Robot.tickNumber
        lastCachedValue = answer
        if (milliseconds > 5)
            println("desired position took ${milliseconds} ms")
        return answer
    }

    //region "Easy code": hardware, motion magic, logging.

    var pivotState = PivotState.Up
    private var rollerState = RollerState.Off

    var isZeroed = false
    val coralCurrentDebouncer = Debouncer(0.15, Debouncer.DebounceType.kBoth)
    val algaeCurrentDebouncer = Debouncer(0.25, Debouncer.DebounceType.kBoth)
    var hasObject: Boolean = false
    val autoTimer = Timer()
    val undebouncedHasObject: Boolean get() = if(rollerState == RollerState.Idle || rollerState == RollerState.SlowIdle) statorCurrentSignal.valueAsDouble > Constants.Arm.IDLE_CURRENT_DRAW
            else statorCurrentSignal.valueAsDouble > Constants.Arm.CURRENT_DRAW

    val armOffsetIncrementRadians = Math.toRadians(0.1)

    var armOffsetRadians = 0.0

    override fun periodic() {
//        println("negative ${Controls.wantOffsetArmNegative} positive ${Controls.wantOffsetArmPositive}")
        if (Controls.wantOffsetArmPositive) offsetArm(armOffsetIncrementRadians)
        if (Controls.wantOffsetArmNegative) offsetArm(-armOffsetIncrementRadians)

        val atStartOfAuto = (Robot.isAutonomous && autoTimer.get() < 0.75)
        statorCurrentSignal.refresh()
        val debouncedHasCoral = coralCurrentDebouncer.calculate(undebouncedHasObject)
        val debouncedHasAlgae = algaeCurrentDebouncer.calculate(undebouncedHasObject)
        hasObject = atStartOfAuto || if (rollerState == RollerState.AlgaeIdle) debouncedHasAlgae else debouncedHasCoral
        if (!isZeroed || !Elevator.isZeroed)
            return
        rollerMotor.set(
            if(atStartOfAuto) RollerState.FastIdle.dutyCycle else rollerState.dutyCycle,
        )
        val positionSetpoint = desiredPosition
        val gravityFeedforward = Constants.Arm.POSITION_DEPENDENT_KG * Math.sin(position)

        armPivotMotor.setControl(MotionMagicVoltage((positionSetpoint- armOffsetRadians)/(2*Math.PI)).withFeedForward(gravityFeedforward))
    }

    fun closeClampedPosition(): Double{
        var x = absoluteEncoder.position.valueAsDouble - Constants.Arm.PIVOT_ABS_ENCODER_OFFSET_ENCODER_ROTATIONS
        while (x < -0.5) x+= 1.0
        while (x > 0.5) x -= 1.0
        val rawReadingArmRotations = x * Constants.Arm.PIVOT_ENCODER_RATIO

        val allowedOffsetArmRotations = 12.0/360.0

        // Hack
        if (abs(rawReadingArmRotations - 0.5) < allowedOffsetArmRotations) {
            return 0.5
        } else if (abs(rawReadingArmRotations - (-0.5)) < allowedOffsetArmRotations) {
            return -0.5
        } else {
            return rawReadingArmRotations
        }
    }

    fun resetRelativeFromAbsolute() {
        armPivotMotor.setPosition(closeClampedPosition())
        isZeroed = true
    }

    val position get() = armPivotMotor.position.valueAsDouble * 2*Math.PI + armOffsetRadians

    val atSetpoint: Boolean get() = abs(desiredPosition - position) < Constants.Arm.SETPOINT_THRESHOLD

    fun setState(pivot: PivotState, rollers: RollerState) {
        pivotState = pivot
        rollerState = rollers
    }

    fun setCoastEnabled(coast: Boolean) {
        if (coast) armPivotMotor.setNeutralMode(NeutralModeValue.Coast)
        else armPivotMotor.setNeutralMode(NeutralModeValue.Brake)
    }

    // For when the arm skips.
    fun offsetArm(r: Double) {
        armOffsetRadians += r
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.addDoubleProperty("Arm offset (deg)", {Math.toDegrees(armOffsetRadians)}, {})
        builder.addDoubleProperty("Arm position (deg)", {Math.toDegrees(position)}, {})
        builder.addDoubleProperty("Raw Encoder", {absoluteEncoder.position.valueAsDouble}, {})
//        builder.addDoubleProperty("Clamped position encoder (deg)", { closeClampedPosition() * 360.0 }, {})
        builder.addBooleanProperty("Has object?", {hasObject}, {})
//        builder.addBooleanProperty("Undebounced has object?", { undebouncedHasObject}, {})
        builder.addDoubleProperty("Desired position (deg)", {Math.toDegrees(desiredPosition)}, {})
        builder.addDoubleProperty("Motion magic setpoint (deg)", {360.0*armPivotMotor.closedLoopReference.valueAsDouble}, {})
        // builder.addDoubleProperty("Before-safety desired angle (deg)", {Math.toDegrees(pivotState.desiredAngle)}, {})
        // builder.addStringProperty("Arm pivot state", { pivotState.toString() }, {})
        builder.addBooleanProperty("At setpoint?", {atSetpoint}, {})
        // builder.addDoubleProperty("Zeroing Position", { closeClampedPosition() * 360.0 }, {})
        // builder.addDoubleProperty("angle limit", { elevatorToArm.get(Elevator.height)}, {})
        builder.addDoubleProperty("roller current", { statorCurrentSignal.valueAsDouble},{})
        Utils.addClosedLoopProperties("Arm pivot", armPivotMotor, builder)
        // builder.addBooleanProperty("Coast enabled?", { Robot.wasCoastModeEnabled }, {})
    }
    //endregion
}

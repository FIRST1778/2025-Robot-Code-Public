package org.chillout1778.subsystems

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.math.util.Units
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.chillout1778.Constants
import org.chillout1778.Constants.Field.BLUE_REEF_CENTER
import org.chillout1778.Robot
import org.chillout1778.Utils
import org.chillout1778.Utils.mirrorIfRed
import org.chillout1778.Utils.wrapTo0_2PI
import kotlin.math.abs
import kotlin.system.measureTimeMillis

object Elevator: SubsystemBase() {
    enum class State(private val rawExtension: Double) {
        Down(0.0),
        PreHandoff(Units.inchesToMeters(36.0)),
        Handoff(Units.inchesToMeters(33.25)),
        SourceIntake(Units.inchesToMeters(53.0)),
        PreScore(Units.inchesToMeters(20.0)),
        Trough(Units.inchesToMeters(38.0)),
        L2(Units.inchesToMeters(15.0)),
        L3(L2.rawExtension + Units.inchesToMeters(15.8701)),
        L4(Units.inchesToMeters(54.5 - 0.125)),
        Barge(Units.inchesToMeters(55.0 - 0.125)),
        ScoreL4(L4.rawExtension - Units.inchesToMeters(1.0)),
        ScoreL3(L3.rawExtension - Units.inchesToMeters(3.5)),
        ScoreL2(L2.rawExtension - Units.inchesToMeters(3.5)),
        PostL3(L2.rawExtension - Units.inchesToMeters(6.0)), //TODO: Tune
        PostL2(L2.rawExtension - Units.inchesToMeters(3.5)), //TODO: Tune
        AutoAlgae(Units.inchesToMeters(21.75)),
        LowAlgae(Units.inchesToMeters(22.25)),
        HighAlgae(LowAlgae.rawExtension + Units.inchesToMeters(15.8701)),
        Processor(Units.inchesToMeters(20.0)),
        AlgaeRest(Units.inchesToMeters(15.0)),
        GroundAlgaeIntake(0.14),
        PopsiclePickup(0.065),
        ;
        val extension: Double get() {
            return if (this == AutoAlgae) {
                rawExtension + preferredAlgaeHeight.offset
            } else {
                rawExtension
            }
        }
    }

    private val mainMotor = TalonFX(Constants.CanIds.ELEVATOR_MAIN_MOTOR).apply {
        configurator.apply(Constants.Elevator.MOTOR_CONFIG)
    }
    private val followerMotor = TalonFX(Constants.CanIds.ELEVATOR_FOLLOWER_MOTOR).apply {
        setControl(Follower(mainMotor.deviceID, true))
    }

    var isZeroed = false
    fun zero() {
        mainMotor.setPosition(0.0)
        isZeroed = true
    }
    val statorCurrent get() = mainMotor.statorCurrent.valueAsDouble

    private val armToElevator = InterpolatingDoubleTreeMap().apply {
        for ((armAngle, elevatorHeight) in Constants.armElevatorPairs)
            put(armAngle, elevatorHeight + Units.inchesToMeters(.5))
    }

    private val armToElevatorWhenIntakeDown = InterpolatingDoubleTreeMap().apply {
        for ((armAngle, elevatorHeight) in Constants.armInterpolationIntakeDown)
            put(armAngle, elevatorHeight + Units.inchesToMeters(.5))
    }

    var state = State.Down

    var lastClampedSetpointForLogging: Double = 0.0
    fun clampSetpoint(s: Double): Double{
        var ret: Double = 0.0
        val time = measureTimeMillis {
            val armDesiredPositionSignum = Math.signum(Arm.desiredPosition)
            val interpolationTableInput = Math.PI - abs(MathUtil.angleModulus(
                if(/*armDesiredPositionSignum != 0.0 && */Math.signum(Arm.position) != armDesiredPositionSignum/* && abs(Arm.position) > Math.toRadians(3.0)*/) 0.0
                else if((Arm.position < 0.0 && Arm.desiredPosition > Arm.position) || (Arm.position > 0.0 && Arm.desiredPosition < Arm.position)) Arm.desiredPosition
                else Arm.position
            ))
            ret = s.coerceIn(
                if (Intake.effectivePivotState == Intake.PivotState.Down && Intake.atSetpoint)
                    armToElevatorWhenIntakeDown.get(interpolationTableInput)
                else
                    armToElevator.get(interpolationTableInput),
                Constants.Elevator.MAX_EXTENSION
            )
            lastClampedSetpointForLogging = ret
        }
        if (time > 5) {
            println("Elevator.clampSetpoint() took $time ms")
        }
        return ret
    }

    //convert rotations to radians
    val height get() = mainMotor.position.valueAsDouble
    val velocity get() = mainMotor.velocity.valueAsDouble

    val atSetpoint get() = abs(height - state.extension) < Constants.Elevator.SETPOINT_THRESHOLD
    val lazierAtSetpoint get() = abs(height - state.extension) < Constants.Elevator.LAZIER_SETPOINT_THRESHOLD
    val atOrAboveSetpoint get() = (height + Constants.Elevator.SETPOINT_THRESHOLD) >= state.extension

    fun setZeroingVoltage(){
        mainMotor.setVoltage(Constants.Elevator.ZERO_VOLTAGE)
    }

    fun stop(){
        mainMotor.setVoltage(0.0)
    }

    fun setCoastEnabled(coast: Boolean) {
        if (coast) {
            mainMotor.setNeutralMode(NeutralModeValue.Coast)
            followerMotor.setNeutralMode(NeutralModeValue.Coast)
        } else {
            mainMotor.setNeutralMode(NeutralModeValue.Brake)
            followerMotor.setNeutralMode(NeutralModeValue.Brake)
        }
    }

    override fun periodic() {
        if (!isZeroed || !Arm.isZeroed)
            return
        mainMotor.setControl(MotionMagicVoltage(clampSetpoint(state.extension)))
    }

    enum class AlgaeHeight(val offset: Double) { High(Units.inchesToMeters(15.8701)), Low(0.0) }

    val endOfManipulatorPose: Translation2d get() {
        return Translation2d(Constants.Arm.CORAL_CENTER_OFFSET, 0.0)
            .rotateBy(Swerve.estimatedPose.rotation)
            .plus(Swerve.estimatedPose.translation)
    }

    val preferredAlgaeHeight: AlgaeHeight get() {
        // Find direction of vector from center of reef to the center of the robot
        // (This is counterclockwise from straight-up)
        var degreesAroundReefCenter: Double =
            (endOfManipulatorPose - BLUE_REEF_CENTER.mirrorIfRed()).angle.degrees
        // Mirror angle for red alliance since driver stations always face a high algae,
        // but all our math is from blue alliance perspective so need to invert.
        if (Robot.isRedAlliance)
            degreesAroundReefCenter += 180.0
        //  Subtract 30 degrees so that 0 aligns with a corner of the reef.
        val algaeDirection = Math.toDegrees(wrapTo0_2PI(Math.toRadians(degreesAroundReefCenter - 30.0)))
        return if (300.0 < algaeDirection && algaeDirection < 360.0
            || 180.0 < algaeDirection && algaeDirection < 240.0
            || 60.0 < algaeDirection && algaeDirection < 120.0) {
            AlgaeHeight.Low
        } else  {
            AlgaeHeight.High
        }
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.addDoubleProperty("Height", {height}, {})
        builder.addDoubleProperty("Setpoint", {state.extension}, {})
        builder.addDoubleProperty("Clamped setpoint", { lastClampedSetpointForLogging}, {})
        builder.addDoubleProperty("Motion magic setpoint (deg)", {mainMotor.closedLoopReference.valueAsDouble}, {})
        // builder.addStringProperty("State", {state.toString()}, {})
        builder.addBooleanProperty("At setpoint?", {atSetpoint}, {})
        builder.addBooleanProperty("Is Zeroed?", {isZeroed}, {})
        Utils.addClosedLoopProperties("Elevator", mainMotor, builder)
        // builder.addBooleanProperty("Is Coast enabled?", { Robot.wasCoastModeEnabled }, {})
        // builder.addStringProperty("Preferred Algae Height", { preferredAlgaeHeight.toString() }, {})
    }
}

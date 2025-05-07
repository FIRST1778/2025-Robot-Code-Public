package org.chillout1778.subsystems

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.math.MathUtil
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.chillout1778.Constants
import org.chillout1778.Controls
import org.chillout1778.Utils
import kotlin.math.abs

object Intake : SubsystemBase(){
    // Angles measured positive downward, and the stowed starting position is 0.0.
    enum class PivotState(val angleSetpoint: Double) {
        Down(Math.toRadians(126.0)),
        Trough(Math.toRadians(25.639507)),
        Up(0.0),
        OperatorControl(0.0),
    }
    enum class RollerState(val rollingVolts: Double, val centeringVoltage: Double) {
        In(-6.0, -8.0),
        SlowIn(-2.0, -3.0),
        TroughOut(3.25, 0.0),
        Out(8.0, 0.0),
        Off(0.0, 0.0),
        AlgaeModeIdle(0.0, 0.0),
        OperatorControl(0.0, 0.0),
    }

    private var realPivotState = PivotState.Up
    private var realRollerState = RollerState.Off

    val effectivePivotState get(): PivotState {
        return if (realPivotState != PivotState.OperatorControl) realPivotState
        else if (unsafeToGoUp) PivotState.Down
        else if (hasCoral) PivotState.Up
        else if (Superstructure.inputs.wantGroundIntake) PivotState.Down
        else PivotState.Up
    }
    val effectiveRollerState get(): RollerState {
        return if (realRollerState != RollerState.OperatorControl) realRollerState
        else if (Superstructure.inputs.wantGroundIntake) RollerState.In
        else if (hasCoral) RollerState.SlowIn
        else RollerState.Off
    }

    private val pivotMotor = TalonFX(Constants.CanIds.INTAKE_PIVOT_MOTOR).apply {
        configurator.apply(Constants.Intake.PIVOT_CONFIG)
    }
    private val rollerMotor = TalonFX(Constants.CanIds.INTAKE_ROLLER_MOTOR).apply {
        configurator.apply(MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
    }
    private val centeringMotor = TalonFX(Constants.CanIds.INTAKE_CENTERING_MOTOR).apply {
        configurator.apply(MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
    }

    // TODO: subsume factor of 2pi into gear ratio
    val angle get() = pivotMotor.position.valueAsDouble * 2*Math.PI
    val velocity get() = pivotMotor.velocity.valueAsDouble * 2*Math.PI
    val hasCoral get() = !linebreak.get() || Controls.operatorController.hid.touchpadButton
//val hasCoral get() = Controls.operatorController.hid.circleButton
    val atSetpoint get() = Math.abs(angle - effectivePivotState.angleSetpoint) < Constants.Intake.SETPOINT_THRESHOLD

    var isZeroed: Boolean = false
    fun zero() {
        pivotMotor.setPosition(0.0) // reset relative encoder
        // TODO: do we need to reset the motion magic PID?
        isZeroed = true
    }
    fun setZeroingVoltage() {
        pivotMotor.setVoltage(Constants.Intake.ZERO_VOLTAGE)
    }
    fun stop(){
        pivotMotor.setVoltage(0.0)
        rollerMotor.setVoltage(0.0)
        centeringMotor.setVoltage(0.0)
    }

    override fun periodic() {
        if (!isZeroed)
            return
        pivotMotor.setControl(MotionMagicVoltage(effectivePivotState.angleSetpoint/(2*Math.PI)))
        rollerMotor.setVoltage(effectiveRollerState.rollingVolts)
        centeringMotor.setVoltage(effectiveRollerState.centeringVoltage)
    }

    fun setState(p: PivotState, r: RollerState) {
        realPivotState = p
        realRollerState = if(hasCoral && r == RollerState.Off) { // what in the world is this...
            if(Controls.superstructureInputs.wantedScoringLevel != Superstructure.ScoringLevel.TROUGH)
                RollerState.In
            else RollerState.SlowIn
        }else if(hasCoral && r == RollerState.AlgaeModeIdle){
            RollerState.SlowIn
        }
        else r
    }
    private val linebreak = DigitalInput(Constants.DioIds.INTAKE_LINEBREAK)

    private val unsafeToGoUp: Boolean get() {
        return abs(MathUtil.angleModulus(Arm.position)) < Math.PI - Arm.elevatorToArm.get(Elevator.height)
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.addDoubleProperty("Intake angle", { Math.toDegrees(angle) }, {})
        builder.addDoubleProperty("Intake setpoint", { Math.toDegrees(effectivePivotState.angleSetpoint) }, {})
        builder.addBooleanProperty("at setpoint?", { atSetpoint }, {})
        builder.addBooleanProperty("Intake have coral", { hasCoral }, {})
        // builder.addDoubleProperty("centering voltage", { effectiveRollerState.centeringVoltage }, {})
        // builder.addBooleanProperty("linebreak", { linebreak.get() }, {})
        builder.addStringProperty("Effective intake pivot state", { effectivePivotState.toString()}, {})
        builder.addStringProperty("Underlying intake pivot state", { realPivotState.toString()}, {})
        builder.addBooleanProperty("Is Zeroed?", {isZeroed}, {})
        Utils.addClosedLoopProperties("Intake Pivot", pivotMotor, builder)
        builder.addBooleanProperty("unsafe for intake to go up?", {unsafeToGoUp}, {})
    }
}
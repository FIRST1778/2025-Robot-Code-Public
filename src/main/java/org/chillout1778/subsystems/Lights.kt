package org.chillout1778.subsystems

import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Hertz
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.measure.Frequency
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.chillout1778.Robot
import kotlin.math.abs

// right - 34 leds, left - 33, cross - 20

object Lights: SubsystemBase() {
    private val LENGTH = 86 // probs should be a constant
    private var leds: AddressableLED = AddressableLED(3).apply {
        setColorOrder(AddressableLED.ColorOrder.kRGB)
    }
    var ledBuff : AddressableLEDBuffer = AddressableLEDBuffer(LENGTH)

    var rightSegment = ledBuff.createView(0, 32) // right segment
    var crossSegment = ledBuff.createView(33, 52) // cross segment
    var leftSegment = ledBuff.createView(53, 85).reversed() // left segment

    var batteryIndicator = ledBuff.createView(34, 48) // battery charge progress bar
    var batteryIndicatorMirror = ledBuff.createView(49, 53).reversed()

    var blackPattern = LEDPattern.solid(Color.kBlack)
    var lightsTimer = Timer()

    init {
        lightsTimer.reset()
        lightsTimer.start()

        leds.setLength(LENGTH)
        leds.start()
    }


   override fun periodic() {
       val status = getDisabledStatus()
       if (Robot.isEnabled) {
           if (RobotController.isBrownedOut()) {
               writeLedColor(LedColors.PoopBrown)
           } else if (!status.CANHealthy) {
               writeLedColor(LedColors.BadRed)
           } else if (Swerve.isAligned) {
//                nuclearRats(3.0,
//                    if (Superstructure.inputs.wantedScoringLevel == Superstructure.ScoringLevel.TROUGH)
//                        LedColors.LightBlue
//                    else
//                        LedColors.DarkBlue)
               blinkGreen()
           } else if (Arm.hasObject) {
               writeLedColor(LedColors.PureWhite)
           } else if (Intake.hasCoral) {
               writeLedColor(LedColors.LightBlue)
           } else {
               writeLedColor(LedColors.DarkBlue)
           }
           if (Arm.isArmStuck) {
               setCrossbarColor(LedColors.BadRed)
           }
       } else {
           writeDisabledStatus(status)
           disabledAnimations()
       }

       leds.setData(ledBuff)
   }



    fun writeLedColor(c: LedColors) {
        c.solidPattern.applyTo(ledBuff)
    }

    data class RobotStatus(
        val CANHealthy : Boolean,
        val armCorrectOrientation: Boolean,
        val batteryVoltage: Double,
        val camerasConnected: Boolean,
        val brakeMode: Boolean,
        val intakeHasCoral: Boolean,
    )

    enum class LedColors(val color: Color, val solidPattern: LEDPattern = LEDPattern.solid(color)) {
        GoodGreen(Color(0, 255, 0)),
        BadRed(Color(255, 0, 0)),
        WarningYellow(Color(255, 255, 0)),
        PureWhite(Color(255, 255, 255)),
        LightBlue(Color(0, 175, 255)),
        DarkBlue(Color(0, 0, 255)),
        PoopBrown(Color(160,82,45)),
        TotalBlack(Color.kBlack),
    }

    fun getDisabledStatus(): RobotStatus {
//        try {
//            volts = Robot.pdh.voltage
//        } catch (_: Exception) {}
        return RobotStatus(
            CANHealthy = Arm.armPivotMotor.isConnected && Arm.rollerMotor.isConnected,
            armCorrectOrientation = abs(Arm.closeClampedPosition()) == 0.5,
            batteryVoltage = 12.7,
            camerasConnected = Vision.allConnected(),
            brakeMode = !Robot.wasCoastModeEnabled,
            intakeHasCoral = Intake.hasCoral,
        )
    }

    fun boolColor(b: Boolean) = if (b) LedColors.DarkBlue.color else LedColors.BadRed.color

    var allChecksGood = false

    fun writeDisabledStatus(state: RobotStatus) {
        blackPattern.applyTo(ledBuff)
        LedColors.DarkBlue.solidPattern.applyTo(crossSegment)

        ledBuff.setLED(48, boolColor(state.camerasConnected))
        ledBuff.setLED(47, boolColor(state.camerasConnected))

        ledBuff.setLED(46, boolColor(state.CANHealthy))
        ledBuff.setLED(45, boolColor(state.CANHealthy))

        ledBuff.setLED(44, boolColor(state.armCorrectOrientation))
        ledBuff.setLED(43, boolColor(state.armCorrectOrientation))

        ledBuff.setLED(42, boolColor(state.brakeMode))
        ledBuff.setLED(41, boolColor(state.brakeMode))

        ledBuff.setLED(40, boolColor(!state.intakeHasCoral))
        ledBuff.setLED(39, boolColor(!state.intakeHasCoral))
    }

    var progressBarAnimation: Double = 0.0
    var disabledRainbow = LEDPattern.rainbow(255, 255)
                         .scrollAtRelativeSpeed(Frequency.ofBaseUnits(0.5, Units.Hertz))

    fun disabledAnimations() {
        if (Robot.wasEnabledThenDisabled && lightsTimer.get() <= 5.0) {
            disabledRainbow.applyTo(rightSegment)
            disabledRainbow.applyTo(leftSegment)
            lightsTimer.restart()
        } else if (Robot.wasEnabledThenDisabled && lightsTimer.get() > 5.0) {
            Robot.wasEnabledThenDisabled = false
            lightsTimer.restart()
        // first two seconds after robot code boots up, complete a blue progressBar
        } else if (lightsTimer.get() <= 2.0) {

            progressBarAnimation += 0.02

            if (progressBarAnimation >= 1.0) {
                progressBarAnimation = 0.0
            }

            var progressBarPattern = LEDPattern.solid(LedColors.LightBlue.color)
                .mask(LEDPattern.progressMaskLayer {progressBarAnimation})

            progressBarPattern.applyTo(rightSegment)
            progressBarPattern.applyTo(leftSegment)

            

        // then recursively, every 10 seconds run a light across the bar
        } else if (lightsTimer.get() % 10.0 <= 1.0) {
            nuclearRats(2.0, overlayColor = LedColors.LightBlue)
        }
    }

    fun nuclearRats(frequency: Double = 4.0, baseColor: LedColors = LedColors.TotalBlack,
                    overlayColor: LedColors = LedColors.GoodGreen ) {
        val overlayStepsPattern = LEDPattern.gradient(
            LEDPattern.GradientType.kContinuous,
            baseColor.color,
            overlayColor.color
        )
        val overlayStepsFinal = overlayStepsPattern.scrollAtRelativeSpeed(Frequency.ofBaseUnits(frequency, Units.Hertz))

        val overlayCenterMask = LEDPattern.solid(baseColor.color)
        val overlayCenterFlashed = overlayCenterMask.blink(Time.ofBaseUnits(1.0/frequency, Units.Seconds),
                                                            Time.ofBaseUnits(1.0/frequency, Units.Seconds)
        )
        val overlayFinalPattern = overlayCenterMask.overlayOn(overlayCenterFlashed)
        overlayStepsFinal.applyTo(leftSegment)
        overlayStepsFinal.applyTo(rightSegment)
        if (baseColor != LedColors.TotalBlack)
            overlayFinalPattern.applyTo(crossSegment)
        
    }

    val blinkyPattern = LEDPattern.solid(LedColors.GoodGreen.color).blink(Seconds.of(0.15))

    fun blinkGreen() {
        blinkyPattern.applyTo(ledBuff)
        
    }

    fun setCrossbarColor(c: LedColors) {
        c.solidPattern.applyTo(crossSegment)
    }

}

package org.chillout1778.commands

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.Constants
import org.chillout1778.subsystems.Intake

class ZeroIntakeCommand(private val forced : Boolean = false): Command() {
    init {
        addRequirements(Intake)
    }

    private var shouldZero = false

    override fun initialize() {
        if (Intake.isZeroed && !forced) { // VERY IMPORTANT, DO NOT REMOVE
            // If the intake is already zeroed and we're not forcing,
            // just cancel this command
            shouldZero = false
            this.cancel()
            return
        }
        Intake.isZeroed = false
        Intake.setZeroingVoltage()
        shouldZero = true
    }

    override fun isFinished(): Boolean {
        return Intake.velocity < Constants.Intake.ZERO_MIN_CURRENT
    }

    override fun end(interrupted: Boolean) {
        if (!interrupted && shouldZero) { // VERY IMPORTANT, DO NOT REMOVE
            Intake.stop()
            Intake.zero()
        }
    }
}
package org.chillout1778.commands

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.Constants
import org.chillout1778.subsystems.Elevator

class ZeroElevatorCommand(private val forced: Boolean = false) : Command() {
    init {
        addRequirements(Elevator)
    }

    private var shouldZero = false

    override fun initialize() {
        if (Elevator.isZeroed && !forced) { // VERY IMPORTANT, DO NOT REMOVE
            // If the elevator is already zeroed and we're not forcing,
            // just cancel this command
            shouldZero = false
            this.cancel()
            return
        }
        Elevator.isZeroed = false
        Elevator.setZeroingVoltage()
        shouldZero = true
    }

    override fun isFinished(): Boolean {
        return Elevator.statorCurrent > Constants.Elevator.ZERO_MIN_CURRENT
    }

    override fun end(interrupted: Boolean) {
        if (!interrupted && shouldZero) { // VERY IMPORTANT, DO NOT REMOVE
            Elevator.stop()
            Elevator.zero()
        }
    }
}
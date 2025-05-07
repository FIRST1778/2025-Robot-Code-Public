package org.chillout1778.commands

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.Controls
import org.chillout1778.subsystems.Superstructure

class TeleopSuperstructureCommand(): Command() {
    init {
        addRequirements(Superstructure)
    }

    override fun execute() {
        Superstructure.inputs = Controls.superstructureInputs
    }

    override fun end(interrupted: Boolean) {
        Superstructure.emptyInputs()
    }
}
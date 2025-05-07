package org.chillout1778.commands

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.subsystems.Arm

class ZeroArmCommand(private val forced: Boolean = false): Command() {
    init{
        addRequirements(Arm)
    }

    override fun initialize() {
        if(Arm.isZeroed && !forced){ // VERY IMPORTANT, DO NOT REMOVE
            this.cancel()
            return
        }
        Arm.resetRelativeFromAbsolute()
    }
    override fun isFinished() = true
}
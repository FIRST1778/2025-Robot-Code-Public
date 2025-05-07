package org.chillout1778.commands

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.subsystems.Arm
import org.chillout1778.subsystems.Elevator
import org.chillout1778.subsystems.Superstructure

class Qual59Command: Command() {
    /*
                225  135  L4ScoreCoral
       212.28   260  100  L4FinishScoreCoral
       212.70   0    0    Down
       213.24   250  110  DescoreAlgae
     */
    init {
        addRequirements(Superstructure, Arm, Elevator)
    }
    val timer = Timer()
    var phase = 0
    override fun initialize() {
        timer.reset()
        timer.stop()
        Elevator.state = Elevator.State.Barge
        phase = 0
    }
    override fun execute() {
        if (Elevator.state != Elevator.State.Barge || !Elevator.atSetpoint)
            return // unnecessary safeties
        when (phase) {
            0 -> {
                if (Elevator.atSetpoint) {
                    phase = 1
                    Arm.pivotState = Arm.PivotState.L4ScoreCoral
                }
            }
            1 -> {
                if (Arm.atSetpoint) {
                    phase = 2
                    Arm.pivotState = Arm.PivotState.L4FinishScoreCoral
                    timer.restart()
                }
            }
            2 -> {
                if (timer.hasElapsed(0.42)) {
                    phase = 3
                    Arm.pivotState = Arm.PivotState.Down
                }
            }
            3 -> {
                if (timer.hasElapsed(0.96)) {
                    phase = 4
                    Arm.pivotState = Arm.PivotState.DescoreAlgae
                }
            }
        }
    }
    override fun end(interrupted: Boolean) {
        Superstructure.setStates()
    }
}
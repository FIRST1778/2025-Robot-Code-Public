package org.chillout1778

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.chillout1778.subsystems.Arm
import org.chillout1778.subsystems.Elevator
import org.chillout1778.subsystems.Superstructure
import kotlin.math.abs

object Controls {
    private val driverController = CommandGenericHID(0)
    val operatorController = CommandPS5Controller(1)

    enum class AlignMode { None, ReefAlign, TroughAlign, AlgaeAlign, BargeAlign }

    data class DriveInputs(
        val forward: Double,
        val left: Double,
        val rotation: Double,
        val deadzone: Double,
        val alignMode: AlignMode,
    ){
        val nonZero: Boolean get() = abs(forward) > deadzone || abs(left) > deadzone || abs(rotation) > deadzone
        val redFlipped: DriveInputs get() = this.copy(forward = -forward, left = -left)
    }

    val emptyInputs = DriveInputs(0.0, 0.0, 0.0, 0.0, AlignMode.None)

    val driverInputs get() = DriveInputs(
        forward = driverController.getRawAxis(2),
        left = -driverController.getRawAxis(3),
        rotation = -driverController.getRawAxis(0),
        deadzone = 0.05,
        alignMode =
            if (wantBargeAutoAlign) AlignMode.BargeAlign
            else if (wantCoralAutoAlign && superstructureInputs.wantedScoringLevel != Superstructure.ScoringLevel.TROUGH) AlignMode.ReefAlign
            else if(wantCoralAutoAlign && superstructureInputs.wantedScoringLevel == Superstructure.ScoringLevel.TROUGH) AlignMode.TroughAlign
            else if(wantAlgaeAutoAlign && superstructureInputs.wantGetAlgae && !Arm.hasObject) AlignMode.AlgaeAlign
            else AlignMode.None
    )
    val operatorInputs: DriveInputs get(){
        return DriveInputs(
            forward = -operatorController.hid.leftY,
            left = -operatorController.hid.leftX,
            rotation = -operatorController.hid.rightX,
            deadzone = 0.1,
            alignMode =
                if (wantBargeAutoAlign) AlignMode.BargeAlign
                else if (wantCoralAutoAlign && superstructureInputs.wantedScoringLevel != Superstructure.ScoringLevel.TROUGH) AlignMode.ReefAlign
                else if(wantCoralAutoAlign && superstructureInputs.wantedScoringLevel == Superstructure.ScoringLevel.TROUGH) AlignMode.TroughAlign
                else if(wantAlgaeAutoAlign && superstructureInputs.wantGetAlgae && !Arm.hasObject) AlignMode.AlgaeAlign
                else AlignMode.None
        )
    }

    val wantCoralAutoAlign get() = superstructureInputs.wantExtend
    val wantAlgaeAutoAlign get() = superstructureInputs.wantGetAlgae && Arm.atSetpoint && Elevator.atSetpoint
    val wantBargeAutoAlign get() = superstructureInputs.wantExtend
            && (Superstructure.state == Superstructure.State.AlgaeRest
            || Superstructure.state == Superstructure.State.PreBarge || Superstructure.state == Superstructure.State.ScoreBarge)

    val wantOffsetArmPositive get() = operatorController.hid.leftX > 0.9 && operatorController.hid.l3Button
    val wantOffsetArmNegative get() = operatorController.hid.leftX < -0.9 && operatorController.hid.l3Button

    var lastScoringLevel = Superstructure.ScoringLevel.TROUGH
    val superstructureInputs: Superstructure.SuperstructureInputs get() {
        val level = when (operatorController.hid.pov) {
            0 -> Superstructure.ScoringLevel.L4
            270 -> Superstructure.ScoringLevel.L3
            180 -> Superstructure.ScoringLevel.L2
            90 -> Superstructure.ScoringLevel.TROUGH
            else -> lastScoringLevel
        }
        lastScoringLevel = level
        return Superstructure.SuperstructureInputs(
            wantExtend = operatorController.hid.l2Button,
            wantScore = (driverController.getRawAxis(4) > .5 || operatorController.hid.r3Button)
                    /*&& (Swerve.withinScoringTolerance || level == Superstructure.ScoringLevel.TROUGH)*/, //TODO: Check drone input, operator is backup
            wantGroundIntake = operatorController.hid.r2Button,
            wantArmSourceIntake = operatorController.hid.crossButton,
            wantedScoringLevel = level,
            wantGetAlgae = operatorController.hid.r1Button,
            wantDescoreAlgae = operatorController.hid.triangleButton,
            wantResetSuperstructure = operatorController.hid.optionsButton,
            wantSourceIntake = operatorController.hid.squareButton,
            wantScoreProcessor = operatorController.hid.circleButton,
            wantAlgaeGroundIntake = operatorController.hid.l1Button
//            wantPopsiclePickup = false//operatorController.hid.l1Button
        )
    }
}

package org.chillout1778.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.Constants
import org.chillout1778.Controls
import java.util.function.Supplier
import org.chillout1778.Controls.DriveInputs
import org.chillout1778.Robot
import org.chillout1778.subsystems.Swerve
import kotlin.math.*
import org.chillout1778.Utils
import org.chillout1778.subsystems.Lights

class TeleopDriveCommand(
    private val driveInputsSupplier: Supplier<DriveInputs>
) : Command() {
    init {
        addRequirements(Swerve)
    }

    var previousAlignMode = Controls.AlignMode.None
    val xPID: PIDController = Constants.Swerve.makeAlignDrivePID()
    val yPID: PIDController = Constants.Swerve.makeAlignDrivePID()
    val turnPID: PIDController = Constants.Swerve.makeAlignTurnPID()

    override fun execute() {
        var inputs = driveInputsSupplier.get()
        if (Robot.isRedAlliance)
            inputs = inputs.redFlipped

        if (inputs.nonZero && !Robot.isAutonomous) // TODO: would that autonomous check really work?
            inputs = inputs.copy(alignMode = Controls.AlignMode.None) // if drive inputs non-zero, don't do align

        if (inputs.alignMode != previousAlignMode) {
            xPID.reset()
            yPID.reset()
            turnPID.reset()
        }
        previousAlignMode = inputs.alignMode

        Swerve.isAligned = false // default to false, override later
        val speeds: ChassisSpeeds = if (inputs.alignMode == Controls.AlignMode.None) {
            chassisSpeedsFromDriveInputs(inputs)
        } else if (inputs.alignMode == Controls.AlignMode.BargeAlign) {
            // TODO: Swerve.isAligned = something
            ChassisSpeeds(
                fixBargeTranslationInput(
                    xPID.calculate(Swerve.estimatedPose.x, if(Robot.isOnRedSide)
                        Constants.Field.FIELD_X_SIZE - Constants.Field.BLUE_BARGE_SCORING_X else Constants.Field.BLUE_BARGE_SCORING_X)
                ),
                0.0,
                fixRotationInput(
                    turnPID.calculate(Swerve.estimatedPose.rotation.radians,
                        listOf(Math.PI/2, -Math.PI/2).minBy { abs(it - Swerve.estimatedPose.rotation.radians) })
                )
            )
        } else {
//            Swerve.fudgingMakesADifference = false
            val pose: Pose2d? = when (inputs.alignMode) {
                Controls.AlignMode.TroughAlign -> Swerve.closestTroughScoringPose
                Controls.AlignMode.ReefAlign -> {
                    val fudged = Swerve.closestFudgedScoringPose
//                    val unfudged = Swerve.closestCoralScoringPose
//                    Swerve.fudgingMakesADifference = fudged != null && fudged.value != unfudged
                    fudged?.value
                }
                Controls.AlignMode.AlgaeAlign -> Swerve.closestAlgaeGrabPose
                else -> error("unreachable")
            }
            if (pose == null) {
                chassisSpeedsFromDriveInputs(inputs)
            } else {
                Swerve.isAligned = Swerve.withinTolerance(pose.translation)
                ChassisSpeeds(
                    fixTranslationInput(
                        xPID.calculate(Swerve.estimatedPose.x, pose.x)
                    ),
                    fixTranslationInput(
                        yPID.calculate(Swerve.estimatedPose.y, pose.y)
                    ),
                    fixRotationInput(
                        turnPID.calculate(Swerve.estimatedPose.rotation.radians, pose.rotation.radians)
                    )
                )
            }
        }

        Swerve.driveFieldRelative(speeds)
    }
    fun fixRotationInput(n: Double): Double{
        return Utils.unclampedDeadzone(n.coerceIn(
                -Constants.Swerve.maxAlignRotationSpeed, Constants.Swerve.maxAlignRotationSpeed
        ), 0.03)
    }
    fun fixTranslationInput(n: Double): Double {
        return Utils.unclampedDeadzone(n.coerceIn(
            -Constants.Swerve.maxAlignTranslationSpeed, Constants.Swerve.maxAlignTranslationSpeed
        ), 0.03)
    }
    fun fixBargeTranslationInput(n: Double): Double {
        return Utils.unclampedDeadzone(n.coerceIn(
            -Constants.Swerve.maxBargeAlignTranslationSpeed, Constants.Swerve.maxBargeAlignTranslationSpeed
        ), 0.03)
    }
    fun chassisSpeedsFromDriveInputs(inputs: DriveInputs): ChassisSpeeds {
        val x = inputs.forward
        val y = inputs.left
        var rotation = inputs.rotation
        val theta = atan2(y, x)
        var r = hypot(x, y)
        r = Utils.deadZone(r, inputs.deadzone)
        rotation = Utils.deadZone(rotation, inputs.deadzone)
        r = r*r
        rotation = rotation*rotation*Math.signum(rotation)
        val actualX = r * cos(theta) * Constants.Swerve.MAX_VELOCITY
        val actualY = r * sin(theta) * Constants.Swerve.MAX_VELOCITY
        val actualRotation = rotation * Constants.Swerve.MAX_ANGULAR_VELOCITY
        return ChassisSpeeds(actualX, actualY, actualRotation)
    }

}

package org.chillout1778

import choreo.trajectory.SwerveSample
import choreo.trajectory.Trajectory
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.MotionMagicIsRunningValue
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.util.sendable.SendableBuilder
import kotlin.math.abs

object Utils {
    fun deadZone(inpt: Double, zone: Double): Double {
        // Fancy deadzone: interpolate values from 0.1 to 1.0 into the
        // range 0.0 to 1.0.
        // https://web.archive.org/web/20181021234413/http://www.gamasutra.com/blogs/JoshSutphin/20130416/190541/Doing_Thumbstick_Dead_Zones_Right.php
        return if (abs(inpt) < zone) 0.0
            else if (inpt > 1.0) 1.0
            else if (inpt < -1.0) -1.0
            else ((inpt - zone) / (1.0 - zone))
    }

    fun unclampedDeadzone(inpt: Double, zone: Double) = if(abs(inpt) < zone) 0.0 else inpt

    fun wrapTo0_2PI(a: Double): Double {
        val asdf = MathUtil.angleModulus(a) // wraps from -PI to +PI
        return if (asdf < 0) asdf + 2*Math.PI else asdf
    }

    fun Translation2d.isInsideField(): Boolean {
        return this.x > 0.0 && this.y > 0.0 && this.x < Constants.Field.FIELD_X_SIZE && this.y < Constants.Field.FIELD_Y_SIZE
    }

    fun Translation2d.mirror() = Translation2d(Constants.Field.FIELD_X_SIZE - this.x, this.y)
    fun Rotation2d.mirror() = Rotation2d.kPi - this
    fun Pose2d.mirror() = Pose2d(this.translation.mirror(), this.rotation.mirror())

    fun Translation2d.mirrorIfRed() = if(Robot.isRedAlliance) this.mirror() else this
    fun Rotation2d.mirrorIfRed() = if(Robot.isRedAlliance) this.mirror() else this
    fun Pose2d.mirrorIfRed() = if(Robot.isRedAlliance) this.mirror() else this

    fun addClosedLoopProperties(name: String, motor: TalonFX, builder: SendableBuilder) {
        builder.addDoubleProperty("${name} voltage (V)", { motor.motorVoltage.valueAsDouble }, {})
//        builder.addDoubleProperty("${name} supply Current (As)", { motor.supplyCurrent.valueAsDouble }, {})
//        builder.addDoubleProperty("${name} stator Current (A)", { motor.statorCurrent.valueAsDouble }, {})
        builder.addDoubleProperty("${name} raw velocity", { motor.velocity.valueAsDouble }, {})
        builder.addDoubleProperty("${name} raw acceleration", { motor.acceleration.valueAsDouble }, {})
//        builder.addDoubleProperty("${name} position", { motor.position.valueAsDouble }, {})
//        builder.addDoubleProperty("${name} motion magic setpoint*360", { motor.closedLoopReference.valueAsDouble*360.0}, {})

    }
}

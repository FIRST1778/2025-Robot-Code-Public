package org.chillout1778.commands

import choreo.trajectory.EventMarker
import choreo.trajectory.SwerveSample
import choreo.trajectory.Trajectory
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.Constants
import org.chillout1778.Robot
import org.chillout1778.subsystems.*

class AutoRunnerCommand(private val trajectory: Trajectory<SwerveSample>): Command(), Sendable {
    init {
        addRequirements(Superstructure)
    }

    private data class Event(
        val name: String,
        val timestamp: Double = 0.0,
        val inputs: Superstructure.SuperstructureInputs,
        val waitCondition: (()->Boolean)? = null,
        val requireAlignment: Boolean = false
    )

    private val eventTypes = listOf(
        Event("startL4",
            inputs = Superstructure.SuperstructureInputs(
                wantedScoringLevel = Superstructure.ScoringLevel.L4,
                wantExtend = true,
            )
        ),
        Event("L4",
            inputs = Superstructure.SuperstructureInputs(
                wantedScoringLevel = Superstructure.ScoringLevel.L4,
                wantExtend = true,
                wantScore = true
            ),
            waitCondition = {
                !Arm.hasObject || (Superstructure.state == Superstructure.State.PlaceL4 && Arm.atSetpoint && Elevator.atSetpoint)
            },
            requireAlignment = true
        ),
        Event("intakeDown",
            inputs = Superstructure.SuperstructureInputs(
                wantGroundIntake = true
            )
        ),
        Event("intakeDownNoHandoff",
            inputs = Superstructure.SuperstructureInputs(
                wantGroundIntake = true,
                wantedScoringLevel = Superstructure.ScoringLevel.TROUGH,
            )
        ),
        Event("intakeUp",
            inputs = Superstructure.SuperstructureInputs()
        ),
        Event("zeroInputs",
            // yes, these two are the same, but I don't want to
            // break old autos with intakeUp
            inputs = Superstructure.SuperstructureInputs()
        ),
        Event("startGetAlgae",
            inputs = Superstructure.SuperstructureInputs(
                wantGetAlgae = true
            )
        ),
        Event("waitGetAlgae",
            inputs = Superstructure.SuperstructureInputs(
                wantGetAlgae = true
            ),
            waitCondition = { Arm.hasObject },
            requireAlignment = true
        ),
        Event("extendAlgae",
            inputs = Superstructure.SuperstructureInputs(
                wantExtend = true
            ),
        ),
        Event("scoreAlgae",
            inputs = Superstructure.SuperstructureInputs(
                wantExtend = true,
                wantScore = true
            ),
        ),
        Event("verticalCoral",
            inputs = Superstructure.SuperstructureInputs(
                wantPopsiclePickup = true
            ),
        ),
    )

    private fun eventFromEventMarker(ev: EventMarker): Event {
        for (type in eventTypes) {
            if (type.requireAlignment) assert(type.waitCondition != null) // it simplifies logic if requireAlign only happens when waitCondition!=null
            if (type.name == ev.event) {
                return type.copy(timestamp = ev.timestamp)
            }
        }
        throw Error("unrecognized event marker!!!")
    }

    private val timer = Timer()
    private val events: List<Event> = trajectory.events().sortedBy { it.timestamp }.map { eventFromEventMarker(it) }
    private var eventI = 0
    private var currentWaitEvent: Event? = null
    private var waitingForAlign: Boolean = false
    private var postAlignInputs: Superstructure.SuperstructureInputs? = null

    // Theoretically waitingForAlign and postAlignInputs could be merged
    // in the same way that currentWaitEvent encodes whether we're waiting based on whether it's null or not
    // But it's kinda stupid and I don't know a good name

    override fun initialize() {
        timer.restart()
        val initialPose = trajectory.getInitialPose(Robot.isRedAlliance).get()
        if (Swerve.estimatedPose.translation.getDistance(initialPose.translation) > Constants.Swerve.STARTING_TOLERANCE)
            Swerve.estimatedPose = initialPose
    }

    private fun shouldRunEvent(ev: Event): Boolean {
        return timer.hasElapsed(ev.timestamp) || timer.hasElapsed(trajectory.totalTime)
    }

    var lastPose: Pose2d? = null

    override fun execute() {
        if (timer.isRunning) // could remove this once tested
            assert(currentWaitEvent == null)
        else
            assert(currentWaitEvent != null)

        if (waitingForAlign) {
            assert(lastPose != null)
            if (Swerve.withinTolerance(lastPose!!.translation)) {
                Superstructure.inputs = postAlignInputs!!
                waitingForAlign = false
            }
            Swerve.followPose(lastPose!!)
            return
        }

        if (currentWaitEvent != null && currentWaitEvent!!.waitCondition!!.invoke()) { // Are we waiting for something and it's done?
            Superstructure.emptyInputs()
            currentWaitEvent = null
        } else if (currentWaitEvent == null && eventI < events.size && shouldRunEvent(events[eventI])) { // Is there a new event?
            val ev = events[eventI++]
            if (ev.requireAlignment)
                postAlignInputs = ev.inputs
            else
                Superstructure.inputs = ev.inputs
            if (ev.waitCondition != null) {
                // process waiting event
                currentWaitEvent = ev
                waitingForAlign = ev.requireAlignment
                Swerve.stop()
                timer.stop() // timer can't run while we wait for this to finish
            }
        }

        if (currentWaitEvent == null) { // True when we're driving
            timer.start() // start timer again while we're driving
            val sample = trajectory.sampleAt(timer.get(), Robot.isRedAlliance).orElse(
                // idea here is that after the trajectory ends we can PID to the last pose
                trajectory.getFinalSample(Robot.isRedAlliance).get()
            )
            lastPose = sample.pose
            Swerve.followSample(sample)
        }
    }

    override fun isFinished() = timer.hasElapsed(trajectory.totalTime) && currentWaitEvent == null && eventI >= events.size

    override fun end(interrupted: Boolean) {
        Swerve.stop()
        Superstructure.emptyInputs()
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.addBooleanProperty("timer running?", { timer.isRunning }, {})
        builder.addDoubleProperty("timer time", { timer.get() }, {})
        builder.addDoubleProperty("trajectory total time", { trajectory.totalTime }, {})
        builder.addStringProperty("current wait event", { currentWaitEvent?.name ?: "(none)" }, {})
    }
}
package org.chillout1778.subsystems

import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.chillout1778.Robot
import org.chillout1778.commands.ZeroArmCommand
import org.chillout1778.commands.ZeroElevatorCommand
import org.chillout1778.commands.ZeroIntakeCommand

object Superstructure: SubsystemBase() {
    enum class State(val elevator: Elevator.State,
                     val armPivot: Arm.PivotState,
                     val armRollers: Arm.RollerState,
                     val intakePivot: Intake.PivotState = Intake.PivotState.OperatorControl,
                     val intakeRollers: Intake.RollerState = Intake.RollerState.OperatorControl)
    {
        StartPosition(
            Elevator.State.Down,
            Arm.PivotState.Up, Arm.RollerState.SlowIdle,
            Intake.PivotState.Up, Intake.RollerState.Off),
        Rest(
            Elevator.State.PreHandoff,
            Arm.PivotState.Down, Arm.RollerState.Idle),
        PrePopsiclePickup(
            Elevator.State.PreHandoff,
            Arm.PivotState.PopsiclePickup, Arm.RollerState.In,
            Intake.PivotState.Down, Intake.RollerState.Off
        ),
        PopsiclePickup(
            Elevator.State.PopsiclePickup,
            Arm.PivotState.PopsiclePickup, Arm.RollerState.In,
            Intake.PivotState.Down, Intake.RollerState.Off
        ),
        ArmSourceIntake(
            Elevator.State.Down,
            Arm.PivotState.Up, Arm.RollerState.In),
        SourceIntake(
            Elevator.State.SourceIntake,
            Arm.PivotState.Down, Arm.RollerState.Idle,
            Intake.PivotState.Up, Intake.RollerState.In
        ),
        PreHandoff(
            Elevator.State.Handoff,
            Arm.PivotState.Down, Arm.RollerState.In,
            Intake.PivotState.Up, Intake.RollerState.In),
        Handoff(
            Elevator.State.Handoff,
            Arm.PivotState.Down, Arm.RollerState.In,
            Intake.PivotState.Up, Intake.RollerState.Out),
        PreScore(
            Elevator.State.PreScore,
            Arm.PivotState.Up, Arm.RollerState.Idle,
            Intake.PivotState.Up, Intake.RollerState.Off),
        ReverseHandoff( // TODO: with flexible-intake, this ought to have a PreReverseHandoff or a transition that ensures intake is up
            Elevator.State.PreHandoff,
            Arm.PivotState.Down, Arm.RollerState.Out,
            Intake.PivotState.Up, Intake.RollerState.In),
        PreTrough(
            Elevator.State.Trough,
            Arm.PivotState.Down, Arm.RollerState.Idle,
            Intake.PivotState.Trough, Intake.RollerState.Off),
        Trough(
            Elevator.State.Trough,
            Arm.PivotState.Down, Arm.RollerState.Idle,
            Intake.PivotState.Trough, Intake.RollerState.TroughOut),

        //region L4 Score sequence
        PrepareL4(
            Elevator.State.L4,
            Arm.PivotState.AboveScoreCoral, Arm.RollerState.Idle),
        StartL4(
            Elevator.State.L4,
            Arm.PivotState.L4ScoreCoral, Arm.RollerState.SlowIdle),
        PlaceL4(
            Elevator.State.ScoreL4,
            Arm.PivotState.L4FinishScoreCoral, Arm.RollerState.Off),
        AfterL4(
            Elevator.State.PreHandoff,
            Arm.PivotState.Down, Arm.RollerState.SlowOut),
        //endregion

        //region L3 Score sequence
        PrepareL3(
            Elevator.State.L3,
            Arm.PivotState.AboveScoreCoral, Arm.RollerState.Idle),
        StartL3(
            Elevator.State.L3,
            Arm.PivotState.ScoreCoral, Arm.RollerState.SlowIdle),
        PlaceL3(
            Elevator.State.ScoreL3,
            Arm.PivotState.FinishScoreCoral, Arm.RollerState.Off),
        AfterL3(
            Elevator.State.PostL3,
            Arm.PivotState.Up, Arm.RollerState.SlowOut),
        //endregion

        //region L2 Score sequence
        PrepareL2(
            Elevator.State.L2,
            Arm.PivotState.AboveScoreCoral, Arm.RollerState.Idle),
        StartL2(
            Elevator.State.L2,
            Arm.PivotState.ScoreCoral, Arm.RollerState.SlowIdle),
        PlaceL2(
            Elevator.State.ScoreL2,
            Arm.PivotState.FinishScoreCoral, Arm.RollerState.SlowOut, // Rollers start moving here
        ),
        AfterL2(
            Elevator.State.PostL2,
            Arm.PivotState.Up, Arm.RollerState.Out),
        //endregion

        //region Algae knocking states
        PreGetAlgae(
            Elevator.State.HighAlgae,
            Arm.PivotState.SafeInsideRobotAngle, Arm.RollerState.In),
        GetAlgae(
            Elevator.State.AutoAlgae,
            Arm.PivotState.GetAlgae,  Arm.RollerState.In),
        PostGetAlgae(
            Elevator.State.AutoAlgae,
            Arm.PivotState.PostAlgae, Arm.RollerState.AlgaeIdle),
        AlgaeRest(
            Elevator.State.AlgaeRest,
            Arm.PivotState.AlgaeUp, Arm.RollerState.AlgaeIdle),
        PreBarge(
            Elevator.State.Barge,
            Arm.PivotState.PreBarge, Arm.RollerState.AlgaeIdle),
        ScoreBarge(
            Elevator.State.Barge,
            Arm.PivotState.BargeScore, Arm.RollerState.Out),
        //endregion
        AlgaeDescore(
            Elevator.State.AutoAlgae,
            Arm.PivotState.DescoreAlgae, Arm.RollerState.Descore),
        AlgaeExit(
            Elevator.State.PreHandoff,
            Arm.PivotState.Down, Arm.RollerState.Out),
        PreProcessor(
            Elevator.State.Processor,
            Arm.PivotState.Processor, Arm.RollerState.AlgaeIdle),
        ScoreProcessor(
            Elevator.State.Processor,
            Arm.PivotState.Processor, Arm.RollerState.SlowOut,
        ),
        PreAlgaeGroundIntake(
            Rest.elevator,
            Arm.PivotState.AlgaeGroundPickup, Arm.RollerState.Off,
            Intake.PivotState.Down, Intake.RollerState.Off
        ),
        AlgaeGroundIntake(
            Elevator.State.GroundAlgaeIntake,
            Arm.PivotState.AlgaeGroundPickup, Arm.RollerState.In,
            Intake.PivotState.Down, Intake.RollerState.Off
        ),
        ExitAlgaeGroundIntake(
            Elevator.State.PreHandoff,
            Arm.PivotState.ExitAlgaeGroundPickup, Arm.RollerState.AlgaeIdle,
            Intake.PivotState.Down, Intake.RollerState.Off
        ),
    }

    enum class ScoringLevel {
        TROUGH,
        L2,
        L3,
        L4
        ;
        val index get() = when (this) {
            TROUGH -> 0
            L2 -> 1
            L3 -> 2
            L4 -> 3
        }
    }

    data class SuperstructureInputs (
        val wantExtend: Boolean = false,
        val wantGroundIntake: Boolean = false,
        val wantArmSourceIntake: Boolean = false,
        val wantSourceIntake: Boolean = false,
        val wantScore: Boolean = false,
        val wantedScoringLevel: ScoringLevel = ScoringLevel.L4,
        val wantGetAlgae: Boolean = false,
        val wantDescoreAlgae: Boolean = false,
        val wantVerticalPickup: Boolean = false,
        val wantResetSuperstructure: Boolean = false,
        val wantScoreProcessor: Boolean = false,
        val wantAlgaeGroundIntake: Boolean = false,
        val wantPopsiclePickup: Boolean = false,
    )

    data class Transition(
        val cur: State,
        val next: State,
        val enterFunction: ()->Unit = {},
        val transitionCheck: ()->Boolean
    )

    private val transitions = listOf(
        // this just makes it so the elevator doesn't move until the operator wants to intake
        Transition(State.StartPosition, State.Rest){ inputs.wantGroundIntake || inputs.wantArmSourceIntake},
        Transition(State.StartPosition, State.PreScore){ Robot.isAutonomous },
        // this is just for auto, probably should be done differently
//        Transition(State.StartPosition, State.PreScore){ Robot.isAutonomous },

        // Arm source intaking, must be here
        Transition(State.Rest, State.ArmSourceIntake) { inputs.wantArmSourceIntake },
        Transition(State.ArmSourceIntake, State.Rest) { !inputs.wantArmSourceIntake || Arm.hasObject },

        // Intake source intaking
        Transition(State.Rest, State.SourceIntake) { inputs.wantSourceIntake },
        Transition(State.SourceIntake, State.Rest) { !inputs.wantSourceIntake || Intake.hasCoral },

        //Trough reverse handoff, must be here
        Transition(State.PreScore, State.Rest) { inputs.wantedScoringLevel == ScoringLevel.TROUGH || !Arm.hasObject },
        Transition(State.Rest, State.ReverseHandoff){ Arm.atSetpoint && Elevator.atSetpoint &&
                                                    inputs.wantedScoringLevel == ScoringLevel.TROUGH && Arm.hasObject && !Intake.hasCoral &&
                                                    Intake.effectivePivotState == Intake.PivotState.Up && Intake.atSetpoint},
                                                // TODO: this might somewhat work in flexible-intake but ideally there would be
                                                //  a PreReverseHandoff state.
        Transition(State.ReverseHandoff, State.Rest) { Intake.hasCoral || inputs.wantResetSuperstructure},

        Transition(State.Rest, State.PreTrough) { inputs.wantExtend && inputs.wantedScoringLevel == ScoringLevel.TROUGH && Elevator.atSetpoint && Arm.atSetpoint },
        Transition(State.PreTrough, State.Trough) { Intake.atSetpoint && inputs.wantScore },
        Transition(State.PreTrough, State.Rest) {!inputs.wantExtend},
        Transition(State.Trough, State.Rest) { !inputs.wantScore }, // Don't check for coral here, can cause issues

        Transition(State.Rest, State.PreHandoff) { Elevator.atSetpoint && Arm.atSetpoint && inputs.wantedScoringLevel != ScoringLevel.TROUGH && Intake.hasCoral},

//        Transition(State.Rest, State.VerticalPickup) { inputs.wantVerticalPickup },
//        Transition(State.VerticalPickup, State.PreScore) { Arm.hasCoral },
        Transition(State.PreHandoff, State.Handoff) { Elevator.atSetpoint && Arm.atSetpoint && Intake.atSetpoint },
        Transition(State.Handoff, State.Rest) { Arm.hasObject },
        Transition(State.Rest, State.PreScore) { Arm.hasObject && inputs.wantedScoringLevel != ScoringLevel.TROUGH },
        Transition(State.Handoff, State.Rest) { inputs.wantResetSuperstructure }, // just in case handoff doesn't work

        Transition(State.PreScore, State.PrepareL4) { inputs.wantExtend && inputs.wantedScoringLevel == ScoringLevel.L4 },
        Transition(State.PreScore, State.PrepareL3) { inputs.wantExtend && inputs.wantedScoringLevel == ScoringLevel.L3 },
        Transition(State.PreScore, State.PrepareL2) { inputs.wantExtend && inputs.wantedScoringLevel == ScoringLevel.L2 },

        *scoringTransitions(ScoringLevel.L4, prepare = State.PrepareL4, start = State.StartL4, place = State.PlaceL4, after = State.AfterL4),
        *scoringTransitions(ScoringLevel.L3, prepare = State.PrepareL3, start = State.StartL3, place = State.PlaceL3, after = State.AfterL3),
        *scoringTransitions(ScoringLevel.L2, prepare = State.PrepareL2, start = State.StartL2, place = State.PlaceL2, after = State.AfterL2),

        // Algae Removal
        Transition(State.AlgaeExit, State.PreGetAlgae) { inputs.wantGetAlgae && !Arm.hasObject },
        Transition(State.Rest, State.PreGetAlgae){ inputs.wantGetAlgae && !Arm.hasObject},
        Transition(State.PreGetAlgae, State.Rest) {!inputs.wantGetAlgae },

        Transition(State.PreGetAlgae, State.GetAlgae) { Elevator.atSetpoint },
        Transition(State.GetAlgae, State.PreGetAlgae) {!inputs.wantGetAlgae},//??

        Transition(State.GetAlgae, State.PostGetAlgae) { Arm.hasObject },
        Transition(State.PostGetAlgae, State.AlgaeRest) { Arm.atSetpoint && Arm.atSafeReefDistance() },

        Transition(State.AlgaeRest, State.AlgaeExit) { !Arm.hasObject },
        Transition(State.AlgaeExit, State.Rest) { Arm.atSetpoint && Elevator.atSetpoint },

        Transition(State.AlgaeRest, State.PreBarge){inputs.wantExtend},
        Transition(State.PreBarge, State.AlgaeRest){!inputs.wantExtend},

        Transition(State.PreBarge, State.ScoreBarge){ inputs.wantScore && Swerve.atGoodScoringDistance},
        Transition(State.ScoreBarge, State.PreBarge) {(!inputs.wantExtend || !Arm.hasObject) && Arm.atSafeBargeDistance() },

        Transition(State.Rest, State.AlgaeDescore){ inputs.wantDescoreAlgae },
        Transition(State.AlgaeDescore, State.Rest) {!inputs.wantDescoreAlgae},

        Transition (State.AlgaeRest, State.PreProcessor) { inputs.wantScoreProcessor },
        Transition (State.PreProcessor, State.ScoreProcessor) { inputs.wantScore},

        Transition (State.ScoreProcessor, State.AlgaeRest) { !inputs.wantScoreProcessor && !Arm.hasObject && Arm.atSafeProcessorDistance() },
        Transition (State.PreProcessor, State.AlgaeRest) { !inputs.wantScoreProcessor && Arm.atSafeProcessorDistance() },

        Transition (State.Rest, State.PreAlgaeGroundIntake) { inputs.wantAlgaeGroundIntake && !Arm.hasObject},
        Transition (State.PreAlgaeGroundIntake, State.AlgaeGroundIntake) { inputs.wantAlgaeGroundIntake  && Intake.atSetpoint},
        Transition (State.AlgaeGroundIntake, State.ExitAlgaeGroundIntake) {!inputs.wantAlgaeGroundIntake || Arm.hasObject},
        Transition (State.ExitAlgaeGroundIntake, State.AlgaeRest) { Elevator.atSetpoint && Arm.atSetpoint},


        Transition(State.PopsiclePickup, State.PrePopsiclePickup) { !inputs.wantPopsiclePickup ||
                (Robot.isAutonomous && stateTimer.hasElapsed(popsicleDelay)) },
        Transition(State.PrePopsiclePickup, State.Rest) {!inputs.wantPopsiclePickup},

        Transition (State.Rest, State.PrePopsiclePickup) { inputs.wantPopsiclePickup && !Arm.hasObject},
        Transition (State.PrePopsiclePickup, State.PopsiclePickup) { inputs.wantPopsiclePickup && Intake.atSetpoint},
        Transition (State.PopsiclePickup, State.PreScore) { stateTimer.hasElapsed(popsicleDelay) && Arm.hasObject },
    )

    const val popsicleDelay = 0.5

    fun scoringTransitions(scoringLevel: ScoringLevel,  prepare: State,
                           start: State,  place: State,  after: State): Array<Transition>
    {
        return arrayOf(
            // exit transitions (have to be first)
            Transition(prepare, State.PreScore) {Arm.atSetpoint && (!Arm.hasObject || !inputs.wantExtend || inputs.wantedScoringLevel != scoringLevel)},
            Transition(start, prepare) { !inputs.wantExtend || inputs.wantedScoringLevel != scoringLevel || !Arm.hasObject},
            // normal transitions
            Transition(prepare, start) {Elevator.lazierAtSetpoint && Arm.hasObject && inputs.wantExtend && inputs.wantedScoringLevel == scoringLevel},
            Transition(start, place, enterFunction = {Swerve.markPoseScored()}) { Elevator.atSetpoint && Arm.atSetpoint && inputs.wantScore },
            Transition(place, after) { Elevator.atSetpoint && Arm.atSetpoint && if(place == State.PlaceL2 || place == State.PlaceL3) Arm.atSafePlacementDistance() else true},
            Transition(after, State.Rest) { Arm.insideFrame }
        )
    }

    fun makeZeroAllSubsystemsCommand() = ParallelCommandGroup(
        ZeroIntakeCommand(),
        ZeroElevatorCommand(),
        ZeroArmCommand()
    )

    var inputs = SuperstructureInputs()
//        set(i) { field = i; println("new inputs: $i")}

    fun emptyInputs() {
        inputs = SuperstructureInputs()
    }

    val stateTimer = Timer()

    var state = State.StartPosition
        set(s) {if (s != field) stateTimer.restart(); field = s}

    override fun periodic() {
        if (!Elevator.isZeroed || !Intake.isZeroed || !Arm.isZeroed) {
            return
        }
        stateTimer.start() // ensure timer is running; does nothing if already started

        for (transition in transitions) {
            if (transition.cur == state && transition.transitionCheck()) {
                state = transition.next
                transition.enterFunction()
                setStates()
                return
            }
        }
    }

    fun setStates() {
        // Separate this out so we can use it from other commands
        // to restore control to the superstructure
        Arm.setState(state.armPivot, state.armRollers)
        Elevator.state = state.elevator
        Intake.setState(state.intakePivot, state.intakeRollers)
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.addStringProperty("State", {state.toString()}, {})
        builder.addBooleanProperty("wantExtend", {inputs.wantExtend}, {})
        builder.addBooleanProperty("wantScore", {inputs.wantScore}, {})
        builder.addStringProperty("scoringLevel", {inputs.wantedScoringLevel.toString()}, {})
    }
}

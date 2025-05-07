package org.chillout1778

import choreo.Choreo
import choreo.trajectory.SwerveSample
import choreo.trajectory.Trajectory
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import org.chillout1778.commands.AutoRunnerCommand
import org.chillout1778.commands.TeleopDriveCommand
import org.chillout1778.commands.TeleopSuperstructureCommand
import org.chillout1778.subsystems.*
import kotlin.system.measureTimeMillis

object Robot : TimedRobot() {
    val isRedAlliance get() =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red

    val isOnRedSide get() =
        Swerve.estimatedPose.x > (Constants.Field.FIELD_X_SIZE / 2)

    val enableCoastModeSwitch = DigitalInput(Constants.DioIds.DISABLE_BREAK_MODE)

    var wasEnabledThenDisabled = false
    var wasEnabled = false

    private val autoChooser: SendableChooser<Trajectory<SwerveSample>> = SendableChooser()

    init {
        // This code tells FMS that we use Kotlin (so that we are part of the
        // end-of-year statistics on which languages people use).
        HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin, 0, WPILibVersion.Version)

        // Start logging NetworkTables changes to a USB drive or the RoboRIO.
        DataLogManager.start()

        // Initialize subsystem objects just by referencing them
        Arm
//        AutoContainer
        Elevator
        Intake
        Superstructure
        Swerve
        Vision
        Lights

        Shuffleboard.getTab("Subsystems").add(Arm)
        Shuffleboard.getTab("Subsystems").add(Elevator)
        Shuffleboard.getTab("Subsystems").add(Intake)
        Shuffleboard.getTab("Subsystems").add(Superstructure)
        Shuffleboard.getTab("Subsystems").add(Vision)

        for(trajectoryName in Choreo.availableTrajectories().filterNot{it == "VariablePoses"}) {
            autoChooser.addOption(trajectoryName, Choreo.loadTrajectory<SwerveSample>(trajectoryName).get())
        }


        autoChooser.onChange { t ->
            autoTrajectory = t
            initializeAutonomousCommand()
        }

        Shuffleboard.getTab("Robot").add(autoChooser)

//        DriverStation.silenceJoystickConnectionWarning(true)
    }

    var tickNumber: Long = 0
    override fun robotPeriodic() {
        tickNumber++
        CommandScheduler.getInstance().run()
    }

    var wasCoastModeEnabled: Boolean = false

    override fun disabledInit() {
        if (wasEnabled) wasEnabledThenDisabled = true
        wasEnabled = false
    }
    override fun disabledPeriodic() {
        val pressed = !enableCoastModeSwitch.get()
        if (!wasCoastModeEnabled && pressed) { // rising edge
            Elevator.setCoastEnabled(true)
            Arm.setCoastEnabled(true)
            wasCoastModeEnabled = true
        } else if (wasCoastModeEnabled && !pressed) { // falling edge
            Elevator.setCoastEnabled(false)
            Arm.setCoastEnabled(false)
            wasCoastModeEnabled = false
        }
    }

    override fun disabledExit() {
        Elevator.setCoastEnabled(false)
        Arm.setCoastEnabled(false)
        wasEnabled = true
        wasEnabledThenDisabled = false
    }
    // This is code for running autonomous Commands only in auto mode
    private var didAutoRun = false
    private lateinit var autoTrajectory: Trajectory<SwerveSample>

    private var autonomousCommand: Command = InstantCommand()
    private fun initializeAutonomousCommand() {
        autonomousCommand = Superstructure.makeZeroAllSubsystemsCommand().andThen( // EXTREMELY IMPORTANT TO ZERO
            AutoRunnerCommand(autoTrajectory)
        ).andThen(TeleopDriveCommand(Controls::emptyInputs))
//        Shuffleboard.getTab("Robot").add("Auto runner command", runnerSubCommand)
    }

    override fun autonomousInit() {
        Arm.hasObject = true
        Arm.autoTimer.reset()
        Arm.autoTimer.start()
        didAutoRun = true
        autonomousCommand.schedule()
    }

    override fun autonomousExit() {
        autonomousCommand.cancel()
    }

    override fun teleopInit() {
        if (!didAutoRun)
            Swerve.gyroAngle = if(isRedAlliance) Math.PI else 0.0
        Superstructure.makeZeroAllSubsystemsCommand().schedule()
        Swerve.defaultCommand = TeleopDriveCommand(Controls::driverInputs)
        Superstructure.defaultCommand = TeleopSuperstructureCommand()
    }


    override fun teleopExit() {
        Swerve.removeDefaultCommand()
        Superstructure.removeDefaultCommand()
    }

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
        Superstructure.makeZeroAllSubsystemsCommand().schedule()
        Swerve.gyroAngle = 0.0
    }
}

package org.firstinspires.ftc.teamcode.pedroPathing.trash

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.Intake
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.Outtake
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "Hail Mary", group = "Auton")
@Configurable
class AutoHailMary : NextFTCOpMode() {

    private var pathTimer: Timer? = null
    private var actionTimer: Timer? = null
    private var opmodeTimer: Timer? = null


    init {
        addComponents(
            SubsystemComponent(Intake, Outtake)
        )
        actionTimer = Timer()
        pathTimer = Timer()
        opmodeTimer = Timer().also { it.resetTimer() }
    }

    enum class AutonPath {
        RobotShoot1,
        RobotIntake1,
        RobotShoot2,
        RobotIntake2,
        RobotShoot3,
        RobotIntake3,
        RobotShoot4,
        RobotIntake4,
        RobotShoot5,
        EndAuton
    }

    private var pathState: AutonPath = AutonPath.RobotShoot1

    private fun setPathState(pState: AutonPath) {
        pathState = pState
        pathTimer?.resetTimer()
        actionTimer?.resetTimer()
    }

    private var follower: Follower? = null

    // Positions (from your original auton)
    private val startPose = Pose(33.0, 136.0, Math.toRadians(180.0))
    private val intake1stLinePos = Pose(8.0, 61.5)
    private val intake1ControlPointPos = Pose(73.0, 52.0)

    private val intake2ndLinePos = Pose(15.0, 85.5)
    private val intake2ControlPointPos = Pose(45.0, 87.0)
    private val intake2FirstBallPos = Pose(27.0, intake2ndLinePos.y)

    private val intake3rdLinePos = Pose(8.0, 36.0)
    private val intake3ControlPointPos = Pose(77.0, 33.0)
    private val intake3FirstBallPos = Pose(27.0, intake3rdLinePos.y)

    private val intake4thLinePos = Pose(11.0, 11.0, Math.toRadians(200.0))
    private val intake4ControlPointPos = Pose(10.0, 67.0)
    private val intake4FirstBallPos = Pose(27.0, intake4thLinePos.y)

    private val shootingPose = Pose(58.0, 80.0, Math.toRadians(180.0))
    private val endPose = Pose(50.0, 80.0)

    companion object {
        private const val toleranceIntakeMagSeq = 5.0
        private var magBallHitDelay = 1.0

        // Powers—sync with subsystems if they expose getters, else defaults
        var intakeMaxPower = 1.0
        var shootReturnPower = 1.0

        private var delayAfterIntake = 0.4
        private var intakeEndPosTolerance = 2.0
        private var shootingPoseTolerance = 2.0
    }

    // Paths
    private var robotShootPreload: PathChain? = null
    private var robotIntake1: PathChain? = null
    private var robotGoToShoot1: PathChain? = null
    private var robotIntake2: PathChain? = null
    private var robotGoToShoot2: PathChain? = null
    private var robotIntake3: PathChain? = null
    private var robotGoToShoot3: PathChain? = null
    private var robotIntake4: PathChain? = null
    private var robotGoToShoot4: PathChain? = null

    // Flags (from your original)
    private var pathF1 = true
    private var pathF2 = true
    private var pathF3 = true
    private var pathF4 = true
    private var pathF5 = true

    private var pusherSetUp1 = true
    private var pusherSetUp2 = true
    private var pusherSetUp3 = true
    private var pusherSetUp4 = true
    private var pusherSetUp5 = true

    private var magSeqReady1 = true
    private var magSeqReady2 = true
    private var magSeqReady3 = true
    private var magSeqReady4 = true
    private var magSeqReady5 = true

    private var intakeReached1 = true
    private var intakeReached2 = true
    private var intakeReached3 = true
    private var intakeReached4 = true

    private var intakeDone1 = true
    private var intakeDone2 = true
    private var intakeDone3 = true
    private var intakeDone4 = true

    // Command sequences you provided (as Kotlin Commands)
    private val prepToShootFT: Command
        get() = SequentialGroup(
            Intake.reverseIntake,
            Delay(1.seconds),
            ParallelGroup(
                Outtake.flywheelOn,
                Delay(1.seconds),
                Intake.runIntake
            ),
            Delay(500.milliseconds),
            ParallelGroup(
                Intake.stopIntake,
                Outtake.flywheelOff
            )
        )

    private val preIntakeLine: Command
        get() = SequentialGroup(
            Intake.runIntake,
            Delay(3.seconds),
            Intake.reverseIntake,
            Delay(1.seconds),
            ParallelGroup(
                Intake.stopIntake,
                Outtake.flywheelOn
            )
        )

    private val shoot: Command
        get() = SequentialGroup(
            Intake.runIntake,
            Delay(5.seconds),
            ParallelGroup(
                Intake.stopIntake,
                Outtake.flywheelOff
            )
        )

    private fun buildPaths() {
        robotShootPreload = follower!!.pathBuilder()
            .addPath(BezierLine(startPose, shootingPose))
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()

        robotIntake1 = follower!!.pathBuilder()
            .addPath(BezierCurve(shootingPose, intake1ControlPointPos, intake1stLinePos))
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()

        robotGoToShoot1 = follower!!.pathBuilder()
            .addPath(BezierLine(intake1stLinePos, shootingPose))
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()

        robotIntake2 = follower!!.pathBuilder()
            .addPath(BezierCurve(shootingPose, intake2ControlPointPos, intake2ndLinePos))
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()

        robotGoToShoot2 = follower!!.pathBuilder()
            .addPath(BezierLine(intake2ndLinePos, shootingPose))
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()

        robotIntake3 = follower!!.pathBuilder()
            .addPath(BezierCurve(shootingPose, intake3ControlPointPos, intake3rdLinePos))
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()

        robotGoToShoot3 = follower!!.pathBuilder()
            .addPath(BezierLine(intake3rdLinePos, shootingPose))
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()

        robotIntake4 = follower!!.pathBuilder()
            .addPath(BezierCurve(shootingPose, intake4ControlPointPos, intake4thLinePos))
            .setLinearHeadingInterpolation(shootingPose.heading, intake4thLinePos.heading)
            .build()

        robotGoToShoot4 = follower!!.pathBuilder()
            .addPath(BezierLine(intake4thLinePos, endPose))
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
    }

    private fun autonomousPathUpdate() {
        when (pathState) {
            AutonPath.RobotShoot1 -> {
                follower!!.setMaxPower(shootReturnPower)
                if (pathF1) {
                    follower!!.followPath(robotShootPreload!!)
                    pathF1 = false
                }
                if (follower!!.atPose(shootingPose, shootingPoseTolerance, shootingPoseTolerance)) {
                    if (pusherSetUp1) {
                        pusherSetUp1 = false
                        actionTimer!!.resetTimer()
                        // Schedule your provided sequence at arrival
                        prepToShootFT.schedule()
                    }
                    PanelsTelemetry.telemetry.addData("actionTimer", actionTimer!!.elapsedTimeSeconds)
                    PanelsTelemetry.telemetry.addData("pathTimer", pathTimer!!.elapsedTimeSeconds)
                    // Advance after sequence finishes (~1 + 1 + 0.5 = 2.5s + small buffer)
                    if (actionTimer!!.elapsedTimeSeconds >= 2.6) {
                        setPathState(AutonPath.RobotIntake1)
                    }
                }
            }

            AutonPath.RobotIntake1 -> if (!follower!!.isBusy) {
                follower!!.setMaxPower(intakeMaxPower)
                if (intakeReached1) {
                    follower!!.followPath(robotIntake1!!)
                    intakeReached1 = false

                    // Run your preIntakeLine sequence during approach
                    preIntakeLine.schedule()
                    actionTimer!!.resetTimer()
                }
                if (follower!!.atPose(intake1stLinePos, intakeEndPosTolerance, intakeEndPosTolerance)) {
                    if (intakeDone1) {
                        actionTimer!!.resetTimer()
                        intakeDone1 = false
                    }
                }
                if (!intakeDone1 && actionTimer!!.elapsedTimeSeconds >= delayAfterIntake) {
                    setPathState(AutonPath.RobotShoot2)
                }
            }

            AutonPath.RobotShoot2 -> if (!follower!!.isBusy) {
                follower!!.setMaxPower(shootReturnPower)
                if (pathF2) {
                    follower!!.followPath(robotGoToShoot1!!)
                    pathF2 = false
                }
                if (follower!!.atPose(shootingPose, shootingPoseTolerance, shootingPoseTolerance)) {
                    if (pusherSetUp2) {
                        pusherSetUp2 = false
                        actionTimer!!.resetTimer()
                        // Execute your Shoot sequence at arrival
                        shoot.schedule()
                    }
                    PanelsTelemetry.telemetry.addData("actionTimer", actionTimer!!.elapsedTimeSeconds)
                    PanelsTelemetry.telemetry.addData("pathTimer", pathTimer!!.elapsedTimeSeconds)
                    // Advance after shoot sequence (~5.0s + buffer)
                    if (actionTimer!!.elapsedTimeSeconds >= 5.2) {
                        setPathState(AutonPath.RobotIntake2)
                    }
                }
            }

            AutonPath.RobotIntake2 -> if (!follower!!.isBusy) {
                follower!!.setMaxPower(intakeMaxPower)
                if (intakeReached2) {
                    follower!!.followPath(robotIntake2!!)
                    intakeReached2 = false
                }
                if (follower!!.atPose(intake2ndLinePos, intakeEndPosTolerance, intakeEndPosTolerance)) {
                    if (intakeDone2) {
                        actionTimer!!.resetTimer()
                        intakeDone2 = false
                    }
                }
                if (!intakeDone2 && actionTimer!!.elapsedTimeSeconds >= delayAfterIntake) {
                    setPathState(AutonPath.RobotShoot3)
                }
            }

            AutonPath.RobotShoot3 -> if (!follower!!.isBusy) {
                follower!!.setMaxPower(shootReturnPower)
                if (follower!!.atPose(intake2FirstBallPos, toleranceIntakeMagSeq, toleranceIntakeMagSeq)) {
                    if (magSeqReady2) {
                        magSeqReady2 = false
                        actionTimer!!.resetTimer()
                    }
                }
                if (pathF3) {
                    follower!!.followPath(robotGoToShoot2!!)
                    pathF3 = false
                }
                if (follower!!.atPose(shootingPose, shootingPoseTolerance, shootingPoseTolerance)) {
                    if (pusherSetUp3) {
                        pusherSetUp3 = false
                        actionTimer!!.resetTimer()
                        // Optional: reuse Shoot sequence here
                        shoot.schedule()
                    }
                    PanelsTelemetry.telemetry.addData("actionTimer", actionTimer!!.elapsedTimeSeconds)
                    PanelsTelemetry.telemetry.addData("pathTimer", pathTimer!!.elapsedTimeSeconds)
                    if (actionTimer!!.elapsedTimeSeconds >= 5.2) {
                        setPathState(AutonPath.RobotIntake3)
                    }
                }
            }

            AutonPath.RobotIntake3 -> if (!follower!!.isBusy) {
                follower!!.setMaxPower(intakeMaxPower)
                if (intakeReached3) {
                    follower!!.followPath(robotIntake3!!)
                    intakeReached3 = false
                }
                if (follower!!.atPose(intake3rdLinePos, intakeEndPosTolerance, intakeEndPosTolerance)) {
                    if (intakeDone3) {
                        actionTimer!!.resetTimer()
                        intakeDone3 = false
                    }
                }
                if (!intakeDone3 && actionTimer!!.elapsedTimeSeconds >= delayAfterIntake) {
                    setPathState(AutonPath.RobotShoot4)
                }
            }

            AutonPath.RobotShoot4 -> if (!follower!!.isBusy) {
                follower!!.setMaxPower(shootReturnPower)
                if (follower!!.atPose(intake3FirstBallPos, toleranceIntakeMagSeq, toleranceIntakeMagSeq)) {
                    if (magSeqReady3) {
                        magSeqReady3 = false
                        actionTimer!!.resetTimer()
                    }
                }
                if (pathF4) {
                    follower!!.followPath(robotGoToShoot3!!)
                    pathF4 = false
                }
                if (follower!!.atPose(shootingPose, shootingPoseTolerance, shootingPoseTolerance)) {
                    if (pusherSetUp4) {
                        pusherSetUp4 = false
                        actionTimer!!.resetTimer()
                        // Optional: reuse Shoot sequence here
                        shoot.schedule()
                    }
                    PanelsTelemetry.telemetry.addData("actionTimer", actionTimer!!.elapsedTimeSeconds)
                    PanelsTelemetry.telemetry.addData("pathTimer", pathTimer!!.elapsedTimeSeconds)
                    if (actionTimer!!.elapsedTimeSeconds >= 5.2) {
                        setPathState(AutonPath.EndAuton)
                    }
                }
                if (opmodeTimer!!.elapsedTimeSeconds >= 29.5) {
                    setPathState(AutonPath.EndAuton)
                }
            }

            AutonPath.RobotIntake4 -> if (!follower!!.isBusy) {
                follower!!.setMaxPower(intakeMaxPower)
                if (intakeReached4) {
                    follower!!.followPath(robotIntake4!!)
                    intakeReached4 = false
                }
                if (follower!!.atPose(intake4thLinePos, 0.15, 0.15)) {
                    if (intakeDone4) {
                        actionTimer!!.resetTimer()
                        intakeDone4 = false
                    }
                    if (actionTimer!!.elapsedTimeSeconds >= delayAfterIntake) {
                        setPathState(AutonPath.RobotShoot5)
                    }
                }
            }

            AutonPath.RobotShoot5 -> if (!follower!!.isBusy) {
                follower!!.setMaxPower(shootReturnPower)
                if (follower!!.atPose(intake4FirstBallPos, toleranceIntakeMagSeq, toleranceIntakeMagSeq)) {
                    if (magSeqReady4) {
                        magSeqReady4 = false
                        actionTimer!!.resetTimer()
                    }
                }
                if (pathF5) {
                    follower!!.followPath(robotGoToShoot4!!)
                    pathF5 = false
                }
                if (follower!!.atPose(shootingPose, shootingPoseTolerance, shootingPoseTolerance)) {
                    if (pusherSetUp5) {
                        pusherSetUp5 = false
                        actionTimer!!.resetTimer()
                        // Optional: final Shoot sequence
                        shoot.schedule()
                    }
                    PanelsTelemetry.telemetry.addData("actionTimer", actionTimer!!.elapsedTimeSeconds)
                    PanelsTelemetry.telemetry.addData("pathTimer", pathTimer!!.elapsedTimeSeconds)
                    if (actionTimer!!.elapsedTimeSeconds >= 5.2) {
                        setPathState(AutonPath.EndAuton)
                    }
                }
            }

            AutonPath.EndAuton -> if (!follower!!.isBusy) {
                follower!!.followPath(robotGoToShoot4!!)
            }
        }
    }

    override fun onUpdate() {
        follower!!.update()
        autonomousPathUpdate()

        telemetry.addData("path state", pathState)
        telemetry.addData("x", follower!!.pose.x)
        telemetry.addData("y", follower!!.pose.y)
        telemetry.addData("heading", follower!!.pose.heading)
        telemetry.update()

        PanelsTelemetry.telemetry.update()
    }

    override fun onInit() {
        follower = Constants.createFollower(hardwareMap)
        buildPaths()
        follower!!.setStartingPose(startPose)

        // Pull caps from subsystems if they expose getters
        runCatching { intakeMaxPower = Intake.maxIntakePower }.getOrElse { /* keep default */ }
        runCatching { shootReturnPower = Outtake.returnDrivePower }.getOrElse { /* keep default */ }
    }

    override fun onWaitForStart() {}

    override fun onStartButtonPressed() {
        // Safe outtake at start
        Outtake.flywheelOff.schedule()
        opmodeTimer!!.resetTimer()
        actionTimer!!.resetTimer()
        setPathState(AutonPath.RobotShoot1)
    }

    override fun onStop() {
        Intake.stopIntake.schedule()
        Outtake.flywheelOff.schedule()
    }
}
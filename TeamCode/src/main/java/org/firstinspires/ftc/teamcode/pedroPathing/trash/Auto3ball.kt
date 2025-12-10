package org.firstinspires.ftc.teamcode.pedroPathing.trash

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.Intake
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.Outtake
import kotlin.time.Duration.Companion.seconds
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@Autonomous
class AutoBackwardShoot : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(Outtake, Intake),
            PedroComponent(Constants::createFollower),
            BulkReadComponent,
            BindingsComponent
        )
    }

    var fl = MotorEx("fl")
    var fR = MotorEx("fr")
    var bR = MotorEx("br")
    var bL = MotorEx("bl")
    val drive = MotorGroup(fl, fR, bR, bL)

    private val START_POSE = Pose(20.0, 123.0, Math.toRadians(144.0))
    private val BACKWARD_POSE = Pose(38.5, 104.0, Math.toRadians(144.0))  // Move backward (increase Y)
    private val END_POSE = Pose(54.0, 60.0, Math.toRadians(180.0))

    private val shootSequence: Command
        get() = SequentialGroup(
            // Turn on flywheel
            Outtake.flywheelOn,
            // Wait for flywheel to spin up
            Delay(1.5.seconds),
            // Run intake to feed balls into shooter
            Intake.runIntake,
            // Shoot for 3 seconds
            Delay(3.seconds),
            // Stop everything
            ParallelGroup(
                Intake.stopIntake,
                Outtake.flywheelOff
            )
        )

    private val driveBackwardPath: Command
        get() = InstantCommand {
            val path = PedroComponent.follower
                .pathBuilder()
                .addPath(BezierLine(START_POSE, BACKWARD_POSE))
                .setLinearHeadingInterpolation(START_POSE.heading, BACKWARD_POSE.heading)
                .build()
            PedroComponent.follower.followPath(path)
        }

    private val moveRightPath: Command
        get() = InstantCommand {
            val path = PedroComponent.follower
                .pathBuilder()
                .addPath(BezierLine(BACKWARD_POSE, END_POSE))
                .setLinearHeadingInterpolation(BACKWARD_POSE.heading, END_POSE.heading)
                .build()
            PedroComponent.follower.followPath(path)
        }

    override fun onStartButtonPressed() {
        SequentialGroup(
            // Drive backwards using Pedro for 1 second
            driveBackwardPath,
            Delay(1.seconds),
            // Execute shoot sequence
            shootSequence,
            // Move right for 5 seconds
            moveRightPath,
            Delay(5.seconds)
        ).schedule()
    }
}
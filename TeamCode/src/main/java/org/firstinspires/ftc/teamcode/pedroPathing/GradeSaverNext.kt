package org.firstinspires.ftc.teamcode.pedroPathing


import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.Intake
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.Outtake
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.limeLight
import java.lang.StrictMath.toRadians


@Autonomous(name = "finalAuto2")
class GradeSaverNext : NextFTCOpMode() {


    private var tele = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
    private lateinit var autoPath: AutoPath
    private var index = 0


    init {
        addComponents(
            SubsystemComponent(Outtake, limeLight, Intake),
            PedroComponent(Constants::createFollower),
            BulkReadComponent,
            BindingsComponent
        )
    }


    override fun onInit() {
        follower.setStartingPose(Pose(33.0, 136.0, Math.toRadians(180.0)))
        autoPath = AutoPath()


        tele.run {
            addLine("Status: Initialized")
            addLine("Ready to run autonomous")
            update()
        }
    }


    override fun onStartButtonPressed() {
        // Schedule the first path
        autoPath.next().schedule()
    }


    override fun onUpdate() {
        follower.update()


        tele.run {
            addLine("Path Index: $index / ${autoPath.pathCount}")
            addLine("X: ${"%.2f".format(follower.pose.x)}")
            addLine("Y: ${"%.2f".format(follower.pose.y)}")
            addLine("Heading: ${"%.2f".format(Math.toDegrees(follower.pose.heading))}°")
            addLine()
            update()
        }
    }


    inner class AutoPath {


        // Poses for autonomous
        val startPose = Pose(33.0, 136.0, Math.toRadians(180.0))
        val scorePose = Pose(47.7, 95.9, Math.toRadians(135.0))
        val scorPoseClearCP = Pose(58.3, 54.4)


        val pickUpPose1 = Pose(13.5, 83.71, Math.toRadians(180.0))
        val pickUpPose1CP = Pose(61.8, 82.2)


        val clearOverFlow = Pose(10.0, 74.2, Math.toRadians(90.0))
        val clearOverFlowCp = Pose(56.0, 81.5)


        val pickUpPose2 = Pose(6.25, 59.6, Math.toRadians(180.0))
        val pickUpPose2CP1 = Pose(52.3, 47.0)
        val pickUpPose2CP2 = Pose(51.4, 61.8)


        val pickUpPose3 = Pose(7.0, 36.0, Math.toRadians(180.0))
        val pup3CP1 = Pose(57.0, 23.0)
        val pup3CP2 = Pose(45.1, 36.5)


        val pathCount = 8


        // Path 0: Shoot preload
        val robotShootPreload = SequentialGroup(
            Outtake.flyWheelAuto,
                FollowPath(
                    follower.pathBuilder()
                        .addPath(BezierLine(startPose, scorePose))
                        .setLinearHeadingInterpolation(toRadians(180.0), toRadians(135.0))
                        .build()

            ),
            Intake.intakeSlow,
            Delay(3.0),
            ParallelGroup(
                Intake.stopIntake,
                Outtake.flywheelOff
            ),
            InstantCommand { autoPath.next().schedule() }
        )


        // Path 1: Go to intake position 1
        val robotIntake1 = SequentialGroup(
            Intake.runIntake,
            FollowPath(
                follower.pathBuilder()
                    .addPath(BezierCurve(scorePose, pickUpPose1CP, pickUpPose1))
                    .setConstantHeadingInterpolation(Math.toRadians(180.0))
                    .build()
            ),
            Delay(0.35),
                Intake.stopIntake,
               Intake.reverseIntake,
            Delay(0.2),
            Intake.stopIntake,
            InstantCommand { autoPath.next().schedule() }
        )


        // Path 2: Clear overflow
        val robotClearOverflow = SequentialGroup(
            FollowPath(
                follower.pathBuilder()
                    .addPath(BezierCurve(pickUpPose1, clearOverFlowCp, clearOverFlow))
                    .setLinearHeadingInterpolation(toRadians(180.0), toRadians(90.0))
                    .build()
            ),
            InstantCommand { autoPath.next().schedule() }
        )


        // Path 3: Go to shoot position and shoot
        val robotGoToShoot1 = SequentialGroup(
            ParallelGroup(
                Outtake.flyWheelAuto,
                FollowPath(
                    follower.pathBuilder()
                        .addPath(BezierLine(clearOverFlow, scorePose))
                        .setLinearHeadingInterpolation(toRadians(180.0), toRadians(135.0))
                        .build()
                )
            ),
            Intake.intakeSlow,
            Delay(3.0),
            ParallelGroup(
                Intake.stopIntake,
                Outtake.flywheelOff
            ),
            InstantCommand { autoPath.next().schedule() }
        )


        // Path 4: Go to intake position 2
        val robotIntake2 = SequentialGroup(
            Intake.runIntake,
            FollowPath(
                follower.pathBuilder()
                    .addPath(BezierCurve(scorePose, pickUpPose2CP1, pickUpPose2CP2, pickUpPose2))
                    .setConstantHeadingInterpolation(Math.toRadians(180.0))
                    .build()
            ),
            Delay(0.35),
            Intake.stopIntake,
            Intake.reverseIntake,
            Delay(0.15),
            Intake.stopIntake,
            InstantCommand { autoPath.next().schedule() }
        )


        // Path 5: Go to shoot position and shoot
        val robotGoToShoot2 = SequentialGroup(
            ParallelGroup(
                Outtake.flyWheelAuto,
                FollowPath(
                    follower.pathBuilder()
                        .addPath(BezierCurve(pickUpPose2, scorPoseClearCP, scorePose))
                        .setLinearHeadingInterpolation(toRadians(180.0), toRadians(135.0))
                        .build()
                )
            ),
            Intake.intakeSlow,
            Delay(3.0),
            ParallelGroup(
                Intake.stopIntake,
                Outtake.flywheelOff
            ),
            InstantCommand { autoPath.next().schedule() }
        )


        // Path 6: Go to intake position 3
        val robotIntake3 = SequentialGroup(
            Intake.runIntake,
            FollowPath(
                follower.pathBuilder()
                    .addPath(BezierCurve(scorePose, pup3CP1, pup3CP2, pickUpPose3))
                    .setConstantHeadingInterpolation(Math.toRadians(180.0))
                    .build()
            ),
            Delay(0.35),
            Intake.stopIntake,
            Intake.reverseIntake,
            Delay(0.15),
            Intake.stopIntake,
            InstantCommand { autoPath.next().schedule() }
        )


        // Path 7: Go to shoot position and shoot (final)
        val robotGoToShoot3 = SequentialGroup(
            ParallelGroup(
                Outtake.flyWheelAuto,
                FollowPath(
                    follower.pathBuilder()
                        .addPath(BezierLine(pickUpPose3, scorePose))
                        .setLinearHeadingInterpolation(toRadians(180.0), toRadians(140.0))
                        .build()
                )
            ),
            Intake.intakeSlow,
            Delay(3.0),
            ParallelGroup(
                Intake.stopIntake,
                Outtake.flywheelOff
            )
            // No next() call - this is the final path
        )


        fun next(): SequentialGroup {
            return when (index++) {
                0 -> robotShootPreload
                1 -> robotIntake1
                2 -> robotClearOverflow
                3 -> robotGoToShoot1
                4 -> robotIntake2
                5 -> robotGoToShoot2
                6 -> robotIntake3
                7 -> robotGoToShoot3
                else -> SequentialGroup() // Done
            }
        }
    }
}

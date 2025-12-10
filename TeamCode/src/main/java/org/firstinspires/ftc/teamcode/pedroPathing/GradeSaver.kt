package org.firstinspires.ftc.teamcode.pedroPathing

import android.os.FileUriExposedException
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.Intake
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.Outtake
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.limeLight
import java.lang.StrictMath.toRadians

@Autonomous(name = "finalAuto")
class GradeSaver: NextFTCOpMode() {

    private var pathTimer: Timer? = null
    private var actionTimer: Timer? = null
    private var opmodeTimer: Timer? = null
    init {
        addComponents(SubsystemComponent(Outtake, limeLight, Intake),
            PedroComponent(Constants::createFollower),
            BulkReadComponent,
            BindingsComponent)
        actionTimer = Timer()
        pathTimer = Timer()
        opmodeTimer = Timer().also { it.resetTimer() }
    }
    enum class AutonPath {
        RobotShoot1,
        RobotIntake1,
        RobotClear,
        RobotShoot2,
        RobotIntake2,
        RobotShoot3,
        RobotIntake3,
        RobotShoot4,
        RobotIntake4,
        RobotShoot5,
        EndAuton
    }
    private var pathState = AutonBlueArtifact.AutonPath.RobotShoot1

    fun setPathState(pState: AutonBlueArtifact.AutonPath) {
        pathState = pState
        pathTimer?.resetTimer()
        actionTimer?.resetTimer()

        // Reset per-phase action flags
        // still gotta do that add the components

    }
    private var follower: Follower? = null

    private val startPose = Pose(33.0, 136.0, Math.toRadians(180.0))


    private val ScorePose = Pose(47.7, 95.9, Math.toRadians(135.0))
    private val ScorPoseClearCP = Pose(58.3,54.4)

    private val pickUpPose1 = Pose(14.5,83.71,Math.toRadians(180.0))
    private val pickUpPose1CP = Pose(61.8,82.2)

    private val clearOverFlow = Pose(15.0,74.2,Math.toRadians(90.0))
    private val clearOverFlowCp = Pose(54.0,81.5)

    private val pickUpPose2 = Pose(14.3,59.6,Math.toRadians(180.0))
    private val pickUpPose2CP1 = Pose(52.3,47.0)
    private val pickUpPose2CP2 = Pose(51.4,61.8)

    private val pickUpPose3 = Pose(16.0,36.0, Math.toRadians(180.0))
    private val pup3CP1 = Pose(57.0,23.0)
    private val pup3CP2 = Pose(45.1,36.5)

    private var robotShootPreload: PathChain? = null
    private var robotIntake1: PathChain? = null
    private var robotClearOverflow: PathChain? = null
    private var robotGoToShoot1: PathChain? = null
    private var robotIntake2: PathChain? = null
    private var robotGoToShoot2: PathChain? = null
    private var robotIntake3: PathChain? = null
    private var robotGoToShoot3: PathChain? = null

    private fun buildPaths() {
        robotShootPreload = follower!!.pathBuilder()
            .addPath(BezierLine(startPose, ScorePose))
            .setLinearHeadingInterpolation(toRadians(180.0), toRadians(135.0))
            .build()

        robotIntake1 = follower!!.pathBuilder()
            .addPath(BezierCurve(ScorePose, pickUpPose1CP, pickUpPose1))
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
        robotClearOverflow = follower!!.pathBuilder()
            .addPath(BezierCurve(pickUpPose1, clearOverFlowCp,clearOverFlow))
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()

        robotGoToShoot1 = follower!!.pathBuilder()
            .addPath(BezierLine(clearOverFlow, ScorePose))
            .setLinearHeadingInterpolation(toRadians(180.0), toRadians(135.0))
            .build()


        robotIntake2 = follower!!.pathBuilder()
            .addPath(BezierCurve(ScorePose, pickUpPose2CP1, pickUpPose2CP2,pickUpPose2))
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()

        robotGoToShoot2 = follower!!.pathBuilder()
            .addPath(BezierCurve(pickUpPose2, ScorPoseClearCP,ScorePose))
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()

        robotIntake3 = follower!!.pathBuilder()
            .addPath(BezierCurve(ScorePose, pup3CP1, pup3CP2,pickUpPose3))
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()

        robotGoToShoot3 = follower!!.pathBuilder()
            .addPath(BezierLine(pickUpPose3, ScorePose))
            .setLinearHeadingInterpolation(toRadians(180.0), toRadians(135.0))
            .build()

    }



}
package org.firstinspires.ftc.teamcode.pedroPathing.trash;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "SimpleMoveRight", group = "Autonomous")
@Configurable
public class Auto3ballJava extends OpMode {

    protected Follower follower;
    protected Timer moveTimer;
    int pathState = 0;

    private static final Pose START_POSE = new Pose(21.084, 122.018, Math.toRadians(180));
    private static final Pose END_POSE = new Pose(21.084, 60.0, Math.toRadians(180)); // Move right (decrease Y)

    private TelemetryManager panelsTelemetry;
    private PathChain moveRightPath;

    @Override
    public void init() {
        moveTimer = new Timer();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        // Build the path to move right
        moveRightPath = follower
                .pathBuilder()
                .addPath(new BezierLine(START_POSE, END_POSE))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), END_POSE.getHeading())
                .build();

        panelsTelemetry.debug("Status", "Initialized - Ready to move right");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        moveTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();

        switch (pathState) {
            case 0:
                // Start moving right
                follower.followPath(moveRightPath);
                pathState++;
                break;
            case 1:
                // Keep moving for 5 seconds or until path completes
                if (moveTimer.getElapsedTimeSeconds() >= 5.0) {
                    pathState++;
                }
                break;
            case 2:
                // Stop
                break;
        }

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Time Elapsed", moveTimer.getElapsedTimeSeconds());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }
}
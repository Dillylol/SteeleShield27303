package org.firstinspires.ftc.teamcode.steele27303;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BjornTestAuto (Pose Verify)")
public class BjornTestAuto extends OpMode {

    private Follower follower;
    private PathChain toEndPose;
    private boolean pathStarted = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        // Start at defined Blue Start Pose
        Pose startPose = BjornConstants.Auto.BLUE_AUTO_START_POSE;
        follower.setStartingPose(startPose);

        // Path to the defined BLUE end pose
        // Note: If the end pose is far away, ensure space!
        toEndPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, BjornConstants.Auto.BLUE_AUTO_END_POSE))
                .setLinearHeadingInterpolation(startPose.getHeading(),
                        BjornConstants.Auto.BLUE_AUTO_END_POSE.getHeading())
                .build();

        telemetry.addData("Status", "Init Complete");
        telemetry.addData("Target", BjornConstants.Auto.BLUE_AUTO_END_POSE);
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        if (!pathStarted) {
            // Wait for user to press A to start path
            if (gamepad1.a) {
                follower.followPath(toEndPose);
                pathStarted = true;
            }
            telemetry.addData("Action", "Press A to drive to End Pose");
        } else {
            if (follower.atParametricEnd()) {
                telemetry.addData("Status", "Arrived at End Pose");
            } else {
                telemetry.addData("Status", "Driving...");
            }
        }

        telemetry.addData("Pose", follower.getPose());
        telemetry.update();
    }
}

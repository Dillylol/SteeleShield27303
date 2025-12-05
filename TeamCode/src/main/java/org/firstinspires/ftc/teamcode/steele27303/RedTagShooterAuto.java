package org.firstinspires.ftc.teamcode.steele27303;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.CameraConfig;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera.TagObservation;

@Autonomous(name = "RedTagShooterAuto", group = "Bjorn")
public class RedTagShooterAuto extends BaseTagShooterAuto {

    // ---------------- RED Poses ----------------
    private static final Pose START = pose(0.0, 0.0, -90.0);
    private static final Pose SHOOT_ZONE = pose(0.0, 25.0, -85.0);
    private static final Pose ALIGN1 = pose(-9.4, 32.9, -119.0);
    private static final Pose GRAB1 = pose(-32.9, 16.0, -119.0);
    private static final Pose ALIGN1_BACK = pose(-23.0, 31.0, -119.0);
    private static final Pose PARK = pose(-24.0, 52.0, -35.0);

    private PathChain toShootingSpot;

    @Override
    protected void buildPaths() {
        follower.setStartingPose(START);

        toShootingSpot = follower.pathBuilder()
                .addPath(new BezierLine(START, SHOOT_ZONE))
                .setLinearHeadingInterpolation(START.getHeading(), SHOOT_ZONE.getHeading())
                .build();
    }

    private static Pose pose(double x, double y, double hDeg) {
        return new Pose(x, y, Math.toRadians(hDeg));
    }

    @Override
    protected void startAuto() {
        currentState = AutoState.FOLLOWING_PATH;
        follower.followPath(toShootingSpot);
    }

    @Override
    protected boolean isTargetGoal(TagObservation obs) {
        return obs != null && (obs.id == CameraConfig.RED_GOAL_TAG_ID
                || CameraConfig.CLASS_RED_GOAL.equals(obs.tagClass));
    }

    @Override
    protected void onPathFinished() {
        if (currentState == AutoState.FOLLOWING_PATH) {
            // Arrived at shooting spot
            currentState = AutoState.AIMING_AND_SHOOTING;
        }
    }

    @Override
    protected void onShotFired() {
        // Shot fired, we can park or finish
        currentState = AutoState.FINISHED;
    }
}

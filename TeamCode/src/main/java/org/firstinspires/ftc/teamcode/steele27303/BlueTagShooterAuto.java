package org.firstinspires.ftc.teamcode.steele27303;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.CameraConfig;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera.TagObservation;

@Autonomous(name = "BlueTagShooterAuto", group = "Bjorn")
public class BlueTagShooterAuto extends BaseTagShooterAuto {

    // ---------------- BLUE Poses ----------------
    private static final Pose START = pose(0, 0, 90);
    private static final Pose SHOOT_ZONE = pose(0, 25, -85);
    private static final Pose ALIGN1 = pose(22.6, 37.7, -58);
    private static final Pose GRAB1 = pose(32.9, 16, -58);
    private static final Pose ALIGN1_BACK = pose(23, 31, -58);
    private static final Pose PARK = pose(28.6, 48.7, -145);

    private PathChain toShootingSpot, toAlign1, toGrab1, toAlign1Back, toShoot2;
    private int stage = 0; // 0: Start->Shoot, 1: Shoot->Align1, 2: Align1->Grab1, 3: Grab1->Align1Back, 4: Align1Back->Shoot2

    @Override
    protected void buildPaths() {
        follower.setStartingPose(START);

        toShootingSpot = follower.pathBuilder()
                .addPath(new BezierLine(START, SHOOT_ZONE))
                .setLinearHeadingInterpolation(START.getHeading(), SHOOT_ZONE.getHeading())
                .build();

        toAlign1 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_ZONE, ALIGN1))
                .setLinearHeadingInterpolation(SHOOT_ZONE.getHeading(), ALIGN1.getHeading())
                .build();

        toGrab1 = follower.pathBuilder()
                .addPath(new BezierLine(ALIGN1, GRAB1))
                .setLinearHeadingInterpolation(ALIGN1.getHeading(), GRAB1.getHeading())
                .build();

        toAlign1Back = follower.pathBuilder()
                .addPath(new BezierLine(GRAB1, ALIGN1_BACK))
                .setLinearHeadingInterpolation(GRAB1.getHeading(), ALIGN1_BACK.getHeading())
                .build();

        toShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(ALIGN1_BACK, SHOOT_ZONE))
                .setLinearHeadingInterpolation(ALIGN1_BACK.getHeading(), SHOOT_ZONE.getHeading())
                .build();
    }

    private static Pose pose(double x, double y, double hDeg) {
        return new Pose(x, y, Math.toRadians(hDeg));
    }

    @Override
    protected void startAuto() {
        stage = 0;
        currentState = AutoState.FOLLOWING_PATH;
        follower.followPath(toShootingSpot);
    }

    @Override
    protected boolean isTargetGoal(TagObservation obs) {
        return obs != null && (obs.id == CameraConfig.BLUE_GOAL_TAG_ID
                || CameraConfig.CLASS_BLUE_GOAL.equals(obs.tagClass));
    }

    @Override
    protected void onPathFinished() {
        if (stage == 0) {
            // Arrived at first shooting spot
            currentState = AutoState.AIMING_AND_SHOOTING;
        } else if (stage == 1) {
            // Arrived at ALIGN1 -> Start Intake and go to GRAB1
            startIntake();
            stage = 2;
            follower.followPath(toGrab1, true);
        } else if (stage == 2) {
            // Arrived at GRAB1 -> Go back to ALIGN1_BACK
            stage = 3;
            follower.followPath(toAlign1Back, true);
        } else if (stage == 3) {
            // Arrived at ALIGN1_BACK -> Stop Intake and go to SHOOT
            stopIntake();
            setShooterIdle();
            stage = 4;
            follower.followPath(toShoot2, true);
        } else if (stage == 4) {
            // Arrived at SHOOT2
            currentState = AutoState.AIMING_AND_SHOOTING;
        }
    }

    @Override
    protected void onShotFired() {
        if (stage == 0) {
            // First shot done, go to ALIGN1
            stage = 1;
            currentState = AutoState.FOLLOWING_PATH;
            follower.followPath(toAlign1, true);
        } else {
            // Second shot done, finish
            currentState = AutoState.FINISHED;
        }
    }
}

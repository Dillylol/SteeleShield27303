package org.firstinspires.ftc.teamcode.steele27303;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.CameraConfig;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera.TagObservation;

@Autonomous(name = "RedTagShooterAuto", group = "Bjorn")
public class RedTagShooterAuto extends BaseTagShooterAuto {

    // ---------------- RED Poses ----------------
    private static final Pose START = pose(0, 0, 90);
    private static final Pose SHOOTER = pose(0, -30, 90);

    // Cycle 1
    private static final Pose ROW1 = pose(19.5, -44.5, 55);
    private static final Pose CAPTURE1 = pose(33.5, -24.8, 55);

    // Cycle 2
    private static final Pose ROW2 = pose(39.1, -57.1, 55);
    private static final Pose CAPTURE2 = pose(52.7, -38.1, 55);

    // Cycle 3
    private static final Pose ROW3 = pose(58.2, -71.8, 55);
    private static final Pose CAPTURE3 = pose(71.5, -53.3, 55);

    private PathChain toShooterInitial;

    // Cycle 1 Paths
    private PathChain shooterToRow1, row1ToCapture1, capture1ToRow1, row1ToShooter;

    // Cycle 2 Paths
    private PathChain shooterToRow2, row2ToCapture2, capture2ToRow2, row2ToShooter;

    // Cycle 3 Paths
    private PathChain shooterToRow3, row3ToCapture3, capture3ToRow3, row3ToShooter;

    private int cycleIndex = 0;

    private int stage = 0;

    @Override
    protected void buildPaths() {
        follower.setStartingPose(START);

        // Initial
        toShooterInitial = follower.pathBuilder()
                .addPath(new BezierLine(START, SHOOTER))
                .setLinearHeadingInterpolation(START.getHeading(), SHOOTER.getHeading())
                .build();

        // Cycle 1
        shooterToRow1 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOTER, ROW1))
                .setLinearHeadingInterpolation(SHOOTER.getHeading(), ROW1.getHeading())
                .build();

        row1ToCapture1 = follower.pathBuilder()
                .addPath(new BezierLine(ROW1, CAPTURE1))
                .setLinearHeadingInterpolation(ROW1.getHeading(), CAPTURE1.getHeading())
                .build();

        capture1ToRow1 = follower.pathBuilder()
                .addPath(new BezierLine(CAPTURE1, ROW1))
                .setLinearHeadingInterpolation(CAPTURE1.getHeading(), ROW1.getHeading())
                .build();

        row1ToShooter = follower.pathBuilder()
                .addPath(new BezierLine(ROW1, SHOOTER))
                .setLinearHeadingInterpolation(ROW1.getHeading(), SHOOTER.getHeading())
                .build();

        // Cycle 2
        shooterToRow2 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOTER, ROW2))
                .setLinearHeadingInterpolation(SHOOTER.getHeading(), ROW2.getHeading())
                .build();

        row2ToCapture2 = follower.pathBuilder()
                .addPath(new BezierLine(ROW2, CAPTURE2))
                .setLinearHeadingInterpolation(ROW2.getHeading(), CAPTURE2.getHeading())
                .build();

        capture2ToRow2 = follower.pathBuilder()
                .addPath(new BezierLine(CAPTURE2, ROW2))
                .setLinearHeadingInterpolation(CAPTURE2.getHeading(), ROW2.getHeading())
                .build();

        row2ToShooter = follower.pathBuilder()
                .addPath(new BezierLine(ROW2, SHOOTER))
                .setLinearHeadingInterpolation(ROW2.getHeading(), SHOOTER.getHeading())
                .build();

        // Cycle 3
        shooterToRow3 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOTER, ROW3))
                .setLinearHeadingInterpolation(SHOOTER.getHeading(), ROW3.getHeading())
                .build();

        row3ToCapture3 = follower.pathBuilder()
                .addPath(new BezierLine(ROW3, CAPTURE3))
                .setLinearHeadingInterpolation(ROW3.getHeading(), CAPTURE3.getHeading())
                .build();

        capture3ToRow3 = follower.pathBuilder()
                .addPath(new BezierLine(CAPTURE3, ROW3))
                .setLinearHeadingInterpolation(CAPTURE3.getHeading(), ROW3.getHeading())
                .build();

        row3ToShooter = follower.pathBuilder()
                .addPath(new BezierLine(ROW3, SHOOTER))
                .setLinearHeadingInterpolation(ROW3.getHeading(), SHOOTER.getHeading())
                .build();
    }

    private static Pose pose(double x, double y, double hDeg) {
        return new Pose(x, y, Math.toRadians(hDeg));
    }

    @Override
    protected void startAuto() {
        stage = 0;
        currentState = AutoState.FOLLOWING_PATH;
        follower.followPath(toShooterInitial);
    }

    @Override
    protected boolean isTargetGoal(TagObservation obs) {
        return obs != null && (obs.id == CameraConfig.RED_GOAL_TAG_ID
                || CameraConfig.CLASS_RED_GOAL.equals(obs.tagClass));
    }

    @Override
    protected double getGlanceHeading() {
        return Math.toRadians(180); // Red + 90 (90+90=180)
    }

    @Override
    protected void onDetectionFinished(int[] order) {
        cycleIndex = 0;
        startRow(order[0]);
    }

    private void startRow(int row) {
        if (row == 1) {
            stage = 10;
            currentState = AutoState.FOLLOWING_PATH;
            follower.followPath(shooterToRow1, true);
        } else if (row == 2) {
            stage = 20;
            currentState = AutoState.FOLLOWING_PATH;
            follower.followPath(shooterToRow2, true);
        } else if (row == 3) {
            stage = 30;
            currentState = AutoState.FOLLOWING_PATH;
            follower.followPath(shooterToRow3, true);
        }
    }

    @Override
    protected void onPathFinished() {
        switch (stage) {
            case 0: // Arrived at Shooter (Initial)
                currentState = AutoState.AIMING_AND_SHOOTING;
                stage = 1;
                break;

            // Cycle 1
            case 10: // Arrived at Row 1
                startIntake();
                stage = 11;
                follower.followPath(row1ToCapture1, true);
                break;
            case 11: // Arrived at Capture 1
                stopIntake();
                setShooterIdle();
                stage = 12;
                follower.followPath(capture1ToRow1, true);
                break;
            case 12: // Arrived back at Row 1
                stage = 13;
                follower.followPath(row1ToShooter, true);
                break;
            case 13: // Arrived at Shooter
                currentState = AutoState.AIMING_AND_SHOOTING;
                stage = 14;
                break;

            // Cycle 2
            case 20: // Arrived at Row 2
                startIntake();
                stage = 21;
                follower.followPath(row2ToCapture2, true);
                break;
            case 21: // Arrived at Capture 2
                stopIntake();
                setShooterIdle();
                stage = 22;
                follower.followPath(capture2ToRow2, true);
                break;
            case 22: // Arrived back at Row 2
                stage = 23;
                follower.followPath(row2ToShooter, true);
                break;
            case 23: // Arrived at Shooter
                currentState = AutoState.AIMING_AND_SHOOTING;
                stage = 24;
                break;

            // Cycle 3
            case 30: // Arrived at Row 3
                startIntake();
                stage = 31;
                follower.followPath(row3ToCapture3, true);
                break;
            case 31: // Arrived at Capture 3
                stopIntake();
                setShooterIdle();
                stage = 32;
                follower.followPath(capture3ToRow3, true);
                break;
            case 32: // Arrived back at Row 3
                stage = 33;
                follower.followPath(row3ToShooter, true);
                break;
            case 33: // Arrived at Shooter
                currentState = AutoState.AIMING_AND_SHOOTING;
                stage = 34;
                break;
        }
    }

    @Override
    protected void onShotFired() {
        if (stage == 1) {
            // Initial shot done, start Detection
            currentState = AutoState.DETECTING;
        } else {
            // Cycle shot done, move to next in order
            cycleIndex++;
            if (cycleIndex < cycleOrder.length) {
                startRow(cycleOrder[cycleIndex]);
            } else {
                currentState = AutoState.FINISHED;
                stage = 99;
            }
        }
    }
}

package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.CameraConfig;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera;

import java.util.List;

/**
 * BjornCVTracker — Simplified "Pure CV" Control.
 *
 * Features:
 * - Uses a simple PID loop to turn the robot to face the AprilTag.
 * - Driver retains translation control (Left Stick).
 * - Rotation is automated when tracking is active (A button).
 *
 * Controls:
 * - A: Toggle Tracking (ON/OFF)
 * - D-Pad Up/Down: Adjust Decimation (FPS vs Range)
 */
@TeleOp(name = "BjornCVTracker", group = "Test")
public class BjornCVTracker extends OpMode {

    private Follower follower;
    private AprilTagCamera camera;

    private boolean trackingActive = false;
    private boolean aPrev = false;
    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;

    private float currentDecimation = 3.0f;

    // PID Constants for Turn
    // Tune these for smoothness!
    private static final double kP = 0.035; // Power per degree of error
    private static final double kD = 0.002; // Dampening
    private static final double MAX_TURN_SPEED = 0.5; // Cap turn speed for safety

    private double lastError = 0;
    private long lastTime = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        camera = new AprilTagCamera();
        camera.start(hardwareMap, null, currentDecimation);
        camera.setManualExposure(6, 250);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // 1. Inputs
        if (gamepad1.a && !aPrev) {
            trackingActive = !trackingActive;
        }
        aPrev = gamepad1.a;

        if (gamepad1.dpad_up && !dpadUpPrev)
            changeDecimation(1.0f);
        dpadUpPrev = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !dpadDownPrev)
            changeDecimation(-1.0f);
        dpadDownPrev = gamepad1.dpad_down;

        // 2. CV Logic
        AprilTagCamera.TagObservation target = null;
        List<AprilTagCamera.TagObservation> tags = camera.pollDetections();
        for (AprilTagCamera.TagObservation tag : tags) {
            if (tag.id == CameraConfig.BLUE_GOAL_TAG_ID || tag.id == CameraConfig.RED_GOAL_TAG_ID) {
                target = tag;
                break;
            }
        }

        // 3. Control Logic
        double turnPower = 0;

        if (trackingActive && target != null) {
            // Error is the bearing to the tag (in degrees)
            // bearing = atan2(x, z)
            // x is left/right, z is forward distance
            // We want to turn to center the tag (x=0)
            double bearingRad = Math.atan2(target.x, target.z);
            double errorDeg = Math.toDegrees(bearingRad);

            // Simple PD
            long now = System.currentTimeMillis();
            double dt = (now - lastTime) / 1000.0;
            if (lastTime == 0)
                dt = 0;

            double derivative = (dt > 0) ? (errorDeg - lastError) / dt : 0;

            turnPower = (errorDeg * kP) + (derivative * kD);

            // Clamp and invert if needed (positive turn power usually turns left)
            // If tag is to the right (positive x?), bearing is positive?
            // Let's check coordinate system:
            // Camera: Z forward, X right.
            // If tag is at X = +1 (Right), atan2(1, z) is positive.
            // To face right, we need to turn Right (Negative rotation usually).
            // So we might need -turnPower.
            // Let's try -turnPower first based on standard right-hand rule (Left turn =
            // positive).
            turnPower = -turnPower;

            turnPower = Range.clip(turnPower, -MAX_TURN_SPEED, MAX_TURN_SPEED);

            lastError = errorDeg;
            lastTime = now;
        } else {
            // Manual turn
            turnPower = -gamepad1.right_stick_x;
            lastError = 0; // Reset error when not tracking
        }

        // 4. Drive
        // Note: Pedro's setTeleOpDrive takes (xVel, yVel, turnVel, fieldCentric)
        // We use -left_stick_y for forward (xVel), -left_stick_x for strafe (yVel)
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, turnPower, true);
        follower.update();

        // 5. Telemetry
        telemetry.addData("Tracking", trackingActive ? "ON" : "OFF");
        telemetry.addData("Decimation", currentDecimation);
        if (target != null) {
            telemetry.addData("Target ID", target.id);
            telemetry.addData("Bearing", String.format("%.1f deg", Math.toDegrees(Math.atan2(target.x, target.z))));
            telemetry.addData("Turn Pwr", String.format("%.2f", turnPower));
        }
        telemetry.update();
    }

    private void changeDecimation(float delta) {
        currentDecimation += delta;
        if (currentDecimation < 1.0f)
            currentDecimation = 1.0f;
        camera.start(hardwareMap, null, currentDecimation);
        camera.setManualExposure(6, 250);
    }

    @Override
    public void stop() {
        if (camera != null)
            camera.close();
    }
}

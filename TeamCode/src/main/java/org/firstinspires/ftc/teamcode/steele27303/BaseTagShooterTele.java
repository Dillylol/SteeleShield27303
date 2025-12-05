package org.firstinspires.ftc.teamcode.steele27303;

import android.os.SystemClock;

import androidx.annotation.Nullable;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera.TagObservation;
import org.firstinspires.ftc.teamcode.jules.shot.ShooterController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.Locale;
import java.util.Arrays;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Shared AprilTag-enabled shooter TeleOp logic that extends the legacy
 * BjornTele base class.
 * Concrete subclasses select which goal tags to track (red or blue).
 */
abstract class BaseTagShooterTele extends BjornTele {

    private static final double METERS_TO_FEET = 3.28084;
    private static final double AIM_HEADING_KP = 0.035;
    private static final double MAX_AIM_CORRECTION = 0.45;
    private static final double RPM_SLOPE = 116.4042383594456;
    private static final double RPM_OFFSET = 2284.2966941424975;
    private static final double MANUAL_RPM_STEP = 50.0;
    private static final double MANUAL_RPM_MIN = 1500.0;
    private static final double MANUAL_RPM_MAX = 3200.0;
    private static final long TAG_MEMORY_MS = 250L;

    private BjornHardware hardware;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx intake;
    private IMU imu;
    private Servo lift;
    private ShooterController shooterController;
    private AprilTagCamera aprilTagCamera;
    private Follower follower; // PedroPathing
    private DistanceSensor tofFront;

    private Pose autoDriveTargetPose;

    // TOF Scan State
    private static final int SCAN_SAMPLES = 10;
    private int scanLeft = 0;
    private final double[] scanBuf = new double[SCAN_SAMPLES];
    private double lastScanFt = Double.NaN;
    private long lastScanTime = 0L;
    private static final double SENSOR_TO_CANNON_OFFSET_FT = 0.0;

    private boolean autoAimEnabled = false;
    private boolean manualShooterEnabled = false;
    private boolean autoDriveActive = false; // Pedro Auto-Drive
    private boolean g1LbPrev;
    private boolean g1RbPrev;
    private boolean g1BPrev;
    private boolean g2BPrev;
    private boolean g2RbPrev;
    private boolean g2LbPrev;
    private boolean fireButtonPrev;
    private boolean yawResetPrev;

    private double manualTargetRpm = 2600.0;
    private double activeTargetRpm = 0.0;
    private double intakeCommand = 0.0;
    private double lastAimCorrection = 0.0;

    private TagObservation lastObservation;
    private long lastObservationTs = 0L;
    private ShooterController.ShotMetrics lastShotMetrics;

    protected abstract Pose getAutoEndPose(); // For Pedro Auto-Drive

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(50);
        hardware = BjornHardware.forTeleOp(hardwareMap);
        frontLeft = hardware.frontLeft;
        frontRight = hardware.frontRight;
        backLeft = hardware.backLeft;
        backRight = hardware.backRight;
        intake = hardware.intake;
        imu = hardware.imu;
        lift = hardware.lift;

        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(params);

        shooterController = new ShooterController(
                hardware.wheel,
                hardware.wheel2,
                hardware.intake,
                hardware,
                hardware.getVoltageSensor());

        aprilTagCamera = new AprilTagCamera();
        aprilTagCamera.start(hardwareMap, null);

        // Initialize Pedro Follower
        follower = Constants.createFollower(hardwareMap);
        // We can initialize at Auto End Pose if we trust it, or (0,0)
        // Let's use getAutoEndPose() to be consistent with previous logic
        follower.setStartingPose(getAutoEndPose());
        autoDriveTargetPose = getAutoEndPose();

        tofFront = hardware.frontTof;

    }

    @Override
    public void start() {
        if (lift != null) {
            try {
                // Use hardware.setLiftPosition to apply offsets
                hardware.setLiftPosition(BjornConstants.Servos.LIFT_LOWERED);
            } catch (Exception ignored) {
            }
        }
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        long nowMs = SystemClock.elapsedRealtime();
        follower.update(); // Update Pedro

        // TOF Scanning Logic
        if (scanLeft > 0) {
            double inches = safeTofInches(tofFront);
            scanBuf[SCAN_SAMPLES - scanLeft] = inches;
            scanLeft--;
            if (scanLeft == 0) {
                double inchesMed = median(scanBuf);
                if (inchesMed > 6 && inchesMed <= 96) { // up to ~8 ft
                    double ft = inchesMed / 12.0 + SENSOR_TO_CANNON_OFFSET_FT;
                    lastScanFt = ft;
                    lastScanTime = nowMs;
                    if (manualShooterEnabled) {
                        manualTargetRpm = clampManualRpm(RPM_SLOPE * ft + RPM_OFFSET);
                    }
                }
            }
        }

        updateToggles();
        TagObservation obs = updateObservation(nowMs);
        boolean tracking = autoAimEnabled && obs != null;
        ShooterSolution solution = null;

        if (tracking) {
            // 1. Calculate Aim Heading from CV
            double thetaDeg = Math.toDegrees(Math.atan2(obs.x, obs.z));
            double lateralFeet = obs.x * METERS_TO_FEET;
            double planarMeters = planarDistanceMeters(obs);
            double rangeFeet = planarMeters * METERS_TO_FEET;

            // 2. Check alignment (0.5 ft lateral tolerance)
            boolean aligned = Math.abs(lateralFeet) < 0.5;

            // Calculate CV-based RPM as fallback
            double cvRpm = clampManualRpm(RPM_SLOPE * rangeFeet + RPM_OFFSET);
            double yawGain = 1.0 + Math.abs(thetaDeg) * 0.002; // slight boost for angled shots
            cvRpm = clampManualRpm(cvRpm * yawGain);

            double rpm = 0.0;
            double rangeFt = 0.0;

            if (aligned) {
                // 3. Fire Laser (Trigger scan if idle)
                if (scanLeft == 0) {
                    beginScan();
                }

                // 4. Use Laser Distance for RPM if data is fresh (e.g., < 2000ms)
                if (!Double.isNaN(lastScanFt) && (nowMs - lastScanTime < 2000)) {
                    rangeFt = lastScanFt;
                    rpm = clampManualRpm(RPM_SLOPE * rangeFt + RPM_OFFSET);
                } else {
                    // Fallback to CV RPM if laser is stale/invalid
                    rangeFt = rangeFeet; // Use CV range for telemetry
                    rpm = cvRpm;
                }
            }
            // If not aligned, rpm remains 0.0 (flywheel stops/idles)

            solution = new ShooterSolution(rpm, thetaDeg, rangeFt, lateralFeet, obs.id);
        }

        updateDrive(solution, tracking);
        updateIntake();
        updateShooter(solution, tracking, nowMs);
        handleFire(nowMs);
        shooterController.update(nowMs);
        lastShotMetrics = shooterController.pollShotMetrics();
        sendTelemetry(obs, solution, tracking, nowMs);
    }

    @Override
    public void stop() {
        long nowMs = SystemClock.elapsedRealtime();
        if (shooterController != null) {
            shooterController.stop(nowMs);
        }
        if (intake != null) {
            intake.setPower(0.0);
        }
        if (aprilTagCamera != null) {
            aprilTagCamera.close();
        }
    }

    protected abstract boolean isTargetGoal(TagObservation obs);

    protected abstract String allianceName();

    private void updateToggles() {
        // Manual Shoot Toggle + TOF Scan (Left Bumper)
        boolean manualToggle = gamepad1.left_bumper;
        if (manualToggle && !g1LbPrev) {
            manualShooterEnabled = !manualShooterEnabled;
            beginScan(); // Trigger scan when toggling manual mode
            // If we just enabled it, we might want to update RPM immediately if we have a
            // valid last scan?
            // The scan logic in loop() will handle it if a scan is running.
            // If we want to use the *previous* scan immediately:
            if (manualShooterEnabled && !Double.isNaN(lastScanFt)) {
                manualTargetRpm = clampManualRpm(RPM_SLOPE * lastScanFt + RPM_OFFSET);
            }
        }
        g1LbPrev = manualToggle;

        // Auto-Aim Toggle (B)
        boolean g1B = gamepad1.b;
        if (g1B && !g1BPrev) {
            autoAimEnabled = !autoAimEnabled;
        }
        g1BPrev = g1B;

        // Gamepad 2 B also toggles manual shooter (legacy/redundancy)
        boolean g2B = gamepad2.b;
        if (g2B && !g2BPrev) {
            manualShooterEnabled = !manualShooterEnabled;
            beginScan();
        }
        g2BPrev = g2B;

        // Auto-Drive Waypoint Set (Right Bumper)
        boolean setWaypoint = gamepad1.right_bumper;
        if (setWaypoint && !g1RbPrev) {
            autoDriveTargetPose = follower.getPose();
        }
        g1RbPrev = setWaypoint;

        // Manual RPM Trim (Gamepad 2)
        if (gamepad2.right_bumper && !g2RbPrev) {
            manualTargetRpm = clampManualRpm(manualTargetRpm + MANUAL_RPM_STEP);
        }
        if (gamepad2.left_bumper && !g2LbPrev) {
            manualTargetRpm = clampManualRpm(manualTargetRpm - MANUAL_RPM_STEP);
        }
        g2RbPrev = gamepad2.right_bumper;
        g2LbPrev = gamepad2.left_bumper;

        boolean yawReset = gamepad1.dpad_down;
        if (yawReset && !yawResetPrev && imu != null) {
            imu.resetYaw();
            follower.setStartingPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0));
        }
        yawResetPrev = yawReset;
    }

    private void updateDrive(@Nullable ShooterSolution solution, boolean tracking) {
        // Auto-Drive (Left Trigger)
        if (gamepad1.left_trigger > 0.5) {
            if (!autoDriveActive) {
                Path path = new Path(new BezierLine(follower.getPose(), autoDriveTargetPose));
                path.setLinearHeadingInterpolation(follower.getPose().getHeading(), autoDriveTargetPose.getHeading());
                follower.followPath(path, true);
                autoDriveActive = true;
            }
            return; // Pedro handles drive
        } else {
            if (autoDriveActive) {
                follower.startTeleopDrive();
                autoDriveActive = false;
            }
        }

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        if (tracking && solution != null) {
            double correction = computeAimCorrection(solution);
            rx = Range.clip(rx + correction, -1.0, 1.0);
            lastAimCorrection = correction;
        } else {
            lastAimCorrection = 0.0;
        }

        double heading = (imu != null)
                ? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
                : 0.0;
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
        double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        double fl = (rotY + rotX + rx) / denom;
        double bl = (rotY - rotX + rx) / denom;
        double fr = (rotY - rotX - rx) / denom;
        double br = (rotY + rotX - rx) / denom;
        if (frontLeft != null)
            frontLeft.setPower(fl);
        if (backLeft != null)
            backLeft.setPower(bl);
        if (frontRight != null)
            frontRight.setPower(fr);
        if (backRight != null)
            backRight.setPower(br);

        // Note: We are using manual field centric calculation here instead of Pedro's
        // setTeleOpDrive
        // This preserves the "drive mechanics" the user liked from the original
        // BaseTagShooterTele.
        // But we DO update follower in loop() so it tracks position for Auto-Drive.
    }

    private void updateIntake() {
        double intakeCmdG2 = gamepad2.a ? 1.0 : (gamepad2.x ? -1.0 : 0.0);
        double intakeCmdG1 = gamepad1.a ? 1.0 : (gamepad1.x ? -1.0 : 0.0);
        intakeCommand = (intakeCmdG2 != 0.0) ? intakeCmdG2 : intakeCmdG1;
        if (intake != null) {
            intake.setPower(intakeCommand);
        }
    }

    private void updateShooter(@Nullable ShooterSolution solution, boolean tracking, long nowMs) {
        double target = 0.0;
        if (tracking && solution != null) {
            target = solution.targetRpm;
            shooterController.setTargetRpm(target, nowMs);
        } else if (manualShooterEnabled) {
            target = manualTargetRpm;
            shooterController.setTargetRpm(target, nowMs);
        } else {
            shooterController.stop(nowMs);
        }
        activeTargetRpm = target;

        // ------------ SERVOS TRIGGER LOGIC ------------
        // If the shooter is spinning up (target > 0) AND is physically ready, lift the servo.
        if (activeTargetRpm > 0 && shooterController.isReady(nowMs)) {
            try {
                // Ensure this constant exists in your BjornConstants.Servos class!
                // If it is named differently (e.g. LIFT_UP, LIFT_FIRE), change it here.
                hardware.setLiftPosition(BjornConstants.Servos.LIFT_RAISED);
            } catch (Exception e) {
                // Failsafe if constant doesn't exist
            }
        } else {
            try {
                hardware.setLiftPosition(BjornConstants.Servos.LIFT_LOWERED);
            } catch (Exception e) {
                // Failsafe
            }
        }
        // ----------------------------------------------
    }

    private void handleFire(long nowMs) {
        boolean fireRequest = gamepad1.right_trigger > 0.6 || gamepad2.right_trigger > 0.6;
        if (fireRequest && !fireButtonPrev) {
            shooterController.fire(nowMs);
        }
        fireButtonPrev = fireRequest;
    }

    private void sendTelemetry(@Nullable TagObservation obs,
            @Nullable ShooterSolution solution,
            boolean tracking,
            long nowMs) {
        String autoState;
        if (!autoAimEnabled) {
            autoState = "OFF";
        } else if (tracking) {
            autoState = "ON - TRACKING";
        } else {
            autoState = "ON - NO TAG";
        }
        telemetry.addLine("=== Auto Aim ===");
        telemetry.addData("Alliance", allianceName());
        telemetry.addData("AutoAim", autoState);
        telemetry.addData("TagId", (obs != null) ? obs.id : "none");
        telemetry.addData("Detections", (aprilTagCamera != null) ? aprilTagCamera.getLastDetectionCount() : 0);
        telemetry.addData("Range(ft)", solution != null ? format(solution.rangeFeet) : "n/a");
        telemetry.addData("Theta(deg)", solution != null ? format(solution.aimHeadingDeg) : "n/a");
        telemetry.addData("Lateral(ft)", solution != null ? format(solution.lateralFeet) : "n/a");
        telemetry.addData("AimCorrection", String.format(Locale.US, "%.3f", lastAimCorrection));
        telemetry.addData("LastScan(ft)",
                !Double.isNaN(lastScanFt) ? String.format(Locale.US, "%.2f", lastScanFt) : "n/a");

        telemetry.addLine("=== Shooter ===");
        telemetry.addData("CmdTargetRPM", String.format(Locale.US, "%.0f", activeTargetRpm));
        telemetry.addData("ControllerTargetRPM", shooterController.getTargetRpm());
        telemetry.addData("MeasuredRPM", String.format(Locale.US, "%.0f", shooterController.getMeasuredRpm()));
        telemetry.addData("Ready", shooterController.isReady(nowMs));
        telemetry.addData("LockedOut", shooterController.isLockedOut(nowMs));
        telemetry.addData("ManualTargetRPM", String.format(Locale.US, "%.0f", manualTargetRpm));
        telemetry.addData("ManualEnabled", manualShooterEnabled);
        telemetry.addData("AutoTracking", tracking);
        if (lastShotMetrics != null) {
            telemetry.addData("LastShotRPM", String.format(Locale.US, "%.0f", lastShotMetrics.rpmAtFire));
            telemetry.addData("ReadyLatencyMs", lastShotMetrics.timeToReadyMs);
            telemetry.addData("LastShotTs", lastShotMetrics.fireTimestampMs);
        }

        telemetry.addLine("=== Intake / Power ===");
        telemetry.addData("Intake", intakeCommand);
        telemetry.addData("Battery(V)", hardware != null
                ? String.format(Locale.US, "%.2f", hardware.getBatteryVoltage())
                : "n/a");
        telemetry.update();
    }

    private TagObservation updateObservation(long nowMs) {
        if (aprilTagCamera == null) {
            return lastObservation;
        }
        List<TagObservation> detections = aprilTagCamera.pollDetections();
        TagObservation best = null;
        double bestPlanar = Double.POSITIVE_INFINITY;
        for (TagObservation obs : detections) {
            if (!isTargetGoal(obs)) {
                continue;
            }
            double planar = planarDistanceMeters(obs);
            if (best == null || planar < bestPlanar) {
                bestPlanar = planar;
                best = obs;
            }
        }
        aprilTagCamera.publishDetections();
        if (best != null) {
            lastObservation = best;
            lastObservationTs = nowMs;
        } else if (nowMs - lastObservationTs > TAG_MEMORY_MS) {
            lastObservation = null;
        }
        return lastObservation;
    }



    private static double planarDistanceMeters(TagObservation obs) {
        return Math.hypot(obs.x, obs.z);
    }

    private double clampManualRpm(double rpm) {
        return Range.clip(rpm, MANUAL_RPM_MIN, MANUAL_RPM_MAX);
    }

    private static String format(double value) {
        return String.format(Locale.US, "%.2f", value);
    }

    private double computeAimCorrection(ShooterSolution solution) {
        double errorRad = Math.toRadians(solution.aimHeadingDeg);
        double correction = errorRad * AIM_HEADING_KP;
        return Range.clip(correction, -MAX_AIM_CORRECTION, MAX_AIM_CORRECTION);
    }

    /**
     * Encapsulates the RPM + heading requirements derived from a tag observation.
     */
    protected static final class ShooterSolution {
        final double targetRpm;
        final double aimHeadingDeg;
        final double rangeFeet;
        final double lateralFeet;
        final int tagId;

        ShooterSolution(double targetRpm,
                double aimHeadingDeg,
                double rangeFeet,
                double lateralFeet,
                int tagId) {
            this.targetRpm = targetRpm;
            this.aimHeadingDeg = aimHeadingDeg;
            this.rangeFeet = rangeFeet;
            this.lateralFeet = lateralFeet;
            this.tagId = tagId;
        }
    }

    private void beginScan() {
        Arrays.fill(scanBuf, 0.0);
        scanLeft = SCAN_SAMPLES;
    }

    private static double safeTofInches(DistanceSensor ds) {
        try {
            double d = ds.getDistance(DistanceUnit.INCH);
            return (Double.isNaN(d) || d <= 0) ? -1.0 : d;
        } catch (Exception e) {
            return -1.0;
        }
    }

    private static double median(double[] a) {
        double[] b = Arrays.copyOf(a, a.length);
        Arrays.sort(b);
        int n = b.length;
        return (n % 2 == 1) ? b[n / 2] : 0.5 * (b[n / 2 - 1] + b[n / 2]);
    }
}
package org.firstinspires.ftc.teamcode.steele27303;

import android.os.SystemClock;

import androidx.annotation.Nullable;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.common.CameraConfig;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera.TagObservation;
import org.firstinspires.ftc.teamcode.jules.shot.ShooterController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/**
 * Base class for Autonomous programs that use CV aiming and Laser distance.
 */
public abstract class BaseTagShooterAuto extends OpMode {

    private static final double METERS_TO_FEET = 3.28084;
    private static final double AIM_HEADING_KP = 0.035;
    private static final double MAX_AIM_CORRECTION = 0.45;
    private static final double RPM_SLOPE = 116.4042383594456;
    private static final double RPM_OFFSET = 2284.2966941424975; // Updated to match TeleOp
    private static final long TAG_MEMORY_MS = 250L;

    protected BjornHardware hardware;
    protected DcMotor frontLeft, frontRight, backLeft, backRight;
    protected DcMotorEx intake;
    protected IMU imu;
    protected ShooterController shooterController;
    protected AprilTagCamera aprilTagCamera;
    protected Follower follower;
    protected DistanceSensor tofFront;

    // TOF Scan State
    private static final int SCAN_SAMPLES = 10;
    private int scanLeft = 0;
    private final double[] scanBuf = new double[SCAN_SAMPLES];
    private double lastScanFt = Double.NaN;
    private long lastScanTime = 0L;
    private static final double SENSOR_TO_CANNON_OFFSET_FT = 0.0;

    private TagObservation lastObservation;
    private long lastObservationTs = 0L;
    private long readyStartTime = 0L;

    protected enum AutoState {
        IDLE,
        FOLLOWING_PATH,
        AIMING_AND_SHOOTING,
        DETECTING,
        FINISHED
    }

    protected AutoState currentState = AutoState.IDLE;
    protected PathChain currentPath;
    protected boolean shotsFired = false; // Example flag, can be expanded
    
    // Detection Logic
    protected int[] cycleOrder = {1, 2, 3}; // Default order
    protected boolean detectionDone = false;
    protected long detectionStartTime = 0L;
    protected static final long DETECTION_TIMEOUT_MS = 1500; // 1.5 seconds to glance

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(50);
        hardware = BjornHardware.forTeleOp(hardwareMap); // Using forTeleOp as it inits same hardware
        frontLeft = hardware.frontLeft;
        frontRight = hardware.frontRight;
        backLeft = hardware.backLeft;
        backRight = hardware.backRight;
        intake = hardware.intake;
        imu = hardware.imu;

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

        follower = Constants.createFollower(hardwareMap);
        tofFront = hardware.frontTof;

        buildPaths();
    }

    @Override
    public void start() {
        follower.startTeleopDrive(); // Start Pedro
        startAuto();
    }

    @Override
    public void loop() {
        long nowMs = SystemClock.elapsedRealtime();
        follower.update();
        shooterController.update(nowMs);

        // TOF Scanning Logic (Always available)
        if (scanLeft > 0) {
            double inches = safeTofInches(tofFront);
            scanBuf[SCAN_SAMPLES - scanLeft] = inches;
            scanLeft--;
            if (scanLeft == 0) {
                double inchesMed = median(scanBuf);
                if (inchesMed > 6 && inchesMed <= 96) {
                    double ft = inchesMed / 12.0 + SENSOR_TO_CANNON_OFFSET_FT;
                    lastScanFt = ft;
                    lastScanTime = nowMs;
                }
            }
        }

        switch (currentState) {
            case FOLLOWING_PATH:
                if (!follower.isBusy()) {
                    onPathFinished();
                }
                break;
            case AIMING_AND_SHOOTING:
                aimAndShoot(nowMs);
                break;
            case DETECTING:
                detectObelisk(nowMs);
                break;
            case IDLE:
            case FINISHED:
                break;
        }
        
        telemetry.addData("State", currentState);
        telemetry.update();
    }
    
    @Override
    public void stop() {
        long nowMs = SystemClock.elapsedRealtime();
        if (shooterController != null) {
            shooterController.stop(nowMs);
        }
        if (aprilTagCamera != null) {
            aprilTagCamera.close();
        }
    }

    protected abstract void buildPaths();
    protected abstract void startAuto();
    protected abstract boolean isTargetGoal(TagObservation obs);
    protected abstract void onPathFinished();
    protected abstract double getGlanceHeading();
    protected abstract void onDetectionFinished(int[] order);

    protected void aimAndShoot(long nowMs) {
        TagObservation obs = updateObservation(nowMs);
        
        if (obs == null) {
            // No tag, stop motors (or hold position?)
            drive(0, 0, 0);
            shooterController.stop(nowMs);
            return;
        }

        // 1. Calculate Aim Heading from CV
        double thetaDeg = Math.toDegrees(Math.atan2(obs.x, obs.z));
        double lateralFeet = obs.x * METERS_TO_FEET;
        double planarMeters = Math.hypot(obs.x, obs.z);
        double rangeFeet = planarMeters * METERS_TO_FEET;

        // 2. Check alignment (0.5 ft lateral tolerance)
        boolean aligned = Math.abs(lateralFeet) < 0.5;

        // Calculate CV-based RPM as fallback
        double cvRpm = clampManualRpm(RPM_SLOPE * rangeFeet + RPM_OFFSET);
        double yawGain = 1.0 + Math.abs(thetaDeg) * 0.002;
        cvRpm = clampManualRpm(cvRpm * yawGain);

        double rpm = 0.0;
        double rangeFt = 0.0;

        if (aligned) {
            // 3. Fire Laser (Trigger scan if idle)
            if (scanLeft == 0) {
                beginScan();
            }

            // 4. Use Laser Distance for RPM if data is fresh
            if (!Double.isNaN(lastScanFt) && (nowMs - lastScanTime < 2000)) {
                rangeFt = lastScanFt;
                rpm = clampManualRpm(RPM_SLOPE * rangeFt + RPM_OFFSET);
            } else {
                // Fallback to CV
                rangeFt = rangeFeet;
                rpm = cvRpm;
            }
        }

        // Drive Logic (Aiming)
        double correction = 0.0;
        if (Math.abs(thetaDeg) > 1.0) { // Deadband
             correction = Range.clip(Math.toRadians(thetaDeg) * AIM_HEADING_KP, -MAX_AIM_CORRECTION, MAX_AIM_CORRECTION);
        }
        drive(0, 0, correction); // Rotate to aim

        // Shooter Logic
        if (rpm > 0) {
            shooterController.setTargetRpm(rpm, nowMs);
            
            boolean ready = shooterController.isReady(nowMs);
            
            if (ready) {
                if (readyStartTime == 0) {
                    readyStartTime = nowMs;
                }
                
                // Wait 2 seconds before firing/intaking
                if (nowMs - readyStartTime > 2000) {
                    startIntake();
                    if (!shooterController.isLockedOut(nowMs)) {
                        shooterController.fire(nowMs);
                        onShotFired();
                    }
                } else {
                    stopIntake();
                }
            } else {
                readyStartTime = 0;
                stopIntake();
            }
        } else {
            shooterController.stop(nowMs);
            stopIntake();
            readyStartTime = 0;
        }
    }
    
    protected void onShotFired() {
        // Can be overridden or used to trigger state change
    }

    protected void detectObelisk(long nowMs) {
        if (detectionStartTime == 0) {
            detectionStartTime = nowMs;
        }

        // 1. Rotate to Glance Heading
        double targetHeading = getGlanceHeading();
        double currentHeading = (imu != null)
                ? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
                : 0.0;
        
        double error = AngleUnit.normalizeRadians(targetHeading - currentHeading);
        double correction = Range.clip(error * AIM_HEADING_KP, -MAX_AIM_CORRECTION, MAX_AIM_CORRECTION);
        
        // If within tolerance, we are "glancing"
        if (Math.abs(Math.toDegrees(error)) < 5.0) {
            drive(0, 0, 0); // Stop to stabilize? Or keep correcting?
        } else {
            drive(0, 0, correction);
        }

        // 2. Check for Tags
        List<TagObservation> detections = (aprilTagCamera != null) ? aprilTagCamera.pollDetections() : null;
        if (detections != null) {
            for (TagObservation obs : detections) {
                if (obs.id == CameraConfig.OBELISK_TAG_ID_21) {
                    // GPP -> Row 1, Row 2, Row 3 (or whatever logic)
                    // User: "each row has a specific pattern associated with it depending on the april tag"
                    // User: "robot choose a the row the april tag associates with then executes everything else with no patern"
                    // Interpretation: 
                    // Tag 21 (GPP) -> Do Row 1 first. Then others? 
                    // "executes everything else with no patern" -> implies default order for the rest?
                    // Let's assume:
                    // 21 -> {1, 2, 3} (Matches default if 1 is first)
                    // 22 -> {2, 1, 3} (Row 2 first)
                    // 23 -> {3, 1, 2} (Row 3 first)
                    // Or maybe just the single row? "executes everything else" implies it does all rows.
                    
                    finishDetection(new int[]{1, 2, 3}); 
                    return;
                } else if (obs.id == CameraConfig.OBELISK_TAG_ID_22) {
                    // PGP -> Row 2
                    finishDetection(new int[]{2, 1, 3});
                    return;
                } else if (obs.id == CameraConfig.OBELISK_TAG_ID_23) {
                    // PPG -> Row 3
                    finishDetection(new int[]{3, 1, 2});
                    return;
                }
            }
        }

        // 3. Timeout
        if (nowMs - detectionStartTime > DETECTION_TIMEOUT_MS) {
            // Failed to find tag, use default
            finishDetection(new int[]{1, 2, 3});
        }
    }
    
    private void finishDetection(int[] order) {
        cycleOrder = order;
        detectionDone = true;
        detectionStartTime = 0;
        onDetectionFinished(order);
    }

    // ---------------- Helper Methods for Subclasses ----------------

    protected void startIntake() {
        if (intake != null) intake.setPower(1.0);
    }

    protected void stopIntake() {
        if (intake != null) intake.setPower(0.0);
    }

    protected void reverseIntake() {
        if (intake != null) intake.setPower(-1.0);
    }

    protected void stopShooter() {
        if (shooterController != null) shooterController.stop(SystemClock.elapsedRealtime());
    }

    protected void setShooterIdle() {
        if (shooterController != null) shooterController.setTargetRpm(2000, SystemClock.elapsedRealtime());
    }

    private void drive(double x, double y, double rx) {
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
        if (frontLeft != null) frontLeft.setPower(fl);
        if (backLeft != null) backLeft.setPower(bl);
        if (frontRight != null) frontRight.setPower(fr);
        if (backRight != null) backRight.setPower(br);
    }

    private TagObservation updateObservation(long nowMs) {
        if (aprilTagCamera == null) return lastObservation;
        List<TagObservation> detections = aprilTagCamera.pollDetections();
        TagObservation best = null;
        double bestPlanar = Double.POSITIVE_INFINITY;
        for (TagObservation obs : detections) {
            if (!isTargetGoal(obs)) continue;
            double planar = Math.hypot(obs.x, obs.z);
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
    
    private double clampManualRpm(double rpm) {
        return Range.clip(rpm, 1500.0, 3200.0);
    }
}

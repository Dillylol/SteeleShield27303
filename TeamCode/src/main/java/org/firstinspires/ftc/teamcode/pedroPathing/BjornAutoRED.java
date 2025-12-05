package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.BjornHardware;

import java.util.Arrays;
import java.util.List;

/**
 * BjornAUTO2 — TeleOp-inspired autonomous with dynamic flywheel RPM and intake
 * pulses.
 *
 * RED SIDE (mirrored from BLUE): (x, y, θ) → (−x, y, 180° − θ), θ normalized to
 * (−180°, 180°].
 * Includes: settle-before-scan, lift→intake delay gate, intake ON during GRAB +
 * return to ALIGN1_BACK,
 * final RPM bias, and full state machine. Uses the NEW Pedro packages
 * (com.pedropathing.*).
 *
 * Updates:
 * 5) ADDED: S-Curve flywheel ramping, Sensor Fusion (ToF+CV+Pedro), and
 * Drivetrain Optimization.
 */
@Autonomous(name = "BjornAutoRED")
public class BjornAutoRED extends OpMode {

    // ---------------- Hardware ----------------
    private Follower follower;
    private DcMotorEx Intake, Wheel, Wheel2;
    private Servo Lift;
    private DistanceSensor tof;
    private org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera camera;

    // S-Curve State
    private double currentRampRPM = 0.0;
    private long lastLoopMs = 0;
    private double targetRpm = 0;
    private boolean wheelOn = false;

    // ---------------- Tunables ----------------
    // Speeds
    private static double DRIVE_SPEED_NORMAL = 1.00;
    private static double DRIVE_SPEED_ALIGN = 0.70; // approaching shoot zone

    // Wheel RPMs
    private static double WHEEL_IDLE_RPM = 2000; // flywheel idle while navigating or staging
    private static double WHEEL_MAX_RPM = 4000; // maximum allowed
    private static double WHEEL_MIN_RPM = 2000; // minimum usable for launches

    // Intake power
    private static double INTAKE_POWER = 0.70;

    // Intake pulse pattern when shooting
    private static long INTAKE_PULSE_ON_MS = 3000L; // on duration
    private static long INTAKE_PULSE_OFF_MS = 1000L; // off duration between pulses
    private static long SHOOT_WINDOW_MS = 10000L; // total time for a shoot phase

    // Lift positions (if used)
    private static double LIFT_LOWERED = BjornConstants.Servos.LIFT_LOWERED;
    private static double LIFT_RAISED = BjornConstants.Servos.LIFT_RAISED;
    private static boolean USE_LIFT = true; // flip to false if no lift gate

    // Delay after commanding lift open before intake may run
    private static long LIFT_TO_INTAKE_DELAY_MS = 1000L; // 1.0s (tunable)

    // Settle delay at shoot pose BEFORE starting TOF scan
    private static long SETTLE_BEFORE_SCAN_MS = 500L; // 0.5s (tunable)

    // Distance→RPM mapping (feet)
    // rpm = M_RPM_PER_FT * feet + B_RPM_OFFSET (clamped to [WHEEL_MIN_RPM,
    // WHEEL_MAX_RPM])
    private static double M_RPM_PER_FT = 116.4042383594456;
    private static double B_RPM_OFFSET = 2084.2966941424975;

    // final RPM bias to compensate small undershoot/overshoot (applied only to
    // final launch target)
    private static double FINAL_RPM_OFFSET = 150.0; // + adds RPM, − subtracts

    // Hysteresis (for deciding if wheel is "ready")
    private static double READY_ON_RPM = 2200; // consider ready when >= this
    private static double READY_OFF_RPM = 2100; // drop ready if <= this

    // Scan parameters
    private static int SCAN_SAMPLES = 15; // median filter size
    private static double SENSOR_OFFSET_FT = 0.0; // add small offset if sensor is not at muzzle

    // Encoder math if you convert to velocity control
    private static double MOTOR_ENCODER_CPR = 28.0;
    private static double GEAR_RATIO = 1.0;
    private static double TPR = MOTOR_ENCODER_CPR * GEAR_RATIO;

    // ---------------- RED Poses (mirrored from your BLUE) ----------------
    // BLUE source (for reference):
    // START(0,0,265), SHOOT(0,25,-90), ALIGN1(22.6,37.7,-58), GRAB1(32.9,16,-58),
    // ALIGN1_BACK(23,31,-58), PARK(24,52,-145)
    // RED after mirror (x→-x, θ→180-θ):
    private static final Pose START = pose(0.0, 0.0, -85.0);
    private static final Pose SHOOT_ZONE = pose(0.0, 25.0, -85.0);
    private static final Pose ALIGN1 = pose(-9.4, 32.9, -119.0);
    private static final Pose GRAB1 = pose(-32.9, 16.0, -119.0);
    private static final Pose ALIGN1_BACK = pose(-23.0, 31.0, -119.0);
    // Optionals if you later restore them on BLUE:
    // private static final Pose ALIGN2 = pose( -35.3, 52.8, -122.0);
    // private static final Pose GRAB2 = pose( -47.0, 32.3, -122.0);
    private static final Pose PARK = pose(-24.0, 52.0, -35.0);

    // ---------------- Paths ----------------
    private PathChain toShoot, toAlign1, toGrab1, toAlign1Back, toShoot2, toAlign2, toGrab2, toShoot3, toPark;

    // ---------------- State Machine ----------------
    private enum State {
        TO_SHOOT, SHOOT1,
        TO_ALIGN1, TO_GRAB1, TO_ALIGN1_BACK,
        TO_SHOOT2, SHOOT2,
        TO_ALIGN2, TO_GRAB2,
        TO_SHOOT3, SHOOT3,
        TO_PARK, DONE
    }

    private State state;

    // Pulse/phase timers
    private long shootPhaseStart = -1;
    private long pulseAnchor = -1; // base timestamp for pulses
    private boolean intakeOn = false;
    private boolean ready = false;

    // Gate: when lift was commanded open (for intake gating)
    private long liftOpenedAt = -1; // -1 = not open / not yet timed

    // Settle-before-scan handling
    private boolean scanStarted = false;
    private long scanDelayUntil = -1; // time when scanning may begin

    // Scan buffer
    private int scanLeft = 0;
    private final double[] scan = new double[16];

    private BjornHardware hardware;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap); // your Pedro v1 helper
        follower.setStartingPose(new Pose(START.getX(), START.getY(), START.getHeading()));

        hardware = BjornHardware.forAutonomous(hardwareMap);

        Intake = hardware.intake;
        Wheel = hardware.wheel;
        Wheel2 = hardware.wheel2;
        Lift = hardware.lift;
        tof = hardware.frontTof;

        // Initialize Camera
        camera = new org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera();
        camera.start(hardwareMap, null);
        camera.setManualExposure(6, 250);

        // Build paths (Pedro v1 linear heading interpolation)
        toShoot = line(START, SHOOT_ZONE);
        toAlign1 = line(SHOOT_ZONE, ALIGN1);
        toGrab1 = line(ALIGN1, GRAB1);
        toAlign1Back = line(GRAB1, ALIGN1_BACK);
        toShoot2 = line(ALIGN1_BACK, SHOOT_ZONE);
        toPark = line(SHOOT_ZONE, PARK);
        // If you later enable ALIGN2/GRAB2 legs, also build:
        // toAlign2 = line(SHOOT_ZONE, ALIGN2);
        // toGrab2 = line(ALIGN2, GRAB2);
        // toShoot3 = line(GRAB2, SHOOT_ZONE);

        // Init mechanisms — do NOT move servos here to avoid pre-start motion
        setWheelRPM(0);
        Intake.setPower(0);

        // Start motion pre-armed as in your original flow
        setDriveSpeed(DRIVE_SPEED_ALIGN);
        follower.followPath(toShoot);
        state = State.TO_SHOOT;
    }

    @Override
    public void start() {
        // Move servos on start to comply with "no movement during init" requirement
        if (USE_LIFT)
            hardware.setLiftPosition(LIFT_LOWERED);
    }

    @Override
    public void loop() {
        follower.update();

        // S-Curve Update
        long nowMs = System.currentTimeMillis();
        double dt = (nowMs - lastLoopMs) / 1000.0;
        if (lastLoopMs == 0)
            dt = 0.0;
        lastLoopMs = nowMs;

        currentRampRPM = updateRamp(currentRampRPM, targetRpm, dt);
        double pwr = (currentRampRPM <= 0) ? 0.0 : clamp(currentRampRPM / WHEEL_MAX_RPM, 0.0, 1.0);
        setWheelPower(pwr);

        switch (state) {
            case TO_SHOOT:
                if (follower.atParametricEnd()) {
                    beginShootPhase();
                    state = State.SHOOT1;
                }
                break;

            case SHOOT1:
                if (runShootPhase()) {
                    setDriveSpeed(DRIVE_SPEED_NORMAL);
                    follower.followPath(toAlign1, true);
                    state = State.TO_ALIGN1;
                }
                break;

            case TO_ALIGN1:
                if (follower.atParametricEnd()) {
                    Intake.setPower(INTAKE_POWER);
                    follower.followPath(toGrab1, true);
                    state = State.TO_GRAB1;
                }
                break;

            case TO_GRAB1:
                // Keep intake ON while returning to ALIGN1_BACK (do NOT turn off here)
                if (follower.atParametricEnd()) {
                    setWheelRPM(WHEEL_IDLE_RPM);
                    follower.followPath(toAlign1Back, true);
                    /* Intake.setPower(0); */ state = State.TO_ALIGN1_BACK;
                }
                break;

            case TO_ALIGN1_BACK:
                // Turn intake OFF only after we have reached ALIGN1_BACK
                if (follower.atParametricEnd()) {
                    Intake.setPower(0);
                    follower.followPath(toShoot2, true);
                    state = State.TO_SHOOT2;
                }
                break;

            case TO_SHOOT2:
                if (follower.atParametricEnd()) {
                    beginShootPhase();
                    state = State.SHOOT2;
                }
                break;

            case SHOOT2:
                if (runShootPhase()) {
                    follower.followPath(toPark, true);
                    state = State.TO_PARK;
                }
                break;

            /*
             * If restoring second cycle later:
             * case TO_ALIGN2:
             * if (follower.atParametricEnd()) { Intake.setPower(INTAKE_POWER);
             * follower.followPath(toGrab2, true); state = State.TO_GRAB2; }
             * break;
             * 
             * case TO_GRAB2:
             * if (follower.atParametricEnd()) { setWheelRPM(WHEEL_IDLE_RPM);
             * Intake.setPower(0); follower.followPath(toShoot3, true); state =
             * State.TO_SHOOT3; }
             * break;
             * 
             * case TO_SHOOT3:
             * if (follower.atParametricEnd()) { beginShootPhase(); state = State.SHOOT3; }
             * break;
             * 
             * case SHOOT3:
             * if (runShootPhase()) { follower.followPath(toPark, true); state =
             * State.TO_PARK; }
             * break;
             */

            case TO_PARK:
                if (follower.atParametricEnd()) {
                    shutdown();
                    state = State.DONE;
                }
                break;

            case DONE:
                // hold final pose
                break;
        }

        // Minimal telemetry
        long now = System.currentTimeMillis();
        telemetry.addData("State", state);
        telemetry.addData("RPM target", (int) targetRpm);
        telemetry.addData("Ramp RPM", (int) currentRampRPM);
        telemetry.addData("Final RPM offset", (int) FINAL_RPM_OFFSET);
        telemetry.addData("Wheel ready", ready);
        long waitRemaining = (liftOpenedAt < 0) ? -1 : Math.max(0, LIFT_TO_INTAKE_DELAY_MS - (now - liftOpenedAt));
        telemetry.addData("Lift→Intake wait (ms)", waitRemaining);
        long settleRemain = (scanDelayUntil < 0) ? -1 : Math.max(0, scanDelayUntil - now);
        telemetry.addData("Settle→Scan wait (ms)", settleRemain);
        telemetry.update();
    }

    // ---------------- Shooter phases ----------------
    private void beginShootPhase() {
        shootPhaseStart = System.currentTimeMillis();
        pulseAnchor = shootPhaseStart;
        intakeOn = false;
        liftOpenedAt = -1; // reset gate timer each shoot phase

        // Start wheel at idle immediately; spinning during settle
        setWheelRPM(WHEEL_IDLE_RPM);

        // Lift closed until the wheel is ready (runtime movement only)
        if (USE_LIFT)
            Lift.setPosition(LIFT_LOWERED);

        // Arm a delayed scan start; do NOT begin scanning yet
        scanStarted = false;
        scanDelayUntil = shootPhaseStart + SETTLE_BEFORE_SCAN_MS;
        scanLeft = 0; // ensure we don't accidentally consume any prior buffer
    }

    /** Returns true when the shoot window completes. */
    private boolean runShootPhase() {
        long now = System.currentTimeMillis();
        long elapsed = now - shootPhaseStart;

        // Kick off scan AFTER settle delay
        if (!scanStarted && now >= scanDelayUntil) {
            beginScan();
            scanStarted = true;
        }

        // 1) Finish scan → compute dynamic RPM once
        if (scanLeft > 0) {
            double inches = safeTofInches(tof);
            scan[SCAN_SAMPLES - scanLeft] = inches;
            scanLeft--;
            if (scanLeft == 0) {
                // Sensor Fusion: ToF + CV + Pedro
                double distToF = median(scan, SCAN_SAMPLES) / 12.0 + SENSOR_OFFSET_FT;

                double distCV = -1.0;
                List<org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera.TagObservation> tags = camera
                        .pollDetections();
                for (org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera.TagObservation tag : tags) {
                    if (tag.id == org.firstinspires.ftc.teamcode.common.CameraConfig.RED_GOAL_TAG_ID) {
                        distCV = tag.z * 3.28084; // Meters to Feet
                        break;
                    }
                }

                double distPedro = Math.abs(SHOOT_ZONE.getY() - follower.getPose().getY()) / 12.0; // Approx Y distance
                                                                                                   // in feet

                // Weighted Average (Prioritize CV > ToF > Pedro)
                double finalDist = distToF;
                if (distCV > 0) {
                    finalDist = (distToF * 0.4) + (distCV * 0.6);
                } else if (distToF < 0.5) { // ToF invalid
                    finalDist = distPedro;
                }

                if (finalDist > 0.5 && finalDist <= 10.0) {
                    double dyn = clamp(M_RPM_PER_FT * finalDist + B_RPM_OFFSET, WHEEL_MIN_RPM, WHEEL_MAX_RPM);
                    double biased = clamp(dyn + FINAL_RPM_OFFSET, WHEEL_MIN_RPM, WHEEL_MAX_RPM);
                    setWheelRPM(biased); // apply final offset only to the launch target
                } else {
                    double fallback = (WHEEL_MIN_RPM + WHEEL_MAX_RPM) * 0.5;
                    setWheelRPM(clamp(fallback + FINAL_RPM_OFFSET, WHEEL_MIN_RPM, WHEEL_MAX_RPM));
                }
            }
        }

        // 2) Simple readiness check with hysteresis
        double wheelRpm = toRPM(safeAssemblyVel(), TPR);
        if (!ready && wheelRpm >= READY_ON_RPM)
            ready = true;
        if (ready && wheelRpm <= READY_OFF_RPM)
            ready = false;

        // 3) Lift behavior + gate intake by delay AFTER lift opens
        boolean liftGatePassed;
        if (USE_LIFT) {
            if (ready) {
                hardware.setLiftPosition(LIFT_RAISED);
                if (liftOpenedAt < 0)
                    liftOpenedAt = now; // start delay the first time we open
            } else {
                hardware.setLiftPosition(LIFT_LOWERED);
                liftOpenedAt = -1; // reset if we close the lift
            }
            liftGatePassed = (liftOpenedAt >= 0) && (now - liftOpenedAt >= LIFT_TO_INTAKE_DELAY_MS);
        } else {
            liftGatePassed = true; // no lift → no gating
        }

        // 4) Intake pulsing ONLY when wheel is ready AND lift-open delay has passed
        boolean allowIntake = ready && liftGatePassed;
        if (allowIntake) {
            long sinceAnchor = now - pulseAnchor;
            long period = INTAKE_PULSE_ON_MS + INTAKE_PULSE_OFF_MS;
            long m = sinceAnchor % period;
            boolean shouldBeOn = (m < INTAKE_PULSE_ON_MS);
            if (shouldBeOn != intakeOn) {
                intakeOn = shouldBeOn;
                Intake.setPower(intakeOn ? INTAKE_POWER : 0.0);
            }
        } else {
            if (intakeOn) {
                intakeOn = false;
                Intake.setPower(0.0);
            }
        }

        // 5) End of shoot window
        if (elapsed >= SHOOT_WINDOW_MS) {
            Intake.setPower(0.0);
            if (USE_LIFT)
                Lift.setPosition(LIFT_LOWERED);
            setWheelRPM(0);
            return true;
        }
        return false;
    }

    // ---------------- Pedro helpers ----------------
    private PathChain line(Pose a, Pose b) {
        return follower.pathBuilder()
                .addPath(new BezierLine(a, b))
                .setLinearHeadingInterpolation(a.getHeading(), b.getHeading())
                .build();
    }

    private static Pose pose(double x, double y, double hDeg) {
        return new Pose(x, y, Math.toRadians(hDeg));
    }

    private void setDriveSpeed(double pwr) {
        // Drivetrain Optimizer: Scale power based on battery voltage
        double optimizedPwr = BjornHardware.forAutonomous(hardwareMap).getOptimizedDrivePower(pwr);
        try {
            follower.setMaxPower(optimizedPwr);
        } catch (Throwable ignored) {
        }
    }

    // ---------------- Wheel control ----------------
    private void setWheelRPM(double rpm) {
        targetRpm = Math.max(0, rpm);
        wheelOn = targetRpm > 0;
    }

    private double updateRamp(double current, double target, double dt) {
        double error = target - current;
        if (Math.abs(error) < 1.0)
            return target;
        double sign = Math.signum(error);
        double rate = BjornConstants.Power.RAMP_COEF * Math.abs(current) * Math.abs(error)
                + BjornConstants.Power.RAMP_MIN_RATE;
        double step = rate * dt;
        if (Math.abs(step) > Math.abs(error))
            return target;
        return current + sign * step;
    }

    // ---------------- Scan utils ----------------
    private void beginScan() {
        Arrays.fill(scan, 0.0);
        scanLeft = Math.min(SCAN_SAMPLES, scan.length);
    }

    private void setWheelPower(double power) {
        Wheel.setPower(power);
        if (Wheel2 != null) {
            Wheel2.setPower(power);
        }
    }

    private double safeAssemblyVel() {
        double v1 = safeVel(Wheel);
        double v2 = safeVel(Wheel2);
        return (Wheel2 != null) ? 0.5 * (v1 + v2) : v1;
    }

    private static double safeVel(DcMotorEx m) {
        if (m == null) {
            return 0.0;
        }
        try {
            return m.getVelocity();
        } catch (Exception e) {
            return 0.0;
        }
    }

    private static double toRPM(double ticksPerSec, double ticksPerRev) {
        return (ticksPerRev <= 0) ? 0.0 : (ticksPerSec / ticksPerRev) * 60.0;
    }

    private static double safeTofInches(DistanceSensor ds) {
        try {
            double d = ds.getDistance(DistanceUnit.INCH);
            return (Double.isNaN(d) || d <= 0) ? -1.0 : d;
        } catch (Exception e) {
            return -1.0;
        }
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double median(double[] a, int n) {
        double[] b = Arrays.copyOf(a, n);
        Arrays.sort(b);
        return (n % 2 == 1) ? b[n / 2] : 0.5 * (b[n / 2 - 1] + b[n / 2]);
    }

    private void shutdown() {
        Intake.setPower(0);
        setWheelRPM(0);
        if (USE_LIFT)
            Lift.setPosition(LIFT_LOWERED);
        if (camera != null)
            camera.close();
    }
}
// Certified Dylen Vasquez Design

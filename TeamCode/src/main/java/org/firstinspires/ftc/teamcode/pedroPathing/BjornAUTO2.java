package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

/**
 * BjornAUTO2 — TeleOp-inspired autonomous with dynamic flywheel RPM and intake pulses.
 *
 * New path plan (inches, headings in degrees):
 *   A)  ( 6, 35, -95)  → Activate shooter (pulse intake, dynamic RPM)
 *   A1) alignment set
 *   B)  (18, 44, -55)  → Intake starts
 *   C)  (37, -5, -55)  → Grab set; Wheel to idle
 *       (On next leg start: Intake OFF)
 *   D)  (18, 44, -55)  → Return/Reverse leg
 *   E)  ( 6, 35, -95)  → Activate shooter (pulse intake, dynamic RPM)
 *   F)  (50, 40, -55)  → Intake starts (second set)
 *   G)  (37, 60, -55)  → Grab set; Wheel on (idle)
 *   H)  ( 6, 35, -95)  → Activate shooter (pulse intake, dynamic RPM)
 *   I)  (30, 52, -145) → Leave / park
 *
 * Tunables are grouped below (RPMs, timings, mapping slope/intercept, scan samples, etc.).
 * Distance→RPM logic mirrors TeleOp "B" behavior: median TOF scan → linear RPM model → clamp/hysteresis.
 */
@Autonomous(name = "BjornAUTO2 (Tele shooter + dyn RPM)")
public class BjornAUTO2 extends OpMode {

    // ---------------- Hardware ----------------
    private Follower follower;
    private DcMotorEx Intake, Wheel;
    private Servo Lift;
    private DistanceSensor tof;

    // ---------------- Tunables ----------------
    // Speeds
    private static double DRIVE_SPEED_NORMAL = 1.00;
    private static double DRIVE_SPEED_ALIGN  = 0.70; // approaching shoot zone

    // Wheel RPMs
    private static double WHEEL_IDLE_RPM     = 1200;   // flywheel idle while navigating or staging
    private static double WHEEL_MAX_RPM      = 3600;   // maximum allowed
    private static double WHEEL_MIN_RPM      = 1700;   // minimum usable for launches

    // Intake power
    private static double INTAKE_POWER       = 1.00;

    // Intake pulse pattern when shooting
    private static long   INTAKE_PULSE_ON_MS  = 1000L; // 1s on
    private static long   INTAKE_PULSE_OFF_MS = 3000L; // 3s off between pulses
    private static long   SHOOT_WINDOW_MS     = 5000L; // total time for a shoot phase (may span more than one pulse)

    // Lift positions (if used)
    private static double LIFT_LOWERED = 0.10;
    private static double LIFT_RAISED  = 0.85;
    private static boolean USE_LIFT    = true; // flip to false if no lift gate

    // Distance→RPM mapping (feet)
    //   rpm = M_RPM_PER_FT * feet + B_RPM_OFFSET  (clamped to [WHEEL_MIN_RPM, WHEEL_MAX_RPM])
    private static double M_RPM_PER_FT = 400.0;
    private static double B_RPM_OFFSET = 1400.0;

    // Hysteresis (for deciding if wheel is "ready")
    private static double READY_ON_RPM  = 2700; // consider ready when >= this
    private static double READY_OFF_RPM = 2500; // drop ready if <= this

    // Scan parameters
    private static int    SCAN_SAMPLES = 7;   // median filter size
    private static double SENSOR_OFFSET_FT = 0.0; // add small offset if sensor is not at muzzle

    // Encoder math if you convert to velocity control
    private static double MOTOR_ENCODER_CPR = 28.0;
    private static double GEAR_RATIO        = 1.0;
    private static double TPR               = MOTOR_ENCODER_CPR * GEAR_RATIO;

    // ---------------- Poses ----------------
    private static final Pose START       = pose(0, 0, 265);
    private static final Pose SHOOT_ZONE  = pose( 3.5, 31, -95);
    private static final Pose ALIGN1      = pose(15, 44.5, -55);
    private static final Pose GRAB1       = pose(32, 22.6, -55);
    private static final Pose ALIGN1_BACK = pose(18, 44, -55);
    private static final Pose ALIGN2      = pose(33.8, 60, -55);
    private static final Pose GRAB2       = pose(52, 37, -55);
    private static final Pose PARK        = pose(24, 52, -145);

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

    // Shooter controller
    private double targetRpm = 0;
    private boolean wheelOn = false;
    private boolean ready = false;

    // Pulse timer
    private long shootPhaseStart = -1;
    private long pulseAnchor = -1; // base timestamp for pulses
    private boolean intakeOn = false;

    // Scan buffer
    private int scanLeft = 0;
    private final double[] scan = new double[16];

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(START.getX(), START.getY(), START.getHeading()));

        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Wheel  = hardwareMap.get(DcMotorEx.class, "Wheel");
        Lift   = hardwareMap.get(Servo.class,     "Lift");
        tof    = hardwareMap.get(DistanceSensor.class, "TOF");

        // Motor directions & behaviors
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setDirection(DcMotor.Direction.REVERSE);
        Wheel.setDirection(DcMotor.Direction.REVERSE);

        // Build paths
        toShoot      = line(START,      SHOOT_ZONE);
        toAlign1     = line(SHOOT_ZONE, ALIGN1);
        toGrab1      = line(ALIGN1,     GRAB1);
        toAlign1Back = line(GRAB1,      ALIGN1_BACK);
        toShoot2     = line(ALIGN1_BACK, SHOOT_ZONE);
        toAlign2     = line(SHOOT_ZONE, ALIGN2);
        toGrab2      = line(ALIGN2,     GRAB2);
        toShoot3     = line(GRAB2,      SHOOT_ZONE);
        toPark       = line(SHOOT_ZONE, PARK);

        // Init mechanisms
        setWheelRPM(0);
        Intake.setPower(0);
        if (USE_LIFT) Lift.setPosition(LIFT_LOWERED);

        // Start
        setDriveSpeed(DRIVE_SPEED_ALIGN);
        follower.followPath(toShoot);
        state = State.TO_SHOOT;
    }

    @Override
    public void loop() {
        follower.update();

        switch (state) {
            case TO_SHOOT:
                if (follower.atParametricEnd()) { beginShootPhase(); state = State.SHOOT1; }
                break;

            case SHOOT1:
                if (runShootPhase()) { setDriveSpeed(DRIVE_SPEED_NORMAL); follower.followPath(toAlign1, true); state = State.TO_ALIGN1; }
                break;

            case TO_ALIGN1:
                if (follower.atParametricEnd()) { Intake.setPower(INTAKE_POWER); follower.followPath(toGrab1, true); state = State.TO_GRAB1; }
                break;

            case TO_GRAB1:
                if (follower.atParametricEnd()) { setWheelRPM(WHEEL_IDLE_RPM); follower.followPath(toAlign1Back, true); Intake.setPower(0); state = State.TO_ALIGN1_BACK; }
                break;

            case TO_ALIGN1_BACK:
                if (follower.atParametricEnd()) { follower.followPath(toShoot2, true); state = State.TO_SHOOT2; }
                break;

            case TO_SHOOT2:
                if (follower.atParametricEnd()) { beginShootPhase(); state = State.SHOOT2; }
                break;

            case SHOOT2:
                if (runShootPhase()) { follower.followPath(toAlign2, true); state = State.TO_ALIGN2; }
                break;

            case TO_ALIGN2:
                if (follower.atParametricEnd()) { Intake.setPower(INTAKE_POWER); follower.followPath(toGrab2, true); state = State.TO_GRAB2; }
                break;

            case TO_GRAB2:
                if (follower.atParametricEnd()) { setWheelRPM(WHEEL_IDLE_RPM); Intake.setPower(0); follower.followPath(toShoot3, true); state = State.TO_SHOOT3; }
                break;

            case TO_SHOOT3:
                if (follower.atParametricEnd()) { beginShootPhase(); state = State.SHOOT3; }
                break;

            case SHOOT3:
                if (runShootPhase()) { follower.followPath(toPark, true); state = State.TO_PARK; }
                break;

            case TO_PARK:
                if (follower.atParametricEnd()) { shutdown(); state = State.DONE; }
                break;

            case DONE:
                // hold final pose
                break;
        }

        // Minimal telemetry
        telemetry.addData("State", state);
        telemetry.addData("RPM target", (int)targetRpm);
        telemetry.addData("Wheel ready", ready);
        telemetry.update();
    }

    // ---------------- Shooter phases ----------------
    private void beginShootPhase() {
        shootPhaseStart = System.currentTimeMillis();
        pulseAnchor     = shootPhaseStart;
        intakeOn        = false;
        // start wheel at idle, then scan to dynamic RPM
        setWheelRPM(WHEEL_IDLE_RPM);
        if (USE_LIFT) Lift.setPosition(LIFT_LOWERED);
        beginScan();
    }

    /** Returns true when the shoot window completes. */
    private boolean runShootPhase() {
        long now = System.currentTimeMillis();
        long elapsed = now - shootPhaseStart;

        // 1) Finish scan → compute dynamic RPM once
        if (scanLeft > 0) {
            double inches = safeTofInches(tof);
            scan[SCAN_SAMPLES - scanLeft] = inches;
            scanLeft--;
            if (scanLeft == 0) {
                double inchesMed = median(scan, SCAN_SAMPLES);
                if (inchesMed > 6 && inchesMed <= 120) {
                    double ft = inchesMed / 12.0 + SENSOR_OFFSET_FT;
                    double dyn = clamp(M_RPM_PER_FT * ft + B_RPM_OFFSET, WHEEL_MIN_RPM, WHEEL_MAX_RPM);
                    setWheelRPM(dyn);
                } else {
                    setWheelRPM((WHEEL_MIN_RPM + WHEEL_MAX_RPM) * 0.5);
                }
            }
        }

        // 2) Simple readiness check with hysteresis
        double wheelRpm = toRPM(safeVel(Wheel), TPR);
        if (!ready && wheelRpm >= READY_ON_RPM) ready = true;
        if (ready && wheelRpm <= READY_OFF_RPM) ready = false;

        // 3) Optional lift behavior
        if (USE_LIFT) Lift.setPosition(ready ? LIFT_RAISED : LIFT_LOWERED);

        // 4) Intake pulsing once wheel is ready (Tele-style bursts)
        if (ready) {
            long sinceAnchor = now - pulseAnchor;
            long period = INTAKE_PULSE_ON_MS + INTAKE_PULSE_OFF_MS;
            long m = sinceAnchor % period;
            boolean shouldBeOn = (m < INTAKE_PULSE_ON_MS);
            if (shouldBeOn != intakeOn) {
                intakeOn = shouldBeOn;
                Intake.setPower(intakeOn ? INTAKE_POWER : 0.0);
            }
        } else {
            // keep intake off until ready
            if (intakeOn) { intakeOn = false; Intake.setPower(0.0); }
        }

        // 5) End of shoot window
        if (elapsed >= SHOOT_WINDOW_MS) {
            Intake.setPower(0.0);
            if (USE_LIFT) Lift.setPosition(LIFT_LOWERED);
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
        try { follower.setMaxPower(pwr); } catch (Throwable ignored) {}
    }

    // ---------------- Wheel control ----------------
    private void setWheelRPM(double rpm) {
        targetRpm = Math.max(0, rpm);
        // If you have RUN_USING_ENCODER+PIDF, replace this with setVelocity(tps) using TPR.
        double pwr = (rpm <= 0) ? 0.0 : clamp(rpm / WHEEL_MAX_RPM, 0.0, 1.0);
        Wheel.setPower(pwr);
        wheelOn = pwr > 0.02;
    }

    // ---------------- Scan utils ----------------
    private void beginScan() { Arrays.fill(scan, 0.0); scanLeft = Math.min(SCAN_SAMPLES, scan.length); }

    private static double safeVel(DcMotorEx m) { try { return m.getVelocity(); } catch (Exception e) { return 0.0; } }
    private static double toRPM(double ticksPerSec, double ticksPerRev) { return (ticksPerRev <= 0) ? 0.0 : (ticksPerSec / ticksPerRev) * 60.0; }
    private static double safeTofInches(DistanceSensor ds) {
        try {
            double d = ds.getDistance(DistanceUnit.INCH);
            return (Double.isNaN(d) || d <= 0) ? -1.0 : d;
        } catch (Exception e) { return -1.0; }
    }

    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }

    private static double median(double[] a, int n) {
        double[] b = Arrays.copyOf(a, n);
        Arrays.sort(b);
        return (n % 2 == 1) ? b[n/2] : 0.5 * (b[n/2 - 1] + b[n/2]);
    }

    private void shutdown() {
        Intake.setPower(0);
        setWheelRPM(0);
        if (USE_LIFT) Lift.setPosition(LIFT_LOWERED);
    }
}

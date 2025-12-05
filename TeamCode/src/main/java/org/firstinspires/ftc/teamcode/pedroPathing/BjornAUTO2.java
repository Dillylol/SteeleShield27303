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
 * Adds:
 *  • **1.5s delay** between lift opening and intake turning on (prevents intercepts).
 *  • Telemetry shows **current RPM** and **target RPM**.
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
 */
@Autonomous(name = "DONTUSE")
public class BjornAUTO2 extends OpMode {

    // ---------------- Hardware ----------------
    private Follower follower;
    private DcMotorEx Intake, Wheel, Wheel2;
    private Servo Lift;
    private DistanceSensor tof;

    // ---------------- Tunables ----------------
    // Speeds
    private static double DRIVE_SPEED_NORMAL = 1.00;
    private static double DRIVE_SPEED_ALIGN  = 0.70; // approaching shoot zone

    // Wheel RPMs
    private static double WHEEL_IDLE_RPM     = 2000;   // flywheel idle while navigating or staging
    private static double WHEEL_MAX_RPM      = 4000;   // maximum allowed
    private static double WHEEL_MIN_RPM      = 2200;   // minimum usable for launches

    // Intake power
    private static double INTAKE_POWER       = 1.00;

    // Intake pulse pattern when shooting
    private static long   INTAKE_PULSE_ON_MS  = 1500L; // on window
    private static long   INTAKE_PULSE_OFF_MS = 500L; // off window between pulses
    private static long   SHOOT_WINDOW_MS     = 10000L; // total time for a shoot phase

    // Lift positions (if used)
    private static double LIFT_LOWERED = 0.10;
    private static double LIFT_RAISED  = 0.65;
    private static boolean USE_LIFT    = true; // flip to false if no lift gate

    // Distance→RPM mapping (feet)
    //   rpm = M_RPM_PER_FT * feet + B_RPM_OFFSET  (clamped to [WHEEL_MIN_RPM, WHEEL_MAX_RPM])
    private static double M_RPM_PER_FT = 116.4042383594456;
    private static double B_RPM_OFFSET = 2084.2966941424975;

    // Hysteresis (for deciding if wheel is "ready")
    private static double READY_ON_RPM  = 2200; // consider ready when >= this
    private static double READY_OFF_RPM = 2100; // drop ready if <= this

    // Scan parameters
    private static int    SCAN_SAMPLES = 15;   // median filter size
    private static double SENSOR_OFFSET_FT = 0.0; // add small offset if sensor is not at muzzle

    // Encoder math if you convert to velocity control
    private static double MOTOR_ENCODER_CPR = 28.0;
    private static double GEAR_RATIO        = 1.0;
    private static double TPR               = MOTOR_ENCODER_CPR * GEAR_RATIO;

    // Delay between lift opening and intake start (ms)
    private static final long INTAKE_DELAY_AFTER_LIFT_MS = 100L; // **1.5 seconds**

    // ---------------- Poses ----------------
    private static final Pose START       = pose(0, 0, 265); //129
    private static final Pose SHOOT_ZONE  = pose( 4.5, 33, -95);
    private static final Pose ALIGN1      = pose(23, 36, -58);
    private static final Pose GRAB1       = pose(30.2, 17.8, -58);
    private static final Pose ALIGN1_BACK = pose(23, 36, -58);
    private static final Pose ALIGN2      = pose(35.3, 52.8, -58);
    private static final Pose GRAB2       = pose(47, 32.3, -58);
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
    private boolean ready = false;

    // Timing
    private long shootPhaseStart = -1;  // shoot-phase start time
    private long pulseAnchor = -1;      // base timestamp for pulses
    private long readySinceMs = -1;     // first moment wheel became ready (for intake delay)

    // Intake state
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
        Wheel2 = hardwareMap.get(DcMotorEx.class, "Wheel2");
        Lift   = hardwareMap.get(Servo.class,     "Lift");
        tof    = hardwareMap.get(DistanceSensor.class, "TOF");

        // Motor behaviors
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setDirection(DcMotor.Direction.REVERSE);
        Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Wheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Wheel.setDirection(DcMotor.Direction.REVERSE);
        Wheel2.setDirection(DcMotor.Direction.REVERSE);

        // Paths
        toShoot      = line(START,       SHOOT_ZONE);
        toAlign1     = line(SHOOT_ZONE,  ALIGN1);
        toGrab1      = line(ALIGN1,      GRAB1);
        toAlign1Back = line(GRAB1,       ALIGN1_BACK);
        toShoot2     = line(ALIGN1_BACK, SHOOT_ZONE);
        toAlign2     = line(SHOOT_ZONE,  ALIGN2);
        toGrab2      = line(ALIGN2,      GRAB2);
        toShoot3     = line(GRAB2,       SHOOT_ZONE);
        toPark       = line(SHOOT_ZONE,  PARK);

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
                break;
        }

        // Telemetry (now includes live RPM)
        double currentRpm = toRPM(safeAssemblyVel(), TPR);
        telemetry.addData("State", state);
        telemetry.addData("RPM target", (int) targetRpm);
        telemetry.addData("RPM current", (int) currentRpm);
        telemetry.addData("Wheel ready", ready);
        telemetry.update();
    }

    // ---------------- Shooter phases ----------------
    private void beginShootPhase() {
        shootPhaseStart = System.currentTimeMillis();
        pulseAnchor     = shootPhaseStart;
        readySinceMs    = -1;
        intakeOn        = false;
        ready           = false;

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

        // 2) Readiness check with hysteresis
        double wheelRpm = toRPM(safeAssemblyVel(), TPR);
        if (!ready && wheelRpm >= READY_ON_RPM) {
            ready = true;
            readySinceMs = now; // mark the moment the wheel is considered ready
            if (USE_LIFT) Lift.setPosition(LIFT_RAISED); // open lift now
        }
        if (ready && wheelRpm <= READY_OFF_RPM) {
            ready = false; // dropped out of ready band
            readySinceMs = -1;
            if (USE_LIFT) Lift.setPosition(LIFT_LOWERED);
        }

        // 3) Intake pulsing only AFTER delay from lift open
        boolean pastDelay = ready && readySinceMs > 0 && (now - readySinceMs) >= INTAKE_DELAY_AFTER_LIFT_MS;
        if (pastDelay) {
            long sinceAnchor = now - pulseAnchor;
            long period = INTAKE_PULSE_ON_MS + INTAKE_PULSE_OFF_MS;
            long m = sinceAnchor % period;
            boolean shouldBeOn = (m < INTAKE_PULSE_ON_MS);
            if (shouldBeOn != intakeOn) {
                intakeOn = shouldBeOn;
                Intake.setPower(intakeOn ? INTAKE_POWER : 0.0);
            }
        } else {
            // keep intake off until the 1.5s delay has elapsed
            if (intakeOn) { intakeOn = false; Intake.setPower(0.0); }
        }

        // 4) End of shoot window
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
        setWheelPower(pwr);
    }

    // ---------------- Scan utils ----------------
    private void beginScan() { Arrays.fill(scan, 0.0); scanLeft = Math.min(SCAN_SAMPLES, scan.length); }

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
        try { return m.getVelocity(); } catch (Exception e) { return 0.0; }
    }
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

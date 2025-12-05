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
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

/**
 * BjornAUTO2 — Dual-stage flywheel control:
 *   1) Legacy velocity chaser (battery-compensated setVelocity) to ramp fast.
 *   2) Automatic handover to PIDF "lock" at the same target once stable.
 *
 * Pathing, lift behavior, and intake controls mirror latest versions.
 * Flywheel only is upgraded to dual-stage control.
 */
@Autonomous(name = "or this")
public class BjornAUTO2_DualStage extends OpMode {
    // ---------------- Hardware ----------------
    private Follower follower;
    private DcMotorEx Intake, Wheel, Wheel2;
    private Servo Lift;
    private DistanceSensor tof;

    // ---------------- Tunables ----------------
    // Drive
    private static final double DRIVE_SPEED_ALIGN  = 0.70;
    private static final double DRIVE_SPEED_NORMAL = 1.00;

    // Lift (from latest Tele)
    private static final double LIFT_LOWERED_POS = 0.10;
    private static final double LIFT_RAISED_POS  = 0.65;

    // Intake
    private static final double INTAKE_POWER = 1.0;

    // Wheel gearing/units
    private static final double MOTOR_ENCODER_CPR = 28.0;
    private static final double WHEEL_GEAR_RATIO  = 1.0; // set yours
    private static final double WHEEL_TPR         = MOTOR_ENCODER_CPR * WHEEL_GEAR_RATIO;

    // RPM limits
    private static final double RPM_MIN = 2200.0;
    private static final double RPM_MAX = 4000.0;
    private static final double IDLE_RPM = 2000.0;

    // Distance→RPM linear model (feet)
    // rpm = M_RPM_PER_FT * feet + B_RPM_OFFSET
    private static final double M_RPM_PER_FT = 116.4042383594456;
    private static final double B_RPM_OFFSET = 2084.2966941424975;
    private static final double SENSOR_OFFSET_FT = 0.0;

    // Scan
    private static final int SCAN_SAMPLES = 15;

    // ===== Dual‑stage handover parameters =====
    // Stage 1: legacy chaser target is slightly below final setpoint to avoid overshoot.
    private static final double CHASER_OFFSET_RPM = 120.0;      // chase target = dyn - this
    private static final double CHASER_MIN_RPM    = 1800.0;     // don't chase below this
    // Handover: require band and dwell
    private static final double HANDOVER_BAND_RPM = 40.0;       // |meas - target| <= band
    private static final long   HANDOVER_HOLD_MS  = 350;        // must hold band this long
    // Safety: drop back to chaser if we fall out of band for too long
    private static final long   RELOCK_GRACE_MS   = 250;        // allowed band miss before fallback

    // "Ready" gate (for lift/intake control)
    private static final double READY_TOL_RPM = 60.0;           // allow intake/lift when within this band

    // --------------- Latest poses (from uploaded auton) ---------------
    private static final Pose START       = pose(0, 0, 265);
    private static final Pose SHOOT_ZONE  = pose( 4.5, 33, -95);
    private static final Pose ALIGN1      = pose(23, 36, -58);
    private static final Pose GRAB1       = pose(30.2, 17.8, -58);
    private static final Pose ALIGN1_BACK = pose(23, 36, -58);
    private static final Pose ALIGN2      = pose(35.3, 52.8, -58);
    private static final Pose GRAB2       = pose(47, 32.3, -58);
    private static final Pose PARK        = pose(24, 52, -145);

    // Paths
    private PathChain toShoot, toAlign1, toGrab1, toAlign1Back, toShoot2, toAlign2, toGrab2, toShoot3, toPark;

    // State machine
    private enum State {
        TO_SHOOT, SHOOT1,
        TO_ALIGN1, TO_GRAB1, TO_ALIGN1_BACK,
        TO_SHOOT2, SHOOT2,
        TO_ALIGN2, TO_GRAB2,
        TO_SHOOT3, SHOOT3,
        TO_PARK, DONE
    }
    private State state;

    // Shooting phase
    private long shootPhaseStartMs = -1;
    private static final long SHOOT_WINDOW_MS = 10000L;

    // Scan buffer
    private int scanLeft = 0;
    private final double[] scan = new double[SCAN_SAMPLES];

    // Dynamic target RPM (computed per shoot)
    private double dynTargetRPM = RPM_MIN;

    // Dual‑stage controller
    private enum WheelMode { OFF, CHASER, PID_LOCK }
    private WheelMode wheelMode = WheelMode.OFF;
    private long bandEnterMs = -1;
    private long lastBandMissMs = -1;
    private boolean liftIsRaised = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(START.getX(), START.getY(), START.getHeading()));

        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Wheel  = hardwareMap.get(DcMotorEx.class, "Wheel");
        Wheel2 = hardwareMap.get(DcMotorEx.class, "Wheel2");
        Lift   = hardwareMap.get(Servo.class,     "Lift");
        tof    = hardwareMap.get(DistanceSensor.class, "TOF");

        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Wheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setDirection(DcMotor.Direction.REVERSE);
        Wheel.setDirection(DcMotor.Direction.REVERSE);
        Wheel2.setDirection(DcMotor.Direction.REVERSE);

        // Build latest paths
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
        Intake.setPower(0);
        try { Lift.setPosition(LIFT_LOWERED_POS); } catch (Exception ignored) {}

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
                if (follower.atParametricEnd()) { setWheelOff(); follower.followPath(toAlign1Back, true); Intake.setPower(0); state = State.TO_ALIGN1_BACK; }
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
                if (follower.atParametricEnd()) { setWheelOff(); Intake.setPower(0); follower.followPath(toShoot3, true); state = State.TO_SHOOT3; }
                break;

            case TO_SHOOT3:
                if (follower.atParametricEnd()) { beginShootPhase(); state = State.SHOOT3; }
                break;

            case SHOOT3:
                if (runShootPhase()) { follower.followPath(toPark, true); state = State.TO_PARK; }
                break;

            case TO_PARK:
                if (follower.atParametricEnd()) { setWheelOff(); shutdown(); state = State.DONE; }
                break;

            case DONE:
                break;
        }

        // Telemetry
        final double wheelTps  = safeAssemblyVel();
        final double wheelRpm  = toRPM(wheelTps, WHEEL_TPR);
        telemetry.addData("State", state);
        telemetry.addData("WheelMode", wheelMode);
        telemetry.addData("dynTargetRPM", (int)dynTargetRPM);
        telemetry.addData("measRPM", (int)wheelRpm);
        telemetry.update();
    }

    // ---------- Shoot phase ----------
    private void beginShootPhase() {
        shootPhaseStartMs = System.currentTimeMillis();
        // Prepare: lower lift, compute dynamic target via scan
        try { Lift.setPosition(LIFT_LOWERED_POS); } catch (Exception ignored) {}
        Intake.setPower(0);
        beginScan();
        dynTargetRPM = RPM_MIN; // will update after scan completes
        // Start wheel in CHASER
        ensureRunUsingEncoder();
        wheelMode = WheelMode.CHASER;
        bandEnterMs = -1;
        lastBandMissMs = -1;
    }

    private boolean runShootPhase() {
        long nowMs = System.currentTimeMillis();

        // 1) Finish scan to establish dynTargetRPM
        if (scanLeft > 0) {
            double inches = safeTofInches(tof);
            scan[SCAN_SAMPLES - scanLeft] = inches;
            scanLeft--;
            if (scanLeft == 0) {
                double inchesMed = median(scan, SCAN_SAMPLES);
                if (inchesMed > 6 && inchesMed <= 120) {
                    double ft = inchesMed/12.0 + SENSOR_OFFSET_FT;
                    dynTargetRPM = clamp(M_RPM_PER_FT * ft + B_RPM_OFFSET, RPM_MIN, RPM_MAX);
                } else {
                    dynTargetRPM = (RPM_MIN + RPM_MAX) * 0.5;
                }
            }
        }

        // 2) Update wheel control (dual‑stage)
        final double wheelTps  = safeAssemblyVel();
        final double measRPM   = toRPM(wheelTps, WHEEL_TPR);
        final boolean inBand   = Math.abs(measRPM - dynTargetRPM) <= HANDOVER_BAND_RPM;

        // Battery compensation (legacy)
        double v = getBatteryVoltage();
        double scale = 12.5 / Math.max(10.0, v);

        switch (wheelMode) {
            case OFF:
                setWheelPower(0.0);
                break;

            case CHASER: {
                double chaseRPM = Math.max(CHASER_MIN_RPM, dynTargetRPM - CHASER_OFFSET_RPM);
                double tps = (chaseRPM / 60.0) * WHEEL_TPR * scale;
                ensureRunUsingEncoder();
                setWheelVelocity(tps);

                if (inBand) {
                    if (bandEnterMs < 0) bandEnterMs = nowMs;
                    if (nowMs - bandEnterMs >= HANDOVER_HOLD_MS) {
                        wheelMode = WheelMode.PID_LOCK;
                        bandEnterMs = -1;
                        lastBandMissMs = -1;
                    }
                } else {
                    bandEnterMs = -1;
                }
                break;
            }

            case PID_LOCK: {
                ensureRunUsingEncoder();
                double tps = (dynTargetRPM / 60.0) * WHEEL_TPR; // handover uses true setpoint (no battery scale)
                setWheelVelocity(tps);
                if (!inBand) {
                    if (lastBandMissMs < 0) lastBandMissMs = nowMs;
                    if (nowMs - lastBandMissMs > RELOCK_GRACE_MS) {
                        // fall back to chaser to re‑acquire
                        wheelMode = WheelMode.CHASER;
                        bandEnterMs = -1;
                        lastBandMissMs = -1;
                    }
                } else {
                    lastBandMissMs = -1;
                }
                break;
            }
        }

        // 3) Lift/intake when effectively "ready"
        boolean ready = Math.abs(measRPM - dynTargetRPM) <= READY_TOL_RPM;
        if (ready) {
            if (!liftIsRaised) {
                try { Lift.setPosition(LIFT_RAISED_POS); } catch (Exception ignored) {}
                liftIsRaised = true;
            }
            Intake.setPower(INTAKE_POWER);
        } else {
            Intake.setPower(0);
            if (liftIsRaised) {
                try { Lift.setPosition(LIFT_LOWERED_POS); } catch (Exception ignored) {}
                liftIsRaised = false;
            }
        }

        // 4) End condition
        if (nowMs - shootPhaseStartMs >= SHOOT_WINDOW_MS) {
            Intake.setPower(0);
            try { Lift.setPosition(LIFT_LOWERED_POS); } catch (Exception ignored) {}
            setWheelOff();
            return true;
        }
        return false;
    }

    // ---------- Wheel helpers ----------
    private void setWheelOff() {
        wheelMode = WheelMode.OFF;
        setWheelPower(0.0);
    }
    private void ensureRunUsingEncoder() {
        ensureRunUsingEncoder(Wheel);
        ensureRunUsingEncoder(Wheel2);
    }

    // ---------- Pedro helpers ----------
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

    // ---------- Scan + math ----------
    private void beginScan() {
        Arrays.fill(scan, 0.0);
        scanLeft = SCAN_SAMPLES;
    }
    private void setWheelPower(double power) {
        Wheel.setPower(power);
        if (Wheel2 != null) {
            Wheel2.setPower(power);
        }
    }

    private void setWheelVelocity(double ticksPerSecond) {
        setVelocitySafe(Wheel, ticksPerSecond);
        setVelocitySafe(Wheel2, ticksPerSecond);
    }

    private void ensureRunUsingEncoder(DcMotorEx motor) {
        if (motor != null && motor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    private static void setVelocitySafe(DcMotorEx motor, double ticksPerSecond) {
        if (motor == null) {
            return;
        }
        try {
            motor.setVelocity(ticksPerSecond);
        } catch (Exception ignored) {
        }
    }
    private static double toRPM(double tps, double tpr) { return (tpr <= 0) ? 0.0 : (tps / tpr) * 60.0; }
    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    private static double median(double[] a, int n) {
        double[] b = Arrays.copyOf(a, n);
        Arrays.sort(b);
        return (n % 2 == 1) ? b[n/2] : 0.5 * (b[n/2 - 1] + b[n/2]);
    }
    private static double safeTofInches(DistanceSensor ds) {
        try {
            double d = ds.getDistance(DistanceUnit.INCH);
            return (Double.isNaN(d) || d <= 0) ? -1.0 : d;
        } catch (Exception e) { return -1.0; }
    }

    // ---------- Battery ----------
    private double getBatteryVoltage() {
        double best = 0.0;
        for (VoltageSensor s : hardwareMap.getAll(VoltageSensor.class)) {
            double v = s.getVoltage();
            if (!Double.isNaN(v)) best = Math.max(best, v);
        }
        return (best > 0.0) ? best : 12.0;
    }

    private void shutdown() {
        Intake.setPower(0);
        setWheelOff();
        try { Lift.setPosition(LIFT_LOWERED_POS); } catch (Exception ignored) {}
    }
}

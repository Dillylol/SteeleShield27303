package org.firstinspires.ftc.teamcode.pedroPathing;

// import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

/**
 * BjornAUTO2 — Unified auto that drives like TeleOp paths and launches using the same logic as Tele.
 *
 * Update per request:
 *   • START is now (0, 0, 0)
 *   • Insert START_STAGE immediately after START to preserve the previous run‑up behavior
 *     using coordinates (38, -22, 3.75°)
 *   • Update PICK1 and REVERSE1 to (25, -30, 3.75°) and (38, -20, 3.75°)
 *
 * Sequence now (inches, headings in degrees):
 *   START(0,0,0) → START_STAGE(38,-22,3.75) → PICK1(25,-30,3.75) → REVERSE1(38,-20,3.75) → LINEUP1(6,30,-90) → LAUNCH
 *   BACK_START(15,30,33) → PICK2(53,40,-55) → REVERSE2(43,53,-55) → LINEUP2(6,30,-90) → LAUNCH → SCRAM(28,50,35)
 */
@Autonomous(name = "BjornAUTO2")
public class BjornAUTO2 extends OpMode {

    // ---------------- Hardware ----------------
    private DcMotorEx Intake, Wheel;
    private Servo Lift;
    private DistanceSensor tofFront;
    private Follower follower;

    // ---------------- Drive paths ----------------
    private PathChain toStage, toPick1, toReverse1, toLineup1, backToStart, toPick2, toReverse2, toLineup2, toScram;

    // ---------------- Tuning (match Tele where possible) ----------------
    // Intake & wheel
    private static final double INTAKE_POWER = 1.0;
    private static final double WHEEL_IDLE_RPM = 1200;      // idle while navigating
    private static final double WHEEL_PRESPIN_RPM = 2000;   // pre‑spin during reverse legs

    // RPM thresholds with hysteresis for lift gate
    private static final double WHEEL_READY_ON_RPM  = 2700; // raise ≥ this
    private static final double WHEEL_READY_OFF_RPM = 2500; // drop ≤ this

    // Encoder math for RPM conversion
    private static final double MOTOR_ENCODER_CPR = 28.0;
    private static final double WHEEL_GEAR_RATIO  = 1.0;
    private static final double WHEEL_TPR         = MOTOR_ENCODER_CPR * WHEEL_GEAR_RATIO;

    // Lift positions
    private static final double LIFT_LOWERED_POS  = 0.10;
    private static final double LIFT_RAISED_POS   = 0.85;

    // Launch timing
    private static final long LIFT_OPEN_FEED_DELAY_MS = 1500L; // wait after opening before feeding
    private static final long LAUNCH_PHASE_TIMEOUT_MS = 5000L; // whole launch window

    // Distance→RPM model (copy of Tele linear map)
    //   rpm = M * feet + B  (bounded to [RPM_MIN, RPM_MAX])
    private static final double M_RPM_PER_FT = 400.0; // tune to match Tele
    private static final double B_RPM_OFFSET = 1400.0; // tune to match Tele
    private static final double RPM_MIN = 1700.0;
    private static final double RPM_MAX = 3600.0;

    // TOF parameters
    private static final int     SCAN_SAMPLES = 7;    // median filter
    private static final double  SENSOR_TO_CANNON_OFFSET_FT = 0.0;

    // Poses (inches, headings in degrees → radians)
    private static final Pose START        = pose( 0,  0,   0);
    private static final Pose START_STAGE  = pose(38, -22,  3.75);
    private static final Pose PICK1        = pose(25, -30,  3.75);
    private static final Pose REVERSE1     = pose(38, -20,  3.75);
    private static final Pose LINEUP1      = pose( 6,  30, -90);
    private static final Pose BACK_START   = pose(15,  30,  33);
    private static final Pose PICK2        = pose(53,  40, -55);
    private static final Pose REVERSE2     = pose(43,  53, -55);
    private static final Pose LINEUP2      = pose( 6,  30, -90);
    private static final Pose SCRAM        = pose(28,  50,  35);

    // ---------------- Path Builder Helper ----------------
    /**
     * Some Pedro versions removed PathChain#addPath. Build via the follower's pathBuilder,
     * then return a PathChain.
     */
    private PathChain buildLine(Pose a, Pose b) {
        return follower.pathBuilder()
                .addPath(new BezierLine(a, b))
                .setLinearHeadingInterpolation(a.getHeading(), b.getHeading())
                .build();
    }


    // ---------------- State Machine ----------------
    private enum State {
        TO_STAGE, TO_PICK1, TO_REVERSE1, TO_LINEUP1, LAUNCH1,
        BACK_TO_START, TO_PICK2, TO_REVERSE2, TO_LINEUP2, LAUNCH2,
        TO_SCRAM, DONE
    }
    private State state;

    // Launcher controller (Tele-style but autonomous)
    private double targetWheelRPM = WHEEL_IDLE_RPM;
    private boolean wheelOn = false;
    private boolean liftOpened = false;
    private long liftOpenedAt = -1;
    private long launchStartedAt = -1;
    private int scanLeft = 0;
    private final double[] scanBuf = new double[SCAN_SAMPLES];

    // Telemetry helper
    // private PanelsTelemetry p; // removed; use SDK Telemetry directly

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(50);
        // p = new PanelsTelemetry(telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(START.getX(), START.getY(), START.getHeading()));

        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Wheel  = hardwareMap.get(DcMotorEx.class, "Wheel");
        Lift   = hardwareMap.get(Servo.class,     "Lift");
        tofFront = hardwareMap.get(DistanceSensor.class, "TOF");

        // Directions & behaviors (mirror Tele)
        Intake.setDirection(DcMotor.Direction.REVERSE);
        Wheel.setDirection(DcMotor.Direction.REVERSE);
        DcMotor.ZeroPowerBehavior brake = DcMotor.ZeroPowerBehavior.BRAKE;
        Intake.setZeroPowerBehavior(brake);
        Wheel.setZeroPowerBehavior(brake);

        // Build Pedro path chains (new first leg to START_STAGE)
        toStage      = buildLine(START,       START_STAGE);
        toPick1      = buildLine(START_STAGE, PICK1);
        toReverse1   = buildLine(PICK1,       REVERSE1);
        toLineup1    = buildLine(REVERSE1,    LINEUP1);
        backToStart  = buildLine(LINEUP1,     BACK_START);
        toPick2      = buildLine(BACK_START,  PICK2);
        toReverse2   = buildLine(PICK2,       REVERSE2);
        toLineup2    = buildLine(REVERSE2,    LINEUP2);
        toScram      = buildLine(LINEUP2,     SCRAM);

        // Initialize lift down & wheel idle
        Lift.setPosition(LIFT_LOWERED_POS);
        setWheelRPM(WHEEL_IDLE_RPM);
        wheelOn = true; // keep idling through auto

        state = State.TO_STAGE;
    }

    @Override
    public void loop() {
        // Run follower
        follower.update();

        switch (state) {
            case TO_STAGE:      if (followIfNeeded(toStage))    state = State.TO_PICK1; break;
            case TO_PICK1:      if (followIfNeeded(toPick1))    state = State.TO_REVERSE1; break;
            case TO_REVERSE1:   if (followIfNeeded(toReverse1)) { setWheelRPM(WHEEL_PRESPIN_RPM); state = State.TO_LINEUP1; } break;
            case TO_LINEUP1:    if (followIfNeeded(toLineup1))  state = beginLaunch(); break;

            case LAUNCH1:       if (runLaunchBlock())           state = State.BACK_TO_START; break;
            case BACK_TO_START: if (followIfNeeded(backToStart)) state = State.TO_PICK2; break;

            case TO_PICK2:      if (followIfNeeded(toPick2))    state = State.TO_REVERSE2; break;
            case TO_REVERSE2:   if (followIfNeeded(toReverse2)) { setWheelRPM(WHEEL_PRESPIN_RPM); state = State.TO_LINEUP2; } break;
            case TO_LINEUP2:    if (followIfNeeded(toLineup2))  state = beginLaunch(); break;

            case LAUNCH2:       if (runLaunchBlock())           state = State.TO_SCRAM; break;
            case TO_SCRAM:      if (followIfNeeded(toScram))    state = State.DONE; break;

            case DONE:
                Intake.setPower(0);
                setWheelRPM(0);
                break;
        }

        // Telemetry (simple)
        telemetry.addData("State", state);
        telemetry.addData("Target RPM", (int)targetWheelRPM);
        telemetry.addData("Wheel RPM", (int)toRPM(safeVel(Wheel), WHEEL_TPR));
        telemetry.addData("Lift Open?", liftOpened);
        telemetry.addData("ScanLeft", scanLeft);
        telemetry.update();
    }

    // -------------- Launch block (Tele-style) --------------
    private State beginLaunch() {
        // Kick off a scan and set timeout window
        beginScan();
        launchStartedAt = System.currentTimeMillis();
        liftOpened = false;
        liftOpenedAt = -1;
        // Stay idling until scan finishes and we compute dynamic RPM
        setWheelRPM(WHEEL_IDLE_RPM);
        return (state == State.TO_LINEUP1) ? State.LAUNCH1 : State.LAUNCH2;
    }

    /**
     * Returns true when the launch phase has completed (timeout or finished feed).
     */
    private boolean runLaunchBlock() {
        long now = System.currentTimeMillis();
        long elapsed = now - launchStartedAt;

        // 1) Collect scan → compute dynamic target once
        if (scanLeft > 0) {
            double inches = safeTofInches(tofFront);
            scanBuf[SCAN_SAMPLES - scanLeft] = inches;
            scanLeft--;
            if (scanLeft == 0) {
                double inchesMed = median(scanBuf);
                if (inchesMed > 6 && inchesMed <= 120) {
                    double ft = inchesMed / 12.0 + SENSOR_TO_CANNON_OFFSET_FT;
                    double dyn = clamp(rpmFromFeet(ft), RPM_MIN, RPM_MAX);
                    setWheelRPM(dyn);
                } else {
                    // Bad scan → fall back to a safe mid value
                    setWheelRPM( (RPM_MIN + RPM_MAX) * 0.5 );
                }
            }
        }

        // 2) Lift logic with RPM hysteresis
        double wheelRpm = toRPM(safeVel(Wheel), WHEEL_TPR);
        if (!liftOpened && wheelRpm >= WHEEL_READY_ON_RPM) {
            Lift.setPosition(LIFT_RAISED_POS);
            liftOpened = true;
            liftOpenedAt = now;
        }
        if (liftOpened && wheelRpm <= WHEEL_READY_OFF_RPM && (now - liftOpenedAt) < LIFT_OPEN_FEED_DELAY_MS) {
            // sagged before we even started feeding → drop and re-arm
            Lift.setPosition(LIFT_LOWERED_POS);
            liftOpened = false;
            liftOpenedAt = -1;
        }

        // 3) After lift open delay → feed intake full
        if (liftOpened && (now - liftOpenedAt) >= LIFT_OPEN_FEED_DELAY_MS) {
            Intake.setPower(INTAKE_POWER);
        }

        // 4) Timeout gate — after window ends, shut down and finish phase
        if (elapsed >= LAUNCH_PHASE_TIMEOUT_MS) {
            Intake.setPower(0);
            setWheelRPM(0); // wheel off after shot
            Lift.setPosition(LIFT_LOWERED_POS);
            return true;
        }
        return false;
    }

    // -------------- Pedro helpers --------------
    private boolean followIfNeeded(PathChain chain) {
        if (!follower.isBusy()) follower.followPath(chain, true);
        return !follower.isBusy();
    }

    // (unused) legacy helper removed; using buildLine above.

    private static Pose pose(double x, double y, double hDeg) {
        return new Pose(x, y, Math.toRadians(hDeg));
    }

    // -------------- Wheel control --------------
    private void setWheelRPM(double rpm) {
        // DcMotorEx runs by power by default; if using RUN_USING_ENCODER and PIDF, set velocity if configured.
        // Here we scale a 0..1 power from RPM band as a simple surrogate. Tune if you have velocity PID on.
        targetWheelRPM = rpm;
        double pwr = (rpm <= 0) ? 0 : clamp(rpm / RPM_MAX, 0.0, 1.0);
        Wheel.setPower(pwr);
        wheelOn = pwr > 0.02;
    }

    // -------------- Scan helpers (mirrors Tele) --------------
    private void beginScan() { Arrays.fill(scanBuf, 0.0); scanLeft = SCAN_SAMPLES; }

    private static double rpmFromFeet(double ft) { return M_RPM_PER_FT * ft + B_RPM_OFFSET; }

    // -------------- Utils --------------
    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }

    private static double median(double[] a) {
        double[] b = Arrays.copyOf(a, a.length);
        Arrays.sort(b);
        int n = b.length;
        return (n % 2 == 1) ? b[n/2] : 0.5*(b[n/2 - 1] + b[n/2]);
    }

    private static double safeVel(DcMotorEx m) { try { return m.getVelocity(); } catch (Exception e) { return 0.0; } }

    private static double toRPM(double tps, double tpr) { return (tpr <= 0) ? 0.0 : (tps / tpr) * 60.0; }

    private static double safeTofInches(DistanceSensor ds) {
        try {
            double d = ds.getDistance(DistanceUnit.INCH);
            return (Double.isNaN(d) || d <= 0) ? -1.0 : d;
        } catch (Exception e) { return -1.0; }
    }
}

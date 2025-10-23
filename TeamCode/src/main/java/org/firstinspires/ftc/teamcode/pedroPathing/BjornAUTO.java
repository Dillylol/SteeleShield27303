package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayDeque;
import java.util.Deque;

/**
 * BjornAUTO — Pedro V2 Panels auto with intake-on Phase 1 and safe launch at finish.
 */
@Autonomous(name = "BjornAUTO (V2 Panels + SafeLaunch)", group = "Pedro Pathing")
public class BjornAUTO extends OpMode {

    // ───────── Speeds / powers ─────────
    private static final double SPEED_P1_RUNUP    = 1.00;
    private static final double SPEED_P1_ROTATE   = 1.00;
    private static final double SPEED_P1_APPROACH = 0.50;
    private static final double SPEED_P1_SECURE   = 0.50;
    private static final double SPEED_P2_ALL      = 1.00;

    private static final double INTAKE_POWER      = 1.00;
    private static final double WHEEL_POWER_SHOOT = 1.00;

    // ───────── Wheel readiness (RPM, with hysteresis) ─────────
    private static final double WHEEL_READY_ON_RPM  = 2700; // raise ≥ this
    private static final double WHEEL_READY_OFF_RPM = 2500; // lower ≤ this

    // Encoder math for RPM conversion
    private static final double MOTOR_ENCODER_CPR = 28.0;
    private static final double WHEEL_GEAR_RATIO  = 1.0;
    private static final double WHEEL_TPR         = MOTOR_ENCODER_CPR * WHEEL_GEAR_RATIO;

    // ───────── Lift positions ─────────
    private static final double LIFT_LOWERED_POS  = 0.10;
    private static final double LIFT_RAISED_POS   = 0.85;

    // ───────── Poses ─────────
    private static final Pose START = new Pose(0, 0, Math.toRadians(90));
    private static final Pose WHITE_LINE_APPROACH = new Pose(0, 32.5, Math.toRadians(180.0));
    private static final Pose WHITE_LINE_SECURE   = new Pose(-19, 32.5, Math.toRadians(180.0));
    private static final Pose REVERSE_POSE        = new Pose(35, 32.5, Math.toRadians(130));
    private static final Pose BLUE_BOX_PRIME      = new Pose(35, 65, Math.toRadians(130));

    // Run‑up & rotation stages
    private static final double PRE_ROTATE_ADVANCE_IN = 8.0;
    private static final double ROTATE_STAGE_DELTA_IN = 2.0;
    private static final Pose PRE_ROTATE_STAGE = new Pose(START.getX(), START.getY() + PRE_ROTATE_ADVANCE_IN, START.getHeading());
    private static final Pose ROTATE_STAGE     = new Pose(PRE_ROTATE_STAGE.getX(), PRE_ROTATE_STAGE.getY() + ROTATE_STAGE_DELTA_IN, Math.toRadians(180.0));

    // Pedro
    private Follower follower;

    private PathChain P1_RUNUP_CHAIN, P1_ROTATE_CHAIN, P1_APPROACH_CHAIN, P1_SECURE_CHAIN;
    private PathChain P2_REVERSE_AND_BOX;

    // Hardware
    private DcMotorEx Intake;
    private DcMotorEx Wheel;
    private Servo     Lift;

    // Panels
    private final Telemetry panels = PanelsTelemetry.INSTANCE.getFtcTelemetry();
    private final Deque<Pose> poseHistory = new ArrayDeque<>(100);
    private static final int HISTORY_LIMIT = 60;

    // State
    private enum State { P1_RUNUP, P1_ROTATE, P1_APPROACH, P1_SECURE, P2_ALL, SPINUP_AND_LIFT, DONE }
    private State state = State.P1_RUNUP;
    private boolean liftRaised = false;

    // NEW: delay between lift raise and intake start
    private static final long INTAKE_AFTER_LIFT_DELAY_MS = 1500L;
    private long liftRaisedAtMs = -1L;
    private boolean intakeDelayedStarted = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(START.getX(), START.getY(), START.getHeading()));

        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Wheel  = hardwareMap.get(DcMotorEx.class, "Wheel");
        Lift   = hardwareMap.get(Servo.class,     "Lift");

        DcMotor.ZeroPowerBehavior brake = DcMotor.ZeroPowerBehavior.BRAKE;
        Intake.setZeroPowerBehavior(brake);
        Wheel.setZeroPowerBehavior(brake);
        Intake.setDirection(DcMotor.Direction.REVERSE);
        Wheel.setDirection(DcMotor.Direction.REVERSE);

        liftRaised = false;

        P1_RUNUP_CHAIN     = buildLine(START,              PRE_ROTATE_STAGE,    START.getHeading(),              PRE_ROTATE_STAGE.getHeading());
        P1_ROTATE_CHAIN    = buildLine(PRE_ROTATE_STAGE,   ROTATE_STAGE,        PRE_ROTATE_STAGE.getHeading(),   ROTATE_STAGE.getHeading());
        P1_APPROACH_CHAIN  = buildLine(ROTATE_STAGE,       WHITE_LINE_APPROACH, ROTATE_STAGE.getHeading(),       WHITE_LINE_APPROACH.getHeading());
        P1_SECURE_CHAIN    = buildLine(WHITE_LINE_APPROACH, WHITE_LINE_SECURE,  WHITE_LINE_APPROACH.getHeading(), WHITE_LINE_SECURE.getHeading());
        P2_REVERSE_AND_BOX = buildReverseAndBox(WHITE_LINE_SECURE, REVERSE_POSE, BLUE_BOX_PRIME);

        panels.addData("Hint", "Panels v2 output (pose/state/wheel) — waiting for START");
        panels.update();
        poseHistory.clear();
        poseHistory.add(new Pose(START.getX(), START.getY(), START.getHeading()));
    }

    @Override
    public void start() {
        setSpeed(SPEED_P1_RUNUP);
        follower.followPath(P1_RUNUP_CHAIN);
    }

    @Override
    public void loop() {
        follower.update();
        recordAndDrawPanels();

        switch (state) {
            case P1_RUNUP:
                if (follower.atParametricEnd()) {
                    Lift.setPosition(LIFT_LOWERED_POS);
                    setSpeed(SPEED_P1_ROTATE);
                    follower.followPath(P1_ROTATE_CHAIN, true);
                    state = State.P1_ROTATE;
                }
                break;

            case P1_ROTATE:
                if (follower.atParametricEnd()) {
                    setSpeed(SPEED_P1_APPROACH);
                    follower.followPath(P1_APPROACH_CHAIN, true);
                    state = State.P1_APPROACH;
                    // Intake starts right BEFORE approach (per your move)
                    Intake.setPower(INTAKE_POWER);
                }
                break;

            case P1_APPROACH:
                if (follower.atParametricEnd()) {
                    setSpeed(SPEED_P1_SECURE);
                    follower.followPath(P1_SECURE_CHAIN, true);
                    state = State.P1_SECURE;
                }
                break;

            case P1_SECURE:
                if (follower.atParametricEnd()) {
                    setSpeed(SPEED_P2_ALL);
                    follower.followPath(P2_REVERSE_AND_BOX, true);
                    state = State.P2_ALL;
                    Intake.setPower(0);
                }
                break;

            case P2_ALL:
                if (follower.atParametricEnd()) {
                    Intake.setPower(0.0);                  // wait until wheel is truly at speed
                    Wheel.setPower(WHEEL_POWER_SHOOT);
                    // reset delay flags
                    intakeDelayedStarted = false;
                    liftRaisedAtMs = -1L;
                    state = State.SPINUP_AND_LIFT;
                }
                break;

            case SPINUP_AND_LIFT: {
                double rpm = toRPM(safeVel(Wheel), WHEEL_TPR);
                long now = System.currentTimeMillis();

                if (!liftRaised && rpm >= WHEEL_READY_ON_RPM) {
                    Lift.setPosition(LIFT_RAISED_POS);
                    liftRaised = true;
                    liftRaisedAtMs = now; // start delay timer
                }

                // ADD DELAY HERE — wait 3s after lift raised before starting intake
                if (liftRaised && !intakeDelayedStarted && liftRaisedAtMs > 0 && (now - liftRaisedAtMs) >= INTAKE_AFTER_LIFT_DELAY_MS) {
                    Intake.setPower(INTAKE_POWER);
                    intakeDelayedStarted = true; // one-shot
                }

                // If speed sags below OFF threshold before we even started feeding, drop the lift and re-arm
                if (!intakeDelayedStarted && liftRaised && rpm <= WHEEL_READY_OFF_RPM) {
                    Lift.setPosition(LIFT_LOWERED_POS);
                    liftRaised = false;
                    liftRaisedAtMs = -1L;
                }
                break;
            }

            case DONE:
                Intake.setPower(0.0);
                Wheel.setPower(0.0);
                break;
        }
    }

    // Panels
    private void recordAndDrawPanels() {
        Pose p = follower.getPose();
        if (poseHistory.size() >= HISTORY_LIMIT) poseHistory.pollFirst();
        poseHistory.addLast(new Pose(p.getX(), p.getY(), p.getHeading()));

        double wheelRpm = toRPM(safeVel(Wheel), WHEEL_TPR);
        panels.addData("State", state);
        panels.addData("Pose", fmtPose(p));
        panels.addData("Wheel_RPM", wheelRpm);
        panels.addData("Ready_ON≥RPM", WHEEL_READY_ON_RPM);
        panels.addData("Ready_OFF≤RPM", WHEEL_READY_OFF_RPM);
        long delayLeft = (liftRaised && !intakeDelayedStarted && liftRaisedAtMs > 0) ? Math.max(0, INTAKE_AFTER_LIFT_DELAY_MS - (System.currentTimeMillis() - liftRaisedAtMs)) : 0;
        panels.addData("IntakeDelay_ms", delayLeft);
        panels.addData("Lift", liftRaised ? "RAISED" : "LOWERED");

        StringBuilder sb = new StringBuilder();
        int c = 0;
        for (Pose h : poseHistory) {
            if (c++ % 6 == 0) sb.append("\n");
            sb.append(String.format("[%.0f,%.0f]", h.getX(), h.getY()));
        }
        panels.addData("PathHistory", sb.toString());
        panels.update();
    }

    // Helpers
    private void setSpeed(double pwr) {
        try { follower.setMaxPower(pwr); } catch (Throwable ignored) {}
    }

    private PathChain buildLine(Pose a, Pose b, double hA, double hB) {
        return follower.pathBuilder()
                .addPath(new BezierLine(a, b))
                .setLinearHeadingInterpolation(hA, hB)
                .build();
    }

    private PathChain buildReverseAndBox(Pose lineSecure, Pose reversePose, Pose boxPose) {
        return follower.pathBuilder()
                .addPath(new BezierLine(lineSecure, reversePose))
                .setLinearHeadingInterpolation(lineSecure.getHeading(), reversePose.getHeading())
                .setReversed()
                .addPath(new BezierLine(reversePose, boxPose))
                .setLinearHeadingInterpolation(reversePose.getHeading(), boxPose.getHeading())
                .build();
    }

    private static String fmtPose(Pose p) {
        return String.format("(%.1f, %.1f)  θ=%.0f°", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
    }

    private static double safeVel(DcMotorEx m) {
        try { return m.getVelocity(); } catch (Exception e) { return 0.0; }
    }

    private static double toRPM(double ticksPerSec, double ticksPerRev) {
        return (ticksPerRev <= 0) ? 0.0 : (ticksPerSec / ticksPerRev) * 60.0;
    }
}

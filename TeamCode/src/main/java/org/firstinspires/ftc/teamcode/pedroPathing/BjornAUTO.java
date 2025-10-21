package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;      // Pedro V2 PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayDeque;
import java.util.Deque;

/**
 * BjornAUTO — Pedro V2 style, Panels-enabled, aligned to TeleOp device names.
 *
 * Uses your repo's unified follower factory (Constants.createFollower) and
 * Pedro V2 geometry/paths namespaces to avoid the V1/V2 mixup that caused all the errors.
 */
@Autonomous(name = "BjornAUTO (V2 fixed)", group = "Pedro Pathing")
public class BjornAUTO extends OpMode {

    // ───────── Speeds (replace with your Tuning.* if you expose them there) ─────────
    private static final double SPEED_P1_RUNUP    = 1.0;
    private static final double SPEED_P1_ROTATE   = 1.0; // slower for clean heading
    private static final double SPEED_P1_APPROACH = 0.55;
    private static final double SPEED_P1_SECURE   = 0.50;
    private static final double SPEED_P2_ALL      = 1.00;

    // ───────── Shooter/Intake tuning ─────────
    private static final double INTAKE_POWER      = 0.75; // during Phase 1
    private static final double WHEEL_POWER_SHOOT = 1.00; // after paths complete
    private static final double WHEEL_READY_TPS   = 1800; // tune on-bot (ticks/sec)

    // Lift servo positions
    private static final double LIFT_LOWERED_POS = 0.10;
    private static final double LIFT_RAISED_POS  = 0.85;

    // ───────── Poses (Pedro V2 Pose) ─────────
    private static final Pose START = new Pose(0, 0, Math.toRadians(90));

    // Phase 1 — white line approach
    private static final Pose WHITE_LINE_APPROACH = new Pose(0, 27, Math.toRadians(180.0));
    private static final Pose WHITE_LINE_SECURE   = new Pose(-19, 27, Math.toRadians(180.0));

    // Phase 2 — reverse, then blue box
    private static final Pose REVERSE_POSE   = new Pose(23, 28, Math.toRadians(133.0));
    private static final Pose BLUE_BOX_PRIME = new Pose(48, 38, Math.toRadians(133));

    // Run‑up & rotation stages
    private static final double PRE_ROTATE_ADVANCE_IN = 8.0;  // straight distance before rotating
    private static final double ROTATE_STAGE_DELTA_IN = 2.0;  // small move during rotation

    private static final Pose PRE_ROTATE_STAGE = new Pose(
            START.getX(), START.getY() + PRE_ROTATE_ADVANCE_IN, START.getHeading());

    private static final Pose ROTATE_STAGE = new Pose(
            PRE_ROTATE_STAGE.getX(), PRE_ROTATE_STAGE.getY() + ROTATE_STAGE_DELTA_IN, Math.toRadians(180.0));

    // Pedro objects
    private Follower follower;

    // Phase 1 split chains
    private PathChain P1_RUNUP_CHAIN, P1_ROTATE_CHAIN, P1_APPROACH_CHAIN, P1_SECURE_CHAIN;
    // Phase 2 combined
    private PathChain P2_REVERSE_AND_BOX;

    // Hardware (TeleOp names)
    private DcMotorEx Intake; // motor
    private DcMotorEx Wheel;  // flywheel
    private Servo     Lift;   // gate/lift

    // Panels (Triangle-like history)
    private final Telemetry panels = PanelsTelemetry.INSTANCE.getFtcTelemetry();
    private final Deque<Pose> poseHistory = new ArrayDeque<>(100);
    private static final int HISTORY_LIMIT = 60;

    // State
    private enum State { P1_RUNUP, P1_ROTATE, P1_APPROACH, P1_SECURE, P2_ALL, SPINUP_AND_LIFT, DONE }
    private State state = State.P1_RUNUP;
    private boolean liftRaised = false;

    @Override
    public void init() {
        // ✅ Use your repo's unified factory so Follower has the right constants/localizer
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(START.getX(), START.getY(), START.getHeading()));

        // Map hardware using TeleOp names
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Wheel  = hardwareMap.get(DcMotorEx.class, "Wheel");
        Lift   = hardwareMap.get(Servo.class,     "Lift");

        // Brake where useful
        DcMotor.ZeroPowerBehavior brake = DcMotor.ZeroPowerBehavior.BRAKE;
        Intake.setZeroPowerBehavior(brake);
        Wheel.setZeroPowerBehavior(brake);

        // Servo default
        Lift.setPosition(LIFT_LOWERED_POS);
        liftRaised = false;

        // Build paths — Pedro V2 builder (BezierLine accepts Pose directly)
        P1_RUNUP_CHAIN     = buildLine(START,            PRE_ROTATE_STAGE,    START.getHeading(),            PRE_ROTATE_STAGE.getHeading());
        P1_ROTATE_CHAIN    = buildLine(PRE_ROTATE_STAGE, ROTATE_STAGE,        PRE_ROTATE_STAGE.getHeading(), ROTATE_STAGE.getHeading());
        P1_APPROACH_CHAIN  = buildLine(ROTATE_STAGE,     WHITE_LINE_APPROACH, ROTATE_STAGE.getHeading(),     WHITE_LINE_APPROACH.getHeading());
        P1_SECURE_CHAIN    = buildLine(WHITE_LINE_APPROACH, WHITE_LINE_SECURE, WHITE_LINE_APPROACH.getHeading(), WHITE_LINE_SECURE.getHeading());
        P2_REVERSE_AND_BOX = buildReverseAndBox(WHITE_LINE_SECURE, REVERSE_POSE, BLUE_BOX_PRIME);

        // Start Phase 1 and turn intake on
        setSpeed(SPEED_P1_RUNUP);
        follower.followPath(P1_RUNUP_CHAIN);
        Intake.setPower(INTAKE_POWER);

        // Panels seed/history
        panels.addData("Hint", "Panels v2 output (pose/state/wheel)");
        panels.update();
        poseHistory.clear();
        poseHistory.add(new Pose(START.getX(), START.getY(), START.getHeading()));
    }

    @Override
    public void loop() {
        follower.update();
        recordAndDrawPanels();

        switch (state) {
            case P1_RUNUP:
                if (follower.atParametricEnd()) {
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
                }
                break;

            case P2_ALL:
                if (follower.atParametricEnd()) {
                    Intake.setPower(0.0); // stop feeding; change to INTAKE_POWER to keep feeding
                    Wheel.setPower(WHEEL_POWER_SHOOT);
                    state = State.SPINUP_AND_LIFT;
                }
                break;

            case SPINUP_AND_LIFT:
                double tps = safeVel(Wheel);
                boolean ready = tps >= WHEEL_READY_TPS;
                if (ready && !liftRaised) { Lift.setPosition(LIFT_RAISED_POS);  liftRaised = true; }
                if (!ready && liftRaised) { Lift.setPosition(LIFT_LOWERED_POS); liftRaised = false; }
                break;

            case DONE:
                break;
        }
    }

    // ───────────────── Panels helpers ─────────────────
    private void recordAndDrawPanels() {
        Pose p = follower.getPose();
        if (poseHistory.size() >= HISTORY_LIMIT) poseHistory.pollFirst();
        poseHistory.addLast(new Pose(p.getX(), p.getY(), p.getHeading()));

        panels.addData("State", state);
        panels.addData("Pose", fmtPose(p));
        panels.addData("Wheel_TPS", safeVel(Wheel));
        panels.addData("Wheel_ready≥", WHEEL_READY_TPS);
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

    // ───────────────── core helpers ─────────────────
    private void setSpeed(double pwr) {
        try { follower.setMaxPower(pwr); } catch (Throwable ignored) {}
    }

    private PathChain buildLine(Pose a, Pose b, double hA, double hB) {
        return follower.pathBuilder()
                .addPath(new BezierLine(a, b))
                .setLinearHeadingInterpolation(hA, hB)
                .build();
    }

    /** Phase 2 — reverse to REVERSE_POSE, then forward to BLUE_BOX_PRIME. */
    private PathChain buildReverseAndBox(Pose lineSecure, Pose reversePose, Pose boxPose) {
        return follower.pathBuilder()
                // Reverse run back to REVERSE_POSE
                .addPath(new BezierLine(lineSecure, reversePose))
                .setLinearHeadingInterpolation(lineSecure.getHeading(), reversePose.getHeading())
                .setReversed() // ← no argument in v2

                // Then forward to the blue box prime
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
}
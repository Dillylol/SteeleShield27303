package org.firstinspires.ftc.teamcode.dataCollectors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * EncBot: mecanum drive + 3-wheel odometry using names & geometry from
 * Constants.
 *
 * Pose:
 * x (in) = +left on robot, projected to field
 * y (in) = +forward on robot, projected to field
 * h (rad) = CCW+, normalized (-pi, pi]
 *
 * Encoders:
 * Left = "lf" (FORWARD)
 * Right = "lr" (FORWARD)
 * X = "rf" (REVERSE)
 *
 * Motors (all REVERSE per Constants):
 * lf, lr, rf, rr
 */
public class EncBot {

    // ---- Pull from your Constants (copied here to keep EncBot standalone) ----
    // From localizerConstants:
    private static final double FORWARD_TICKS_TO_IN = 0.001989436789;
    private static final double STRAFE_TICKS_TO_IN = 0.001989436789;
    @SuppressWarnings("unused")
    private static final double TURN_TICKS_TO_IN = 0.001989436789; // not used directly; turn from track width

    private static final double LEFT_POD_Y_IN = -2.5;
    private static final double RIGHT_POD_Y_IN = 2.5;
    private static final double STRAFE_POD_X_IN = -2.5; // + forward, - aft

    private static final String MOTOR_LF = "lf";
    private static final String MOTOR_LR = "lr";
    private static final String MOTOR_RF = "rf";
    private static final String MOTOR_RR = "rr";

    // Directions from your driveConstants (all REVERSE)
    private static final DcMotorSimple.Direction DIR_LF = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction DIR_LR = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction DIR_RF = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction DIR_RR = DcMotorSimple.Direction.FORWARD;

    // Odometry encoders mapping & directions from localizerConstants
    private static final String ODO_LEFT_NAME = "lf";
    private static final String ODO_RIGHT_NAME = "lr";
    private static final String ODO_X_NAME = "rf";
    private static final int ODO_LEFT_SIGN = -1; // Encoder.FORWARD
    private static final int ODO_RIGHT_SIGN = -1; // Encoder.FORWARD
    private static final int ODO_X_SIGN = +1; // Encoder.REVERSE

    // Derived
    private static final double TRACK_WIDTH_IN = (RIGHT_POD_Y_IN - LEFT_POD_Y_IN); // 2.5 - (-2.5) = 5.0 in

    // ---- Hardware ----
    // Drive order: 0=lf, 1=lr, 2=rf, 3=rr (matches names)
    public final DcMotorEx[] motors = new DcMotorEx[4];
    // Odo order: 0=Right, 1=Left, 2=X (matches many downstream expectations; be
    // careful with indexes)
    public final DcMotorEx[] encoders = new DcMotorEx[3];

    // ---- State ----
    private int lastLeftTicks, lastRightTicks, lastXTicks;
    private boolean firstUpdate = true;

    // pose[0]=x(in), pose[1]=y(in), pose[2]=heading(rad)
    private final double[] pose = new double[3];

    // -------------------- Init --------------------
    public void init(HardwareMap hw) {
        // Motors
        motors[0] = hw.get(DcMotorEx.class, MOTOR_LF);
        motors[1] = hw.get(DcMotorEx.class, MOTOR_LR);
        motors[2] = hw.get(DcMotorEx.class, MOTOR_RF);
        motors[3] = hw.get(DcMotorEx.class, MOTOR_RR);

        motors[0].setDirection(DIR_LF);
        motors[1].setDirection(DIR_LR);
        motors[2].setDirection(DIR_RF);
        motors[3].setDirection(DIR_RR);

        for (DcMotorEx m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Encoders (wired to motor ports)
        DcMotorEx encLeft = hw.get(DcMotorEx.class, ODO_LEFT_NAME);
        DcMotorEx encRight = hw.get(DcMotorEx.class, ODO_RIGHT_NAME);
        DcMotorEx encX = hw.get(DcMotorEx.class, ODO_X_NAME);

        encLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        encLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Store in expected order: [Right, Left, X]
        encoders[0] = encRight;
        encoders[1] = encLeft;
        encoders[2] = encX;
    }

    // -------------------- Drive --------------------
    /** Robot-centric mecanum: px=+right, py=+forward, pa=+CCW */
    public void setDrivePower(double px, double py, double pa) {
        double lf = px + py - pa;
        double lr = -px + py - pa;
        double rf = -px + py + pa;
        double rr = px + py + pa;

        double max = Math.max(1.0, Math.max(Math.abs(lf),
                Math.max(Math.abs(lr), Math.max(Math.abs(rf), Math.abs(rr)))));
        lf /= max;
        lr /= max;
        rf /= max;
        rr /= max;

        motors[0].setPower(lf);
        motors[1].setPower(lr);
        motors[2].setPower(rf);
        motors[3].setPower(rr);
    }

    // -------------------- Odometry --------------------
    public void resetOdometry(double x, double y, double headingRad) {
        pose[0] = x;
        pose[1] = y;
        pose[2] = normalizeRadians(headingRad);

        lastRightTicks = encoders[0].getCurrentPosition();
        lastLeftTicks = encoders[1].getCurrentPosition();
        lastXTicks = encoders[2].getCurrentPosition();
        firstUpdate = false;
    }

    /** Update and return {x(in), y(in), h(rad)} */
    public double[] updateOdometry() {
        int nowRight = encoders[0].getCurrentPosition();
        int nowLeft = encoders[1].getCurrentPosition();
        int nowX = encoders[2].getCurrentPosition();

        if (firstUpdate) {
            lastRightTicks = nowRight;
            lastLeftTicks = nowLeft;
            lastXTicks = nowX;
            firstUpdate = false;
            return getPose();
        }

        int dRightTicks = nowRight - lastRightTicks;
        int dLeftTicks = nowLeft - lastLeftTicks;
        int dXTicks = nowX - lastXTicks;

        lastRightTicks = nowRight;
        lastLeftTicks = nowLeft;
        lastXTicks = nowX;

        // Apply encoder direction signs and convert to inches
        double rightDist = (dRightTicks * ODO_RIGHT_SIGN) * FORWARD_TICKS_TO_IN;
        double leftDist = (dLeftTicks * ODO_LEFT_SIGN) * FORWARD_TICKS_TO_IN;
        double xRaw = (dXTicks * ODO_X_SIGN) * STRAFE_TICKS_TO_IN;

        // Forward delta (robot frame)
        double dyR = 0.5 * (rightDist + leftDist);

        // Heading change from L/R separation
        double dHeading = (rightDist - leftDist) / TRACK_WIDTH_IN;

        // Correct X for rotation if X wheel offset from rotation center
        double dxR = xRaw - dHeading * STRAFE_POD_X_IN;

        // Field transform using average heading
        double avgH = pose[2] + 0.5 * dHeading;
        double cos = Math.cos(avgH);
        double sin = Math.sin(avgH);

        double dXfield = dxR * sin + dyR * cos; // +x = left on robot
        double dYfield = -dxR * cos + dyR * sin; // +y = forward on robot

        pose[0] += dXfield;
        pose[1] += dYfield;
        pose[2] = normalizeRadians(pose[2] + dHeading);

        return getPose();
    }

    public double[] getPose() {
        return new double[] { pose[0], pose[1], pose[2] };
    }

    // -------------------- Utils --------------------
    private static double normalizeRadians(double a) {
        while (a <= -Math.PI)
            a += 2.0 * Math.PI;
        while (a > Math.PI)
            a -= 2.0 * Math.PI;
        return a;
    }
}

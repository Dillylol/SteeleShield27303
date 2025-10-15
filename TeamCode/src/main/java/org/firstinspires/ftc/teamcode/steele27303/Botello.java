package org.firstinspires.ftc.teamcode.steele27303;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Botello")
public class Botello extends OpMode {

    private DcMotor BackL, BackR, FrontL, FrontR, Intake, Wheel;

    // --- Boolean Toggle Variables ---
    private boolean isIntakeOn = false;
    private boolean isWheelOn = false;
    private boolean aWasPressed = false;
    private boolean bWasPressed = false;


    @Override
    public void init() {
        BackL = hardwareMap.get(DcMotor.class, "BackL");
        BackR = hardwareMap.get(DcMotor.class, "BackR");
        FrontL = hardwareMap.get(DcMotor.class, "FrontL");
        FrontR = hardwareMap.get(DcMotor.class, "FrontR");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Wheel = hardwareMap.get(DcMotor.class, "Wheel");

        // --- Corrected Motor Directions ---
        // For a typical mecanum drive, you reverse the motors on one side.
        // Let's reverse the right side motors.
        FrontL.setDirection(DcMotor.Direction.REVERSE);
        BackL.setDirection(DcMotor.Direction.REVERSE);
        FrontR.setDirection(DcMotor.Direction.REVERSE);
        BackR.setDirection(DcMotor.Direction.REVERSE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        Wheel.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor.ZeroPowerBehavior brake = DcMotor.ZeroPowerBehavior.BRAKE;
        BackL.setZeroPowerBehavior(brake);
        BackR.setZeroPowerBehavior(brake);
        FrontL.setZeroPowerBehavior(brake);
        FrontR.setZeroPowerBehavior(brake);
        Intake.setZeroPowerBehavior(brake);
        Wheel.setZeroPowerBehavior(brake);
    }

    @Override
    public void loop() {
        // --- Drivetrain Logic ---
        double drive =  gamepad1.left_stick_y;  // Forward/Backward
        double strafe = gamepad1.left_stick_x;   // Strafe Left/Right
        double rotate = gamepad1.right_stick_x;  // Rotate Left/Right

        // --- Corrected Mecanum Drive Formulas ---
        double FrontLPower = drive - strafe + rotate;
        double FrontRPower = drive + strafe - rotate;
        double BackLPower = drive + strafe + rotate;
        double BackRPower = drive - strafe - rotate;

        // Normalize motor powers to ensure they are within the -1.0 to 1.0 range
        double maxPower = Math.max(Math.max(Math.abs(FrontLPower), Math.abs(FrontRPower)),
                Math.max(Math.abs(BackLPower), Math.abs(BackRPower)));

        if (maxPower > 1.0) {
            FrontLPower /= maxPower;
            FrontRPower /= maxPower;
            BackLPower /= maxPower;
            BackRPower /= maxPower;
        }

        FrontL.setPower(FrontLPower);
        FrontR.setPower(FrontRPower);
        BackL.setPower(BackLPower);
        BackR.setPower(BackRPower);


        // --- Intake Motor Toggle Logic ---
        if (gamepad1.a && !aWasPressed) {
            isIntakeOn = !isIntakeOn;
        }
        aWasPressed = gamepad1.a;

        if (isIntakeOn) {
            Intake.setPower(1);
        } else {
            Intake.setPower(0);
        }


        // --- Wheel Motor Toggle Logic ---
        if (gamepad1.b && !bWasPressed) {
            isWheelOn = !isWheelOn;
        }
        bWasPressed = gamepad1.b;

        if (isWheelOn) {
            Wheel.setPower(1);
        } else {
            Wheel.setPower(0);
        }

        // --- Telemetry for Debugging ---
        telemetry.addData("Intake Status", isIntakeOn ? "ON" : "OFF");
        telemetry.addData("Wheel Status", isWheelOn ? "ON" : "OFF");
        telemetry.addData("---", "---");
        telemetry.addData("Gamepad 'a' button pressed", gamepad1.a);
        telemetry.addData("Gamepad 'b' button pressed", gamepad1.b);
        telemetry.update();
    }
}
//Certified Dylen Vasquez Design


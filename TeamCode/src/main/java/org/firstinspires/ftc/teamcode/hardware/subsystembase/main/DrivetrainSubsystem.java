package org.firstinspires.ftc.teamcode.hardware.subsystembase.main;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.base.BaseSubsystem;

@Config
public class DrivetrainSubsystem extends BaseSubsystem {
    // Values
    public static double NORMAL_SPEED = 0.6;
    public static double FULL_SPEED = 1;
    public static double SLOW_SPEED = 0.3;

    public double leftPower;
    public double rightPower;

    public double speedMultiplier;

    // Create hardware variables
    public DcMotorEx leftFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightRear = null;

    // Constructor
    public DrivetrainSubsystem() {
    }

    // Initialize hardware variables
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        super.init(hardwareMap, telemetry);
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront  = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotorEx.class, "rightRear");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        // Drivetrain Motor Directions
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftRear.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightRear.setDirection(DcMotorEx.Direction.REVERSE);

        // Adjusting the Zero Power Behavior changes how the motors behaved when a
        // Power of 0 is applied.

        // Drivetrain Zero Power Behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Default command
    public void defaultCommand(Gamepad gamepad1, Gamepad gamepad2) {
        super.gamepadInit(gamepad1, gamepad2);
        // Control Drivetrain speed with bumpers
        if (gamepad1.left_bumper) {
            speedMultiplier = Drivetrain.FULL_SPEED;
        } else if (gamepad1.right_bumper) {
            speedMultiplier = Drivetrain.SLOW_SPEED;
        } else {
            speedMultiplier = Drivetrain.NORMAL_SPEED;
        }

        // POV Mode uses left stick to go forward, and right stick to turn.
        leftPower    = Range.clip(-gamepad1.left_stick_y + gamepad1.right_stick_x, -1.0, 1.0);
        rightPower   = Range.clip(-gamepad1.left_stick_y - gamepad1.right_stick_x, -1.0, 1.0);

        // Send calculated power to wheels
        leftFront.setPower(leftPower * speedMultiplier);
        leftRear.setPower(leftPower * speedMultiplier);
        rightFront.setPower(rightPower * speedMultiplier);
        rightRear.setPower(rightPower * speedMultiplier);
    }
    public void breakMode() {
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    public void floatMode() {
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        leftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }
}
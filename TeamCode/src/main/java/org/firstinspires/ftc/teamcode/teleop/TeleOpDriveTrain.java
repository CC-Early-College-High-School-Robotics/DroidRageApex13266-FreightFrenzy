package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOP drive train only", group="Beta")
public class TeleOpDriveTrain extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Drivetrain
    private DcMotorEx leftFront = null;
    private DcMotorEx leftRear = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightRear = null;

    double slowMode = 1;

    @Override
    public void init() {
        // Drivetrain Initialization
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront  = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightRear.setDirection(DcMotorEx.Direction.FORWARD);
    }

    @Override
    public void loop() {
        // Drive Train Code

        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        if (gamepad1.left_bumper) {
            slowMode = 0.6;
        } else {
            slowMode = 1;
        }

        leftPower    = drive + turn;
        rightPower   = drive - turn;

        // Send calculated power to wheels
        leftFront.setPower(leftPower * slowMode);
        leftRear.setPower(leftPower * slowMode);
        rightFront.setPower(rightPower * slowMode);
        rightRear.setPower(rightPower * slowMode);
    }
}

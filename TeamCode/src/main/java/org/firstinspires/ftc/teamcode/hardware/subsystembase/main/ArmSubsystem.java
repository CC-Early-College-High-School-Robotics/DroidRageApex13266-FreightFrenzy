package org.firstinspires.ftc.teamcode.hardware.subsystembase.main;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.base.BaseSubsystem;

@Config
public class ArmSubsystem extends BaseSubsystem {
    // Values
    public static double ARM_INTAKE_POS = 0; // 0.04 //-0.09
    public static double ARM_NEUTRAL_POS = -0.003;
    public static double ARM_LOW_POS = -.4884; //0.132 //0.035
    public static double ARM_MID_POS = -.41; //0.25 //0.17
    public static double ARM_HIGH_POS = -.338; //0.36 //0.27

    public static double ARM_POWER = 0.55;
    public static double ARM_SLOW_POWER = 0.3;
    public static double ARM_TICKS_PER_REV = 1425.06;
    public static double ARM_POS_CHANGE_SPEED = 0.0005;
    public static double ARM_INTAKE_WAIT = 0.25;
    boolean setIntakePos = false;
    double targetTime = 0;

    public static double armCurrentPos = 0;

    // Create hardware variables
    public DcMotorEx armMotor = null;

    // Constructor
    public ArmSubsystem() {

    }

    // Initialize hardware variables
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        super.init(hardwareMap, telemetry);
        armMotor = hardwareMap.get(DcMotorEx.class,"armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Arm Encoders
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

    }

    // Arm Commands
    public void setArmPosition(Gamepad gamepad1, Gamepad gamepad2) {
        super.gamepadInit(gamepad1, gamepad2);
        armMotor.setTargetPosition((int) (ARM_TICKS_PER_REV * armCurrentPos));
    }

    public void findArmPosition(Gamepad gamepad1, Gamepad gamepad2) {
        super.gamepadInit(gamepad1, gamepad2);
        // High
        if (gamepad2.dpad_up) {
            armMotor.setPower(ARM_POWER);
            armCurrentPos = ARM_HIGH_POS;
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        // Mid
        if (gamepad2.dpad_right) {
            armMotor.setPower(ARM_POWER);
            armCurrentPos = ARM_MID_POS;
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        // Shared hub
        if (gamepad2.dpad_down) {
            armMotor.setPower(ARM_POWER);
            armCurrentPos = ARM_LOW_POS;
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        // intake/reset position
        if (gamepad2.dpad_left) {

            targetTime = runtime.seconds() + ARM_INTAKE_WAIT;
            setIntakePos = true;
        }
        if (setIntakePos && runtime.seconds() >= targetTime) {
            armCurrentPos = ARM_INTAKE_POS;
        }

        if (-gamepad2.right_stick_y > ControllerSubsystem.TRIGGER_THRESHOLD) {
            armCurrentPos += ARM_POS_CHANGE_SPEED;
        }
        if (-gamepad2.right_stick_y < -ControllerSubsystem.TRIGGER_THRESHOLD) {
            armCurrentPos -= ARM_POS_CHANGE_SPEED;
        }



        if (-gamepad2.right_stick_y > ControllerSubsystem.TRIGGER_THRESHOLD && gamepad2.dpad_left) {
            ARM_INTAKE_POS+= ARM_POS_CHANGE_SPEED;
        }
        if (-gamepad2.right_stick_y < -ControllerSubsystem.TRIGGER_THRESHOLD && gamepad2.dpad_left) {
            ARM_INTAKE_POS -= ARM_POS_CHANGE_SPEED;
        }



        if (-gamepad2.right_stick_y > ControllerSubsystem.TRIGGER_THRESHOLD && gamepad2.dpad_up) {
            ARM_HIGH_POS+= ARM_POS_CHANGE_SPEED;
        }
        if (-gamepad2.right_stick_y < -ControllerSubsystem.TRIGGER_THRESHOLD && gamepad2.dpad_up) {
            ARM_HIGH_POS -= ARM_POS_CHANGE_SPEED;
        }



        if (-gamepad2.right_stick_y > ControllerSubsystem.TRIGGER_THRESHOLD && gamepad2.dpad_right) {
            ARM_MID_POS+= ARM_POS_CHANGE_SPEED;
        }
        if (-gamepad2.right_stick_y < -ControllerSubsystem.TRIGGER_THRESHOLD && gamepad2.dpad_right) {
            ARM_MID_POS -= ARM_POS_CHANGE_SPEED;
        }

        if (-gamepad2.right_stick_y > ControllerSubsystem.TRIGGER_THRESHOLD && gamepad2.dpad_down) {
            ARM_LOW_POS+= ARM_POS_CHANGE_SPEED;
        }
        if (-gamepad2.right_stick_y < -ControllerSubsystem.TRIGGER_THRESHOLD && gamepad2.dpad_down) {
            ARM_LOW_POS -= ARM_POS_CHANGE_SPEED;
        }
    }


    public double getArmPosition() {
        return armMotor.getCurrentPosition()/ ARM_TICKS_PER_REV;

    }

    public void breakMode() {
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    public void floatMode() {
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }
}
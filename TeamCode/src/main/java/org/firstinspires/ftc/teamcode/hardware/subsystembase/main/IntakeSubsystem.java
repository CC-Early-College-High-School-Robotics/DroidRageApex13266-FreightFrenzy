package org.firstinspires.ftc.teamcode.hardware.subsystembase.main;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.base.BaseSubsystem;

@Config
public class IntakeSubsystem extends BaseSubsystem {
    // Values
//    public static double INTAKE_VELOCITY = 1000;
    public static double INTAKE_POWER = 0.8;
    boolean disableIntake = false;

    // Create hardware variables
    public DcMotorEx intakeMotor = null;

    // Constructor
    public IntakeSubsystem() {
    }

    // Initialize hardware variables
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        super.init(hardwareMap, telemetry);

        // Initialize hardware variables
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intakeMotor");

        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    // Default command
    public void defaultCommand(Gamepad gamepad1, Gamepad gamepad2) {
        super.gamepadInit(gamepad1, gamepad2);
        if ((gamepad1.right_trigger >= ControllerSubsystem.TRIGGER_THRESHOLD )&& !disableIntake) {
            intakeMotor.setPower(-INTAKE_POWER);
        }
        else if ((gamepad1.left_trigger >= ControllerSubsystem.TRIGGER_THRESHOLD )&& !disableIntake) {
            intakeMotor.setPower(INTAKE_POWER);
        }
        else if ((gamepad1.right_trigger < ControllerSubsystem.TRIGGER_THRESHOLD && gamepad1.left_trigger < ControllerSubsystem.TRIGGER_THRESHOLD) && !disableIntake) {
            intakeMotor.setVelocity(0);
        }
    }
}
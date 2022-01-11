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
public class CarouselSubsystem extends BaseSubsystem {
    // Values
    public static double CAROUSEL_POWER = 0.4;
    public static double CAROUSEL_SLOW_POWER = 0.4;

    // Create hardware variables
    public DcMotorEx carouselMotor = null;

    // Constructor
    public CarouselSubsystem() {
    }

    // Initialize hardware variables
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        super.init(hardwareMap, telemetry);
        carouselMotor = hardwareMap.get(DcMotorEx.class,"carouselMotor");
        carouselMotor.setDirection(DcMotorEx.Direction.FORWARD);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    // Default command
    public void defaultCommand(Gamepad gamepad1, Gamepad gamepad2) {
        super.gamepadInit(gamepad1, gamepad2);
        if (gamepad1.x) {
            carouselMotor.setPower(CAROUSEL_POWER);
        }
        if (gamepad1.b) {
            carouselMotor.setPower(-CAROUSEL_POWER);
        }
        if (!gamepad1.b && !gamepad1.x) {
            carouselMotor.setPower(0);
        }
    }

    public void breakMode() {
        carouselMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    public void floatMode() {
        carouselMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }
}
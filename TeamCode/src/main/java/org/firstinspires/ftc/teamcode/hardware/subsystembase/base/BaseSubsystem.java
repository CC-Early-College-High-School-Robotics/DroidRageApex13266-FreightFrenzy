package org.firstinspires.ftc.teamcode.hardware.subsystembase.base;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BaseSubsystem {
    // Values

    // Create local OpMode members
    public Gamepad gamepad1, gamepad2;
    public HardwareMap hardwareMap = null;
    public Telemetry telemetry;
    public ElapsedTime runtime  = new ElapsedTime();

    // Constructors
    public BaseSubsystem(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public BaseSubsystem(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    public BaseSubsystem() {

    }

    // Initialize Hardware
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public void gamepadInit(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }
}

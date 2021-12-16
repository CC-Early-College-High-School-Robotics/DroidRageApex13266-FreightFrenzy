package org.firstinspires.ftc.teamcode.hardware.subsystembase.sample;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.base.BaseSubsystem;

@Config
@Disabled
public class SampleSubsystem extends BaseSubsystem {
    // Values

    // Create hardware variables

    // Constructor
    public SampleSubsystem(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry) {
        super(gamepad1, gamepad2);
    }

    // Initialize hardware variables
    public void init() {
        super.init(hardwareMap, telemetry);

    }

    // Default command
    public void defaultCommand() {

    }
}
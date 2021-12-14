package org.firstinspires.ftc.teamcode.teleop.subsystemtest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.AllianceMarkerStickSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.ArmSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.BoxSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.BreakModeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.TelemetrySubsystem;

public class TeleOpBase extends OpMode {
    // Declare subsystems
    AllianceMarkerStickSubsystem stick          = new AllianceMarkerStickSubsystem          ();
    ArmSubsystem arm                            = new ArmSubsystem                          ();
    BoxSubsystem box                            = new BoxSubsystem                          ();
    CarouselSubsystem carousel                  = new CarouselSubsystem                     ();
    IntakeSubsystem intake                      = new IntakeSubsystem                       ();
    TelemetrySubsystem telemetrySubsystem       = new TelemetrySubsystem                    ();

    @Override
    public void init() {
        // Initialize subsystems
        stick.init(hardwareMap, telemetry);
        arm.init(hardwareMap, telemetry);
        box.init(hardwareMap, telemetry);
        carousel.init(hardwareMap, telemetry);
        intake.init(hardwareMap, telemetry);

        telemetrySubsystem.initMessage();
    }

    @Override
    public void loop() {
        stick.defaultCommand(gamepad1, gamepad2);
        arm.findArmPosition(gamepad1, gamepad2);
        arm.setArmPosition(gamepad1, gamepad2);
        box.defaultCommand(gamepad1, gamepad2);
        carousel.defaultCommand(gamepad1, gamepad2);
        intake.defaultCommand(gamepad1, gamepad2);
        telemetrySubsystem.defaultCommand();
    }
}
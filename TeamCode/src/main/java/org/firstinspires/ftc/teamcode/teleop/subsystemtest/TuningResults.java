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

@TeleOp(name="Tuning results")
public class TuningResults extends OpMode {
    // Declare subsystems
    AllianceMarkerStickSubsystem stick          = new AllianceMarkerStickSubsystem          (gamepad1, gamepad2, hardwareMap, telemetry);
    ArmSubsystem arm                            = new ArmSubsystem                          (gamepad1, gamepad2, hardwareMap, telemetry);
    BoxSubsystem box                            = new BoxSubsystem                          (gamepad1, gamepad2, hardwareMap, telemetry);
    CarouselSubsystem carousel                  = new CarouselSubsystem                     (gamepad1, gamepad2, hardwareMap, telemetry);
    DrivetrainSubsystem drivetrain              = new DrivetrainSubsystem                   (gamepad1, gamepad2, hardwareMap, telemetry);
    IntakeSubsystem intake                      = new IntakeSubsystem                       (gamepad1, gamepad2, hardwareMap, telemetry);
    BreakModeSubsystem breakMode                = new BreakModeSubsystem                    (gamepad1, gamepad2, hardwareMap, telemetry, arm, carousel, drivetrain);
    TelemetrySubsystem telemetrySubsystem       = new TelemetrySubsystem                    (gamepad1, gamepad2, hardwareMap, telemetry, stick, arm, box, carousel, drivetrain, intake, breakMode);
    @Override
    public void init() {
        // Initialize subsystems
        stick.init();
        arm.init();
        box.init();
        carousel.init();
        drivetrain.init();
        intake.init();

        telemetrySubsystem.initMessage();
    }

    @Override
    public void loop() {
        stick.defaultCommand();
        arm.findArmPosition();
        arm.setArmPosition();
        box.defaultCommand();
        carousel.defaultCommand();
        drivetrain.defaultCommand();
        intake.defaultCommand();
        breakMode.defaultCommand();
        telemetrySubsystem.verboseCommand();
    }
}
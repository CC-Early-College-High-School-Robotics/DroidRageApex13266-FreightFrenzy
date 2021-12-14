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

@TeleOp(name="TeleOP Main Base extension")
public class TeleOpBaseExtension extends TeleOpBase {
    // Declare subsystems
    DrivetrainSubsystem drivetrain              = new DrivetrainSubsystem                   ();
    BreakModeSubsystem breakMode                = new BreakModeSubsystem                    ();

    @Override
    public void init()  {
        super.init();
        drivetrain.init(hardwareMap, telemetry);
        breakMode.init(gamepad1, gamepad2, arm, carousel, drivetrain);

        telemetrySubsystem.initMessage();
    }

    @Override
    public void loop() {
        super.loop();
        drivetrain.defaultCommand(gamepad1, gamepad2);
        breakMode.defaultCommand();
    }
}
package org.firstinspires.ftc.teamcode.teleop.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Blank teleop")
@Disabled
public class BlankTeleOp extends LinearOpMode {
    int cycles = 0;
    ElapsedTime runtime;

    @Override
    public void runOpMode() {
        runtime = new ElapsedTime();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            cycles++;
            telemetry.addData("Frequency", (int) (cycles / runtime.seconds()) + "hz");
            telemetry.addData("Cycles", cycles);
            telemetry.update();
        }
    }
}
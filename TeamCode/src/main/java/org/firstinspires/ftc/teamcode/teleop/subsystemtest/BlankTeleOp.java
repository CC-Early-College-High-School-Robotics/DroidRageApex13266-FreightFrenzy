package org.firstinspires.ftc.teamcode.teleop.subsystemtest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Blank teleop")
public class BlankTeleOp extends OpMode {
    int cycles = 0;
    ElapsedTime runtime;

    @Override
    public void init()  {
        runtime = new ElapsedTime();
    }

    @Override
    public void loop() {
        cycles++;
        telemetry.addData("Frequency", (int) (cycles / runtime.seconds()) + "hz");
    }
}
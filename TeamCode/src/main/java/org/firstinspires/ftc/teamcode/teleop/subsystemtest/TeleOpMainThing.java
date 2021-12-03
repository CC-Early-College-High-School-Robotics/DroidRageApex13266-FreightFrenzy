package org.firstinspires.ftc.teamcode.teleop.subsystemtest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.teleop.main.TeleOpMain;

public class TeleOpMainThing extends OpMode {
    TeleOpMain teleOpMain = new TeleOpMain();
    @Override
    public void init () {
        teleOpMain.init();
    }
    @Override
    public void start() {
        teleOpMain.start();
    }
    @Override
    public void loop () {
        teleOpMain.loop();
        telemetry.addData("can you", "see this");
    }
}

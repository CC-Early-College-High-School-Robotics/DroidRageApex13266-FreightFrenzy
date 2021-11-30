package org.firstinspires.ftc.teamcode.teleop.test;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Position Programmer", group="test")
public class ServoPositionProgrammer extends OpMode {
    //hardware initialization
    Servo servo;
    double pos = 0.5;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "cameraServo");
    }

    @Override
    public void loop() {


        if(gamepad1.a){
            pos -= 0.001;
        }
        else if(gamepad1.b){
            pos += 0.001;
        }
        pos = Math.min(Math.max(pos, 0), 1);
        servo.setPosition(Math.min(Math.max(pos, 0), 1));
        telemetry.addData("servo pos",servo.getPosition());
        telemetry.addData("desired pos", pos);
        telemetry.update();
        sleep(10);

    }
}

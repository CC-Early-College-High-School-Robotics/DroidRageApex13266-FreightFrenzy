/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.Hardware13266;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOP 13266")
public class TeleOp13266 extends OpMode {

    Hardware13266 robot = new Hardware13266();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        robot.init(hardwareMap);

        // Tell the driver (by printing a message on the driver station) that initialization is complete.
        telemetry.addData("Sup", "Initialized");
    }



    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        robot.runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // Drive Train Code

        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        if (gamepad1.left_bumper) {
            robot.slowMode = 1;
        } else {
            robot.slowMode = 0.6;
        }

        leftPower    = Range.clip(drive + turn, -1.0, 1.0);
        rightPower   = Range.clip(drive - turn, -1.0, 1.0);

        // Send calculated power to wheels
        robot.leftFront.setPower(leftPower * robot.slowMode);
        robot.leftRear.setPower(leftPower * robot.slowMode);
        robot.rightFront.setPower(rightPower * robot.slowMode);
        robot.rightRear.setPower(rightPower * robot.slowMode);
/*
      if (gamepad1.b) {
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotor.setPower(0.5);
        }

 */
      /*
        if (gamepad1.dpad_up) {
            //robot.armMotor.setPower(0.5);
            robot.armMotor.setTargetPosition(armMotor.getCurrentPosition()+1);
        }

        if (gamepad1.dpad_down) {
            //robot.armMotor.setPower(-0.5);
            robot.armMotor.setTargetPosition(armMotor.getCurrentPosition()-1);
        }

        if (gamepad1.x) {
            //robot.armMotor.setPower(0);
            robot.armMotor.setTargetPosition(0);
        }
        */

        // Carousel Code

        if (gamepad2.right_bumper) {
            robot.carouselMotor.setPower(robot.CAROUSEL_POWER);
        }
        if (gamepad2.left_bumper) {
            robot.carouselMotor.setPower(-robot.CAROUSEL_POWER);
        }
        if (!gamepad2.right_bumper && !gamepad2.left_bumper) {
            robot.carouselMotor.setPower(0);
        }

       // box servo code
        /*
        if (gamepad2.left_trigger >= 0.1) {
            robot.boxServo.setPosition(0);
        } else {
            robot.boxServo.setPosition(0.5);
        }
*/

        // Change motors between BRAKE and FLOAT zero power modes
        if (gamepad1.a) {
            robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            robot.carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (gamepad1.b) {
            robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            robot.carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }


        // High
        if (gamepad2.dpad_up) {
            robot.armMotor.setPower(robot.ARM_POWER);
            robot.setArmPosition(robot.ARM_HIGH_POS);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.boxServo.setPosition(robot.BOX_UP);
        }
        // Mid
        if (gamepad2.dpad_right) {
            robot.armMotor.setPower(robot.ARM_POWER);
            robot.setArmPosition(robot.ARM_MID_POS);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.boxServo.setPosition(robot.BOX_UP);
        }
        // Lower
        if (gamepad2.dpad_down) {
            robot.armMotor.setPower(robot.ARM_POWER);
            robot.setArmPosition(robot.ARM_LOW_POS);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.boxServo.setPosition(robot.BOX_UP);


        }
        // intake/reset position
        if (gamepad2.dpad_left) {
            robot.setArmPosition(robot.ARM_INTAKE_POS);
            robot.boxServo.setPosition(robot.BOX_INTAKE);
        }
        if (gamepad2.a) {
            robot.boxServo.setPosition(robot.BOX_INTAKE);
        }
        if (gamepad2.x) {
            robot.boxServo.setPosition(robot.BOX_DROP);
        }
        if (gamepad2.y) {
            robot.boxServo.setPosition(robot.BOX_UP);
        }

        // Intake Motor
        if (gamepad1.right_trigger >= 0.1) {
            robot.intakeMotor.setVelocity(-robot.INTAKE_VELOCITY);
        }
        if (gamepad1.left_trigger >= 0.1) {
            robot.intakeMotor.setVelocity(robot.INTAKE_VELOCITY);
        }
        if (gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1) {
            robot.intakeMotor.setVelocity(0);
        }






        // Show the elapsed game time and wheel power.

        telemetry.addData("Status", "Run Time: " + robot.runtime.toString());
        robot.cycles++;
        telemetry.addData("Frequency", (int) (robot.cycles / robot.runtime.seconds()) + "hz");
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("duck motor position", robot.carouselMotor.getCurrentPosition());
        telemetry.addData("arm motor position", robot.armMotor.getCurrentPosition());
        telemetry.addData("box position", robot.boxServo.getPosition());
        telemetry.addData("arm motor position divided by tick per rev", robot.armMotor.getCurrentPosition()/383.6);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
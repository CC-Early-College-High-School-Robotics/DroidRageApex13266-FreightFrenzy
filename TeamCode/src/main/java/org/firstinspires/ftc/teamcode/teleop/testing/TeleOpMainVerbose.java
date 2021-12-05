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

package org.firstinspires.ftc.teamcode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;

@TeleOp(name="TeleOP Main Verbose")
public class TeleOpMainVerbose extends OpMode {

    // Create references to
    Devices robot = new Devices();
    Drivetrain drive = new Drivetrain();

    // Code to run ONCE when the driver hits INIT

    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        robot.init(hardwareMap);
        drive.init(hardwareMap);

        // Tell the driver (by printing a message on the driver station) that initialization is complete.
        telemetry.addData("GL", "You better win!");
    }

    // Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        robot.runtime.reset();
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {

        // Drive Train Code

        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double forward = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        if (gamepad1.left_bumper) {
            drive.slowMode = drive.FULL_SPEED;
        } else {
            drive.slowMode = drive.SLOW_SPEED;
        }

        leftPower    = Range.clip(forward + turn, -1.0, 1.0);
        rightPower   = Range.clip(forward - turn, -1.0, 1.0);

        // Send calculated power to wheels
        drive.leftFront.setPower(leftPower * drive.slowMode);
        drive.leftRear.setPower(leftPower * drive.slowMode);
        drive.rightFront.setPower(rightPower * drive.slowMode);
        drive.rightRear.setPower(rightPower * drive.slowMode);

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

        // Change motors between BRAKE and FLOAT zero power modes
        if (gamepad1.a) {
            drive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            robot.carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (gamepad1.b) {
            drive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            drive.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            drive.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            drive.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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
        // Shared hub
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
        telemetry.addData("arm pos: ", robot.getArmPosition());
    }
}
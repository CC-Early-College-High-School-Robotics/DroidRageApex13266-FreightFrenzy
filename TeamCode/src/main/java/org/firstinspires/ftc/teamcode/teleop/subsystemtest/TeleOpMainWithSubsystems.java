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
/*
package org.firstinspires.ftc.teamcode.teleop.subsystemtest;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.teleop.main.TeleOpMain;

@TeleOp(name="TeleOP Main (with subsystems)")
public class TeleOpMainWithSubsystems extends OpMode {

    // Declare hardware
    Devices robotHardware = new Devices();
    Drivetrain driveHardware = new Drivetrain();

    private GamepadEx driverGamepad = new GamepadEx(gamepad1);
    private GamepadEx operatorGamepad = new GamepadEx(gamepad2);

    DrivetrainSubsystem drivetrain = new DrivetrainSubsystem(driveHardware, driverGamepad);


    @Override
    public void init() {
        robotHardware.init(hardwareMap);
        driveHardware.init(hardwareMap);

        // Subsystems
        drivetrain.setDefaultCommand(drivetrain.DefaultCommand());

        // Tell the driver (by printing a message on the driver station) that initialization is complete.
        telemetry.addData("GL", "You better win!");
    }
    @Override
    public void loop() {


        // Carousel Code

        if (gamepad2.right_bumper) {
            robotHardware.carouselMotor.setPower(robotHardware.CAROUSEL_POWER);
        }
        if (gamepad2.left_bumper) {
            robotHardware.carouselMotor.setPower(-robotHardware.CAROUSEL_POWER);
        }
        if (!gamepad2.right_bumper && !gamepad2.left_bumper) {
            robotHardware.carouselMotor.setPower(0);
        }

       // box servo code

        // Change motors between BRAKE and FLOAT zero power modes
        if (gamepad1.a) {
            driveHardware.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            driveHardware.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            driveHardware.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            driveHardware.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            robotHardware.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            robotHardware.carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (gamepad1.b) {
            driveHardware.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            driveHardware.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            driveHardware.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            driveHardware.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            robotHardware.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            robotHardware.carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }


        // High
        if (gamepad2.dpad_up) {
            robotHardware.armMotor.setPower(robotHardware.ARM_POWER);
            robotHardware.setArmPosition(robotHardware.ARM_HIGH_POS);
            robotHardware.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotHardware.boxServo.setPosition(robotHardware.BOX_UP);
        }
        // Mid
        if (gamepad2.dpad_right) {
            robotHardware.armMotor.setPower(robotHardware.ARM_POWER);
            robotHardware.setArmPosition(robotHardware.ARM_MID_POS);
            robotHardware.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotHardware.boxServo.setPosition(robotHardware.BOX_UP);
        }
        // Shared hub
        if (gamepad2.dpad_down) {
            robotHardware.armMotor.setPower(robotHardware.ARM_POWER);
            robotHardware.setArmPosition(robotHardware.ARM_SHARED_HUB_POS);
            robotHardware.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotHardware.boxServo.setPosition(robotHardware.BOX_UP);

        }
        // intake/reset position
        if (gamepad2.dpad_left) {
            robotHardware.setArmPosition(robotHardware.ARM_INTAKE_POS);
            robotHardware.boxServo.setPosition(robotHardware.BOX_INTAKE);
        }

        if (gamepad2.a) {
            robotHardware.boxServo.setPosition(robotHardware.BOX_INTAKE);
        }
        if (gamepad2.x) {
            robotHardware.boxServo.setPosition(robotHardware.BOX_DROP);
        }
        if (gamepad2.y) {
            robotHardware.boxServo.setPosition(robotHardware.BOX_UP);
        }

        // Intake Motor
        if (gamepad1.right_trigger >= 0.1) {
            robotHardware.intakeMotor.setVelocity(-robotHardware.INTAKE_VELOCITY);
        }
        if (gamepad1.left_trigger >= 0.1) {
            robotHardware.intakeMotor.setVelocity(robotHardware.INTAKE_VELOCITY);
        }
        if (gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1) {
            robotHardware.intakeMotor.setVelocity(0);
        }

        // Show the elapsed game time and wheel power.

        telemetry.addData("Status", "Run Time: " + robotHardware.runtime.toString());
        robotHardware.cycles++;
        telemetry.addData("Frequency", (int) (robotHardware.cycles / robotHardware.runtime.seconds()) + "hz");

    }
    @Override
    public void stop() {
    }

}
*/
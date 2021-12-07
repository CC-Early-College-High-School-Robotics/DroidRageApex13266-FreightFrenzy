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

package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
@Config
public class Devices {
    /* Constants */

    // Lower positions
    public final double ARM_INTAKE_POS = 0; // 0.04 //-0.09
    public final double ARM_NEUTRAL_POS = 0;

    // Higher Positions
    public final double ARM_LOW_POS = 0.132; //0.132 //0.035
    public final double ARM_MID_POS = 0.25; //0.25 //0.17
    public final double ARM_HIGH_POS = 0.36; //0.36 //0.27

    // Box servo positions
    public final double BOX_UP = 0.641;
    public final double BOX_FORWARD = 0.319;
    public final double BOX_INTAKE = 0.925;
    public final double BOX_DROP = 0.180;

    // Motor constants
    public final double INTAKE_VELOCITY = 1000;
    public final double CAROUSEL_POWER = 0.5;
    public final double CAROUSEL_SLOW_POWER = 0.4;
    public final double ARM_POWER = 0.6;
    public final double ARM_SLOW_POWER = 0.3;
    public final double ARM_TICKS_PER_REV = 1425.06;

    // Cycles variable (to calculate loop time)
    public int cycles = 0;
    public int capServoPos = 0;

    /* Create hardware variables */
    // Motor variables
    public DcMotorEx armMotor = null;
    public DcMotorEx carouselMotor = null;
    public DcMotorEx intakeMotor = null;

    // Servo variables
    public Servo boxServo = null;
    public Servo cameraServo = null;
    public Servo capServo = null;

    // Arm methods to get and set the posiition
    public void setArmPosition(double pos) {
        armMotor.setTargetPosition((int) (ARM_TICKS_PER_REV * pos));
    }
    public double getArmPosition() {
        return armMotor.getCurrentPosition()/ ARM_TICKS_PER_REV;
    }

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    public ElapsedTime runtime  = new ElapsedTime();

    /* Constructor */
    public Devices(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Motor Initialization

        // Arm Motor Initialization
        armMotor = hwMap.get(DcMotorEx.class,"armMotor");
        carouselMotor = hwMap.get(DcMotorEx.class,"carouselMotor");
        intakeMotor = hwMap.get(DcMotorEx.class,"intakeMotor");

        // Servo Initialization
        boxServo = hwMap.get(Servo.class,"boxServo");
        cameraServo = hwMap.get(Servo.class,"cameraServo");
        capServo = hwMap.get(Servo.class, "capServo");

        // Motor Directions
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        carouselMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        carouselMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Servo Directions
        boxServo.setDirection(Servo.Direction.FORWARD);
        cameraServo.setDirection(Servo.Direction.FORWARD);
        capServo.setDirection(Servo.Direction.FORWARD);

        // Adjusting the Zero Power Behavior changes how the motors behaved when a
        // Power of 0 is applied.

        // Motor Zero Power Behaviors
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Turn on Run Using Encoder to use the built in PID controller
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Turn on Run Using Encoder to use the built in PID controller
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Arm Encoders
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
}


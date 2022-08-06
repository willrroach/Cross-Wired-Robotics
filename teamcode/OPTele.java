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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="OurTeleOp", group="Linear Opmode")
//@Disabled
public class OPTele extends LinearOpMode {

    // Declare OpMode members.
    ElapsedTime runtime = new ElapsedTime();

    //FRW (Front right wheel) port 0.
    //FLW (Front Left wheel) port 1.
    //BRW (Back right wheel) port 2.
    //BLW (Back left wheel) port 3.
    //COLLM (Collection Motor) port 0 expansion hub
    //Viper-
    //Carousel-
    //Servo- servo-port 0 on expansion hub

    //creates motors
    private DcMotor leftDriveBack;
    private DcMotor leftDriveFront;
    private DcMotor rightDriveBack;
    private DcMotor rightDriveFront;
    private DcMotor collectionMotor;
    private DcMotor carouselMotor;
    private DcMotor viperMotor;

    //speed for precision mode
    private double motSpeed;


    // amount of movement or slew to servo each cycle (Ms cycle)
    private double increment   = 0.001;

    private Servo servoBasket;

    //This is set initially so that it always starts in a good position
    private double position = 0.467;

    private boolean halfWay = false;


    @Override
    public void runOpMode() {

        //Maps all of the hardware to the software variable names. The green names in "" are what appear on the
        //Driver hub config. settings. These must match exactly
        rightDriveFront = hardwareMap.get(DcMotor.class, "FRW");
        leftDriveFront = hardwareMap.get(DcMotor.class, "FLW");
        rightDriveBack = hardwareMap.get(DcMotor.class, "BRW");
        leftDriveBack = hardwareMap.get(DcMotor.class, "BLW");
        collectionMotor = hardwareMap.get(DcMotor.class, "COLLM");
        carouselMotor = hardwareMap.get(DcMotor.class, "CARM");
        viperMotor = hardwareMap.get(DcMotor.class, "VIPM");

        servoBasket = hardwareMap.get(Servo.class, "SERVOBAS");



        //Sets the directions of all DC Motors
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        leftDriveFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection(DcMotorSimple.Direction.FORWARD);
        collectionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        carouselMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        viperMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Tells Driver hub that init has completed
        telemetry.addData("Status", "Initialized");

        //All code before this does not run until the play button is hit (not init)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //All of this is complex trig. that solves for the movement of the mech. omni. wheels. Do not change. The only thing that
            //I changed was the - sign infront of gamepad1.left_stick_y. I made it negative because joysticks always have the "up" direction
            //as negative. Don't worry about this, and don't change it
            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            //This is precision mode. I just go the output powers from the trig. above and divided it by
            //the value of "motSpeed" which is determined if precision mode is on or off
            leftDriveFront.setPower(v1 / motSpeed);
            rightDriveFront.setPower(v2 / motSpeed);
            leftDriveBack.setPower(v3 / motSpeed);
            rightDriveBack.setPower(v4 / motSpeed);

            //The if-statements for precision mode
            if (gamepad1.right_bumper) {
                motSpeed = 2.5;
            } else {
                motSpeed = 1;
            }


            //The collection spinny thingy. Pretty straight forward. If x, then spin. If not, don't.
            if (gamepad1.x) {
                collectionMotor.setPower(1);
            } else if (!gamepad1.x || !gamepad1.back) {
                collectionMotor.setPower(0);
            }


            //The carousel spinner arm. The "ducky arm" if you will. The first statement is clockwise I believe, and the
            //second is counterclockwise. I may be wrong, but Idk. ~0.5 is the sweet spot for the spinning power
            if (gamepad1.y) {
                carouselMotor.setPower(0.5);
            } else if (gamepad1.b) {
                carouselMotor.setPower(-0.5);
            } else {
                carouselMotor.setPower(0);
            }



            runViperSlide();



            //The servo controls. Press b, increase. Press a, decrease.
            if (gamepad1.dpad_right) {
                // Keep stepping up until we hit the max value.
                position += increment;
            }
            if (gamepad1.dpad_left) {
                // Keep stepping down until we hit the min value.
                position -= increment;
            }

            //Makes it so the position of basket can't go past certain point
            if (position < 0.01699) {
                position = 0.01600;
            }
            if (position > 0.605) {
                position = 0.605;
            }

            //resets servo position
            if (gamepad1.a) {
                position = 0.467;
            }

            //Sets the servo's position
            servoBasket.setPosition(position);


            if (gamepad1.back) {
                collectionMotor.setPower(-1);
            }

            if (gamepad1.start) {
                rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                rightDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }



            // Show the elapsed game time on Driver Hub.
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            //tell DH what the servo's position is, and other things...
            telemetry.addData("Servo Position", position);
            telemetry.addData("Viper Slide Position:", viperMotor.getCurrentPosition());
            telemetry.addData("Right Front Position:", rightDriveFront.getCurrentPosition());
            telemetry.addData("Left Front Position:", leftDriveFront.getCurrentPosition());
            telemetry.addData("Right Back Position:", rightDriveBack.getCurrentPosition());
            telemetry.addData("Left Back Position:", leftDriveBack.getCurrentPosition());
            telemetry.addData("The Truth", "dat bald head of Dr. Arnold is hot");

            //Needs this statement at end to update the driver hub
            telemetry.update();
        }
    }

    public void runViperSlide() {

        if (gamepad1.dpad_down) {
            //neutral / 0 revs
            position = 0.467;
            servoBasket.setPosition(position);
            if (halfWay) {
                sleep(500);
            }
            moveViperSlide(1, 0);
            halfWay = false;
        } else if (gamepad1.dpad_up) {
            //all the way up
            moveViperSlide(1, 2235);
            position = 0.036;
            servoBasket.setPosition(position);
            halfWay = false;
        } else if (gamepad1.left_bumper) {
            //just a little
            moveViperSlide(1, 412);
            position = 0.036;
            servoBasket.setPosition(position);
            halfWay = true;
        }
    }

    public void moveViperSlide(double power, int distance) {

        viperMotor.setTargetPosition(distance);

        viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        viperMotor.setPower(power);

        while (viperMotor.isBusy()) {

        }

        viperMotor.setPower(0);
    }
}
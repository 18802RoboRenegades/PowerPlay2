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

package org.firstinspires.ftc.teamcode.PowerPlayCode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="__System Test", group="Test Mode")

public class SystemTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Hardware3 scout = new Hardware3();
    private double servoPosition = 0;
    private double trapdoor = 0;
    private double clawPositionValue = 0.5;
    private int elevatorPosition = 0;


    @Override
    public void runOpMode() {
        scout.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        /**leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");**/

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        /**leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);**/

        double strafe;
        double tempPosition = 0.3;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.dpad_up) {
                scout.leftFrontWheelMotor.setPower(1);
            } else {
                scout.leftFrontWheelMotor.setPower(0);
            }

            if(gamepad1.dpad_down) {
                scout.leftRearWheelMotor.setPower(1);
            } else {
                scout.leftRearWheelMotor.setPower(0);
            }

            if(gamepad1.dpad_left) {
                scout.rightFrontWheelMotor.setPower(1);
            } else {
                scout.rightFrontWheelMotor.setPower(0);
            }

            if(gamepad1.dpad_right) {
                scout.rightRearWheelMotor.setPower(1);
            } else {
                scout.rightRearWheelMotor.setPower(0);
            }

            if(gamepad1.y) {
                scout.leftFrontWheelMotor.setPower(1);
                scout.leftRearWheelMotor.setPower(1);
                scout.rightFrontWheelMotor.setPower(1);
                scout.rightRearWheelMotor.setPower(1);
            } else {
                scout.leftFrontWheelMotor.setPower(0);
                scout.leftRearWheelMotor.setPower(0);
                scout.rightFrontWheelMotor.setPower(0);
                scout.rightRearWheelMotor.setPower(0);
            }

            if(gamepad1.right_bumper) {
                elevatorPosition += 10;
            } else if (gamepad1.left_bumper) {
                elevatorPosition -= 10;
            }
            double liftPower = scout.GOING_UP_SPEED;

            if (scout.elevatorMotor.getCurrentPosition() > elevatorPosition) {
                liftPower = scout.GOING_DOWN_SPEED;
            } else {
                liftPower = scout.GOING_UP_SPEED;
            }
            scout.elevatorMotor.setTargetPosition(elevatorPosition);
            scout.elevatorMotor.setPower(liftPower);

            if(gamepad1.right_trigger > 0) {
                clawPositionValue += 0.02;
            } else if (gamepad1.left_trigger > 0) {
                clawPositionValue -= 0.02;
            }
            scout.clawServo.setPosition(clawPositionValue);

            telemetry.addData("Claw Position = ", clawPositionValue);
            telemetry.addData("Right Front Encoder = ", scout.rightFrontWheelMotor.getCurrentPosition());
            telemetry.addData("Right Rear Encoder = ", scout.rightRearWheelMotor.getCurrentPosition());
            telemetry.addData("Left Front Encoder = ", scout.leftFrontWheelMotor.getCurrentPosition());
            telemetry.addData("Left Rear Encoder = ", scout.leftRearWheelMotor.getCurrentPosition());
            telemetry.addData("Elevator Encoder = ", scout.elevatorMotor.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

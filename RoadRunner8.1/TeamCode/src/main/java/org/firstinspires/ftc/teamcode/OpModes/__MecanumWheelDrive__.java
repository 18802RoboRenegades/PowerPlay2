package org.firstinspires.ftc.teamcode.OpModes;


import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanumFTCLib;

@TeleOp(name="__MecanumWheelDrive__", group="LinearOpMode")

/**

 This is the DriveOpMode. This is the OpMode that is used for the driver-controlled portion, and
 is also sometimes used for testing.

 notes:

 The armPosition for the high level of the team shipping hub is -0.588.

 **/

public class __MecanumWheelDrive__ extends LinearOpMode
{

    private final static HWProfile2 robot = new HWProfile2();
    private LinearOpMode opMode = this;
    public DriveMecanumFTCLib drive = new DriveMecanumFTCLib(robot, opMode);
    private DistanceSensor sensorColorRange;
    private Servo servoTest;

    private double DriveSpeed = 1;
    private double TurnSpeed = 1;
    private double StrafeSpeed = 1;

    public void runOpMode()
    {
        robot.init(hardwareMap);
        telemetry.addData("Status:", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
//        drive.haltandresetencoders();
        //runtime.reset();

        // run until the end of the match (driver presses STOP)
        double stickDrive = 0;
        double turn = 0;
        double strafe = 0;
        double leftPower = 0;
        double rightPower = 0;
//        double armUpDown;
        int armPosition = 0;

        while (opModeIsActive()) {
            stickDrive = this.gamepad1.left_stick_y * DriveSpeed;
            turn = this.gamepad1.right_stick_x * TurnSpeed;
            strafe = this.gamepad1.left_stick_x * StrafeSpeed;

           drive.StrafeDrive(stickDrive, turn, strafe);
//            robot.motorLF.set(leftPower - strafe);
 //           robot.motorLR.set(leftPower + strafe);
 //           robot.motorRF.set(rightPower + strafe);
 //           robot.motorRR.set(rightPower - strafe);

//            armUpDown = this.gamepad2.left_stick_y * ArmUpDownSpeed;


            if(this.gamepad2.y) {
                armPosition = robot.LIFT_HIGH_JUNCTION;
//                drive.liftPosition(robot.LIFT_HIGH_JUNCTION);
            } else if(this.gamepad2.a) {
                armPosition = robot.LIFT_RESET;
//                drive.liftPosition(robot.LIFT_RESET);
            } else if(this.gamepad2.b){
                armPosition = robot.LIFT_LOW_JUNCTION;
//                drive.liftPosition(robot.LIFT_LOW_JUNCTION);
            } else if(this.gamepad2.x) {
                armPosition = robot.LIFT_MID_JUNCTION;
//                drive.liftPosition(robot.LIFT_MID_JUNCTION);
            } else if (armPosition > robot.LIFT_RESET) {
                armPosition = robot.LIFT_RESET;
            }

            if(this.gamepad2.dpad_up) {
                armPosition = robot.LIFT_CONE5;
            } else if(this.gamepad2.dpad_down) {
                armPosition = robot.LIFT_CONE2;
            } else if(this.gamepad2.dpad_left) {
                armPosition = robot.LIFT_CONE4;
            } else if(this.gamepad2.dpad_right) {
                armPosition = robot.LIFT_CONE3;
            }

            if (this.gamepad2.left_stick_y  > 0.05 || this.gamepad2.left_stick_y < -0.05) {
                armPosition += this.gamepad2.left_stick_y * 10;
            }

            // -0.065

            if(this.gamepad2.right_bumper) {
                drive.closeClaw();
                //                    drivem.closeClaw();
            }

            if(this.gamepad2.left_bumper) {
                drive.openClaw();
            }

//            drive.StrafeDrive(stickDrive, turn, strafe);

            robot.motorBase.setPower(1);
            robot.motorBase.setTargetPosition(armPosition);
//            robot.elevatorMotor.setPower(1);
//            scout.elevatorMotor.setTargetPosition(armPosition);

            //drive.StrafeDrive(stickDrive, turn, strafe);
            //telemetry.addData("bruhdatderepositionrightdere = ", trapdoor);
            telemetry.addData("Status", "Running");
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();

        }
    }

}

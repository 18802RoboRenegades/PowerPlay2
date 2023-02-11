package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanumFTCLib;
import org.firstinspires.ftc.teamcode.PowerPlayCode.DriveClassEnergize;
import org.firstinspires.ftc.teamcode.PowerPlayCode.Hardware3;

@TeleOp(name="__MecanumWheelDrive__", group="LinearOpMode")

/**

 This is the DriveOpMode. This is the OpMode that is used for the driver-controlled portion, and
 is also sometimes used for testing.

 notes:

 The armPosition for the high level of the team shipping hub is -0.588.

 **/

public class __MecanumWheelDrive__ extends LinearOpMode
{

    private DistanceSensor sensorColorRange;
    private Servo servoTest;
    // The line of code below is defining the robot.
    private Hardware3 scout = new Hardware3();
    private HWProfile2 robot = new HWProfile2();
    private double DriveSpeed = 1;
    private double TurnSpeed = 1;
    private double StrafeSpeed = 1;
    private double ArmUpDownSpeed = 0.3;
    private double trapdoor = 0;
    private boolean HandIsOpen = false;
    double ArmVerticalPower = 0;
    private LinearOpMode myOpMode;
    //private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode()
    {
        double strafePower = 0;
        double servoPosition = 0;
        double arm2Position = 0;
        double arm2speed = 0;
        if (scout == null)
        {
            scout = new Hardware3();
        }
        scout.init(hardwareMap);
        DriveClassEnergize drive = new DriveClassEnergize(scout, myOpMode);
        DriveMecanumFTCLib drivem = new DriveMecanumFTCLib(robot, myOpMode);
        telemetry.addData("Status:", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        drive.haltandresetencoders();
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
            stickDrive = gamepad1.left_stick_y * DriveSpeed;
            turn = this.gamepad1.right_stick_x * TurnSpeed;
            strafe = this.gamepad1.left_stick_x * StrafeSpeed;
//            armUpDown = this.gamepad2.left_stick_y * ArmUpDownSpeed;


            if(this.gamepad2.y) {
                armPosition = robot.LIFT_HIGH_JUNCTION;
//                drive.liftPosition(robot.LIFT_HIGH_JUNCTION);
            } else if(this.gamepad2.a) {
                armPosition =  robot.LIFT_RESET;
//                drive.liftPosition(robot.LIFT_RESET);
            } else if(this.gamepad2.b){
                armPosition = robot.LIFT_LOW_JUNCTION;
//                drive.liftPosition(robot.LIFT_LOW_JUNCTION);
            } else if(this.gamepad2.x) {
                armPosition = robot.LIFT_MID_JUNCTION;
//                drive.liftPosition(robot.LIFT_MID_JUNCTION);
            } else if(this.gamepad2.dpad_up){
                armPosition = armPosition + 10;
            } else if(this.gamepad2.dpad_down){
                armPosition = armPosition - 10;
            }
            // -0.065

            drive.liftPosition(armPosition);
            if(this.gamepad2.right_bumper) {
                    drivem.closeClaw();
            }

            if(this.gamepad2.left_bumper) {
                    drivem.openClaw();
            }

             leftPower    = -Range.clip(stickDrive - turn, -scout.MAX_DRIVING_POWER, scout.MAX_DRIVING_POWER);
             rightPower   = -Range.clip(stickDrive + turn, -scout.MAX_DRIVING_POWER, scout.MAX_DRIVING_POWER);
             strafePower = Range.clip(-strafe, -scout.MAX_DRIVING_POWER, scout.MAX_DRIVING_POWER);

            scout.leftFrontWheelMotor.setPower(leftPower - strafePower);
            scout.leftRearWheelMotor.setPower(leftPower + strafePower);
            scout.rightFrontWheelMotor.setPower(rightPower + strafePower);
            scout.rightRearWheelMotor.setPower(rightPower - strafePower);
            scout.elevatorMotor.setPower(1);
            scout.elevatorMotor.setTargetPosition(armPosition);

            //drive.StrafeDrive(stickDrive, turn, strafe);
            //telemetry.addData("bruhdatderepositionrightdere = ", trapdoor);
            telemetry.addData("Status", "Running");
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();

        }
    }
}

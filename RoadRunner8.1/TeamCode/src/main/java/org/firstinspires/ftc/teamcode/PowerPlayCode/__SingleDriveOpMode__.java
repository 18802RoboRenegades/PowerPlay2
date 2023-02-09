package org.firstinspires.ftc.teamcode.PowerPlayCode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PowerPlayCode.DriveClassEnergize;
import org.firstinspires.ftc.teamcode.PowerPlayCode.Hardware3;

@TeleOp(name="__SingleDriveOpMode__", group="LinearOpMode")

/**

 This OpMode is basically the same as DriveOpMode, but only intended for one driver.

 **/
 public class __SingleDriveOpMode__ extends LinearOpMode
{

    private DistanceSensor sensorColorRange;
    private Servo servoTest;
    // The line of code below is defining the robot.
    private Hardware3 scout = new Hardware3();
    private double DriveSpeed = 1;
    private double TurnSpeed = 1;
    private double ArmSensitivity = 1;
    private double ArmSensitivity2 = 1;
    private boolean HandIsOpen = false;
    double arm2Position = 0;
    double arm2speed = 0;
    private LinearOpMode myOpMode;

    public void runOpMode()
    {
        if (scout == null)
        {
            scout = new Hardware3();
        }
        scout.init(hardwareMap);
        DriveClassEnergize drive = new DriveClassEnergize(scout, myOpMode);
        telemetry.addData("Status:", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        double ArmRotationPower = 0;
        double ArmVerticalPower = 0;
        double Arm2Vertical = 0;
        double stickDrive = 0;
        double turn = 0;
        double strafe = 0;
        double leftPower = 0;
        double rightPower = 0;
        double servoPosition = 0;


        while (opModeIsActive()) {
            stickDrive = this.gamepad1.left_stick_y * DriveSpeed;
            turn = this.gamepad1.right_stick_x * TurnSpeed;
            strafe = this.gamepad1.left_stick_x * DriveSpeed;


            if (this.gamepad1.left_stick_button) {
                DriveSpeed = .4;
            } else {
                DriveSpeed = 1;
            }


            if(this.gamepad1.x) {
                if (HandIsOpen == true) {
                    drive.clawwwClose();  // NEEDS A VALUE FOR THE POSITION OF THE CLOSED CLAW!
                    HandIsOpen = false;
                    sleep(300);
                } else if (HandIsOpen == false){
                    HandIsOpen = true;
                    drive.clawwwOpen();  //NEEDS A VALUE FOR THE POSITION OF THE OPEN CLAW!
                    sleep(300);
                }
            }

            drive.StrafeDrive(stickDrive, turn, strafe);
            telemetry.addData("Right Front Encoder = ", scout.rightFrontWheelMotor.getCurrentPosition());
            telemetry.addData("Right Rear Encoder = ", scout.rightRearWheelMotor.getCurrentPosition());
            telemetry.addData("Left Front Encoder = ", scout.leftFrontWheelMotor.getCurrentPosition());
            telemetry.addData("Left Rear Encoder = ", scout.leftRearWheelMotor.getCurrentPosition());
            telemetry.addData("Arm 1 Position = ", servoPosition);
            telemetry.addData("Arm 2 Power = ", ArmVerticalPower);
            telemetry.addData("Status", "Running");
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();

        }
    }
}

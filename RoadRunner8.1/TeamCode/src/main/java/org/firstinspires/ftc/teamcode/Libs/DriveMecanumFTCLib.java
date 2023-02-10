package org.firstinspires.ftc.teamcode.Libs;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;

public class DriveMecanumFTCLib {

    private HWProfile2 robot;
    public LinearOpMode opMode;

    FtcDashboard dashboard;

    /*
     * Constructor method
     */
    public DriveMecanumFTCLib(HWProfile2 myRobot, LinearOpMode myOpMode){
        robot = myRobot;
        opMode = myOpMode;

    }   // close DriveMecanum constructor Method


    /* #########################################################################################
       #########################################################################################
       ################################  DRIVE METHODS #########################################
       #########################################################################################
       #########################################################################################
     */

    /******************************************************************************************
     * Method:      ftclibDrive
     * Function:    Robot drives the direction of the heading, at the power provided,
     *              for the distance provided
     * Note:        This function is intended to work at 0, 90, 180, and -90 headings
     * Parameters:
     * @param heading   - Direction robot should drive
     * @param distance  - Distance in Inches to drive
     */
    public void driveDistance(double heading, double distance) {
        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();
        double initZ = getZAngle();
        double rflrPower = 0;
        double lfrrPower = 0;
        double currentZ, zCorrection, distanceTraveled = 0;
        boolean active = true;
        double derivative, lastError=0, error;
        double integral, drivePower;
        ElapsedTime rotateTime = new ElapsedTime();
        double maxDrivePower = 0.45;
        double Kp = 0.05;
        double Ki = 0.001;
        double Kd = 0.01;
        double minSpeed = 0.15;
        double theta = Math.toRadians(90 + heading);

        robot.motorLF.resetEncoder();
        robot.motorLR.resetEncoder();
        robot.motorRF.resetEncoder();
        robot.motorRR.resetEncoder();

        opMode.sleep(100);  // allow time for encoder resets
        // make sure distance is positive. Use heading to change direction.
        distance = Math.abs(distance);

            while (opMode.opModeIsActive() && active) {

                error = distance - distanceTraveled;
                derivative = lastError - error;
                integral = rotateTime.time() * error;
                drivePower = ((Kp * error) + (Ki * integral) + (Kd * derivative));
                lastError = error;


                if (drivePower > -0.15 && drivePower < 0 ){
                    drivePower = minSpeed;
                } else if (drivePower <0.15 && drivePower > 0){
                    drivePower = minSpeed;
                }
                rflrPower = drivePower * (Math.sin(theta) - Math.cos(theta));
                lfrrPower = drivePower * (Math.sin(theta) + Math.cos(theta));

                if (initZ > 170 || initZ < -170) {
                    currentZ = gyro360(0);      // always use 0 as the reference angle
                } else {
                    currentZ = getZAngle();
                }
                if (currentZ != initZ) {
                    zCorrection = Math.abs(initZ - currentZ) / 100;

                    if (initZ < currentZ) {
                        rflrPower = rflrPower + zCorrection;
                        lfrrPower = lfrrPower - zCorrection;
                    }
                    if (initZ > currentZ) {
                        rflrPower = rflrPower - zCorrection;
                        lfrrPower = lfrrPower + zCorrection;
                    }
                }   // end of if currentZ != initZ

                /*
                 * Limit that value of the drive motors so that the power does not exceed 100%
                 */
                rflrPower = Range.clip(rflrPower, -Math.abs(maxDrivePower), Math.abs(maxDrivePower));
                lfrrPower = Range.clip(lfrrPower, -Math.abs(maxDrivePower), Math.abs(maxDrivePower));

                /*
                 * Apply power to the drive wheels
                 */
                setDrivePower(rflrPower, lfrrPower, rflrPower, lfrrPower);

                distanceTraveled = calcDistance(heading);
                if (distanceTraveled >= distance) {
                    active = false;
                }
                opMode.telemetry.addData("Distance Traveled = ", distanceTraveled);
                opMode.telemetry.addData("Distance = ", distance);
                opMode.telemetry.update();
                opMode.idle();

                dashTelemetry.put("p20 - drive Telemetry Data", "");
                dashTelemetry.put("p25 - Drive Power                  = ", drivePower);
                dashTelemetry.put("p24 - Target Distance              = ", distance);
                dashTelemetry.put("p21 - rflrPower                    = ", rflrPower);
                dashTelemetry.put("p22 - lfrrPower                    = ", lfrrPower);
                dashTelemetry.put("p26 - Distance Traveled            = ", distanceTraveled);
                dashTelemetry.put("p23 - PID IMU Angle X              = ", robot.imu.getAngles()[0]);
                dashboard.sendTelemetryPacket(dashTelemetry);

            }   // end of while (opMode.opModeIsActive() && active) loop
            motorsHalt();
            setMotorVelocityZero();
            setDrivePower(0,0,0,0);

            dashTelemetry.put("p20 - drive Telemetry Data", "");
            dashTelemetry.put("p24 - Target Distance              = ", distance);
            dashTelemetry.put("p21 - rflrPower                    = ", rflrPower);
            dashTelemetry.put("p22 - lfrrPower                    = ", lfrrPower);
            dashTelemetry.put("p26 - Distance Traveled            = ", distanceTraveled);
            dashTelemetry.put("p23 - PID IMU Angle X              = ", robot.imu.getAngles()[0]);
            dashboard.sendTelemetryPacket(dashTelemetry);

    }   // close driveDistance method


    private void setMotorVelocityZero(){
        robot.motorLF.setVelocity(0);
        robot.motorRF.setVelocity(0);
        robot.motorLR.setVelocity(0);
        robot.motorRR.setVelocity(0);
    }
    /**
     * Method: PIDRotate
     * Parameters:
     * @param targetAngle -> desire ending angle/position of the robot
     * @param targetError -> how close should the robot get to the desired angle
     */
    public void pidRotate(double targetAngle, double targetError){
        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();

        double integral = 0;
        ElapsedTime rotateTime = new ElapsedTime();
        double error;
        double Kp = 0.005;
        double Ki = 0.0; //0.001;
        double Kd = 0.0001; //0.02;
        double minRotateSpeed = 0.15;
        double maxRotateSpeed = 0.5;
        double rotationSpeed;
        double derivative, lastError=0;
        double rightRotate;
        double leftRotate;

        // check to see how far the robot is rotating to decide which gyro sensor value to use

        targetAngle = Math.toRadians(targetAngle);  // convert targetAngle to radians
        error = 100 * (getZAngleRadians() - targetAngle);

        // reset the time to track rotation speed
        rotateTime.reset();

        while ((Math.abs(error) >= targetError) && opMode.opModeIsActive()) {
            derivative = (error - lastError) / rotateTime.time();
            integral = integral + (rotateTime.time() * error);
            rotationSpeed = ((Kp * error) + (Ki * integral) + (Kd * derivative));
            lastError = error;

            // Clip motor speed
            rotationSpeed = clamp(rotationSpeed, -maxRotateSpeed, maxRotateSpeed);

            if ((rotationSpeed > -0.15) && (rotationSpeed < 0)) {
                rotationSpeed = -minRotateSpeed;
            } else if ((rotationSpeed < 0.15) && (rotationSpeed > 0)) {
                rotationSpeed = minRotateSpeed;
            }

            leftRotate = -rotationSpeed;
            rightRotate = rotationSpeed;

            setDrivePower(rightRotate, leftRotate, leftRotate, rightRotate);

            // check to see how far the robot is rotating to decide which gyro sensor value to use
            error = 100 * (getZAngleRadians() - targetAngle);

            dashTelemetry.put("p00 - PIDTurn Telemetry Data", "");
            dashTelemetry.put("p01 - PID IMU Angle X              = ", getZAngle());
            dashTelemetry.put("p02 - PID IMU Angle Y              = ", robot.imu.getAngles()[1]);
            dashTelemetry.put("p03 - PID IMU Angle Z              = ", robot.imu.getAngles()[2]);
            dashTelemetry.put("p04 - targetAngle (Radians)        = ", targetAngle);
            dashTelemetry.put("p05 - Current Angle (Radians)      = ", getZAngleRadians());
            dashTelemetry.put("p06 - Angle Error (Radians)        = ", error/100);
            dashTelemetry.put("p07 - Angle Error (Degrees)        = ", Math.toDegrees(getZAngleRadians()-targetAngle));
            dashTelemetry.put("p08 - derivative                   = ", derivative);
            dashTelemetry.put("p09 - integral                     = ", integral);
            dashTelemetry.put("p10 - Turn Time                    = ", rotateTime.time());
            dashboard.sendTelemetryPacket(dashTelemetry);

        }   // end of while Math.abs(error)
        motorsHalt();

    }   //end of the PIDRotate Method

    /******************************************************************************************
     * Sets power to all four drive motors
     * Method:
     * @param RF    - power for right front motor
     * @param LF    - power for left front motor
     * @param LR    - power for left rear motor
     * @param RR    - power for right rear motor
     ******************************************************************************************/
    public void setDrivePower(double RF, double LF, double LR, double RR){
        robot.motorRF.set(RF);
        robot.motorLF.set(LF);
        robot.motorLR.set(LR);
        robot.motorRR.set(RR);
    }   // end of setDrivePower method

    /******************************************************************************************
     * Method:      motorsHalt
     * Function:    Shut off all drive motors
     ******************************************************************************************/
    public void motorsHalt(){
        robot.motorRF.set(0);
        robot.motorLF.set(0);
        robot.motorLR.set(0);
        robot.motorRR.set(0);
    }   // end of motorsHalt method


    /* #########################################################################################
       #########################################################################################
       ################################  MECHANISM METHODS #####################################
       #########################################################################################
       #########################################################################################
     */

    public void closeClaw(){
        robot.clawServo.setPosition(robot.SERVO_GRAB_CLOSE);
    }   // end of closeClaw method

    public void openClaw(){
        robot.clawServo.setPosition(robot.SERVO_GRAB_OPEN);
    }   // end of openClaw method

    public void liftReset(){
        robot.motorBase.setPower(robot.LIFT_POWER);
        robot.motorBase.setTargetPosition(robot.LIFT_RESET);
    }   // end of liftReset method

    public void liftLowJunction(){
        robot.motorBase.setPower(robot.LIFT_POWER);
        robot.motorBase.setTargetPosition(robot.LIFT_LOW_JUNCTION);
    }   // end of liftLowJunction method

    public void liftMidJunction(){
        robot.motorBase.setPower(robot.LIFT_POWER);
        robot.motorBase.setTargetPosition(robot.LIFT_MID_JUNCTION);
    }   // end of liftMidJunction method

    public void liftHighJunction(){
        robot.motorBase.setPower(robot.LIFT_POWER);
        robot.motorBase.setTargetPosition(robot.LIFT_HIGH_JUNCTION);
    }   // end of liftHighJunction method

    public void liftPosition(int targetPosition){
        robot.motorBase.setPower(robot.LIFT_POWER);
        robot.motorBase.setTargetPosition(targetPosition);
    }   // end of liftPosition method


    /*******************************************************************************************
     * Method:      calcDistance
     * Function:    Calculates the distance that the robot has traveled based on a starting set of
     *              encoder values and the current encoder values
     * Parameters:
     * @param heading   - indicates the direction the robot is angled/heading
     * @return  distanceTraveled
     *******************************************************************************************/
    public double calcDistance(double heading){

        double strafeFactor = 1;        // defaults to 1; changed if the robot is strafing

        if(heading == 90 || heading == -90){
            strafeFactor = robot.STRAFE_FACTOR;
        }

        int totEncoder = Math.abs(robot.motorRF.getCurrentPosition()) + Math.abs(robot.motorLF.getCurrentPosition())
                + Math.abs(robot.motorRR.getCurrentPosition()) + Math.abs(robot.motorLR.getCurrentPosition());

        double avgEncoder = totEncoder/ 4;

        double distanceTraveled = avgEncoder / robot.DRIVE_TICKS_PER_INCH;

        return Math.abs(distanceTraveled * strafeFactor);
    }

    /******************************************************************************************
     * Method:  getZAngle()
     ******************************************************************************************/
    public double getZAngle(){
        return (-robot.imu.getAbsoluteHeading());
    }   // close getZAngle method

    /******************************************************************************************
     * Method:  getZAngleRadians()
     ******************************************************************************************/
    public double getZAngleRadians(){
        return (Math.toRadians(-robot.imu.getAbsoluteHeading()));
    }   // close getZAngle method

    /* #########################################################################################
       #########################################################################################
       ################################  CLASS CALCULATIONS ####################################
       #########################################################################################
       #########################################################################################
     */


    /*******************************************************************************************
     * Method gyro360
     *  - Causes the Gyro to behave in 360 mode instead of 180 degree mode
     *******************************************************************************************/
    public double gyro360(double targetAngle){
        double currentZ = getZAngle();
        double rotationalAngle;

        if (targetAngle > 0){
            if ((currentZ >= 0) && (currentZ <= 180)) {
                rotationalAngle = currentZ;
            } else {
                rotationalAngle = 180 + (180 + currentZ);
            }// end if(currentZ <=0) - else
        } else {
            if ((currentZ <= 0) && (currentZ >= -180)) {
                rotationalAngle = currentZ;
            } else {
                rotationalAngle = -180 - (180 - currentZ);
            }   // end if(currentZ <=0) - else
        }   // end if(targetAngle >0)-else

        return rotationalAngle;
    }   // end method gyro360

    /*******************************************************************************************
     * Method gyro360
     *  - Causes the Gyro to behave in 360 mode instead of 180 degree mode
     *******************************************************************************************/
    public double gyro360Radians(double targetAngle){
        double rotationalAngle = 0;

        if(targetAngle > Math.PI){
            rotationalAngle = targetAngle - (2 * Math.PI);
        }
        if(targetAngle < -Math.PI){
            rotationalAngle = targetAngle + (2 * Math.PI);
        }

        return rotationalAngle;
    }   // end method gyro360

}   // close the class

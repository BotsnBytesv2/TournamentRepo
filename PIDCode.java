package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import java.util.List;
import java.lang.Math.*;

@Autonomous(name = "Gyro_Auto", group = "Concept")

public class Gyro_Auto extends LinearOpMode {
	
	private static DcMotor frontleftmotor = null;
    private static DcMotor frontrightmotor = null;
    private static DcMotor backleftmotor = null;
    private static DcMotor backrightmotor = null;
	BNO055IMU               imu;
	Orientation             lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
	static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * Math.PI);
	static final double 	STRAFE_PER_INCH         = 500 ;

    static final double     DRIVE_SPEED             = 0.7;     
    static final double     TURN_SPEED              = 0.5;     

    static final double     HEADING_THRESHOLD       = 1 ;      
    static final double     P_TURN_COEFF            = 0.1;     
    static final double     P_DRIVE_COEFF           = 0.15;     

	


	@Override
	public void runOpMode() {
		frontleftmotor = hardwareMap.get(DcMotor.class, "frontleftmotor");
        frontrightmotor = hardwareMap.get(DcMotor.class, "frontrightmotor");
        backrightmotor = hardwareMap.get(DcMotor.class, "backrightmotor");
        backleftmotor = hardwareMap.get(DcMotor.class, "backleftmotor");
		frontleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		backleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		stopMotor();
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();
		telemetry.addData("Mode", "running");
        telemetry.update();

        while (opModeIsActive())
        {
            unhook();
			
			
		
	}
	}
	
	private void initVuforia() {
         
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
         
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine
    }
    
    private void readPosition(){
        
      if (tfod != null) {
         //position = LEFT;
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected by BnB is ", updatedRecognitions.size());
                      if (updatedRecognitions.size() == 2) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                          //telemetry.addData("Value of object place:", recognition.getLeft());
                          if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getRight();
                            telemetry.addData("Value of object place:", recognition.getLeft());
                          } else if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                              if (silverMineral1X == -1){
                            silverMineral1X = (int) recognition.getLeft();
                              }
                              else {
                                  silverMineral2X = (int) recognition.getLeft();
                              }
                          }
                          
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1) {
                          
                          telemetry.addData("Gold Mineral goldMineralX ",goldMineralX);
                          telemetry.addData("Gold Mineral silverMineral1X ", silverMineral1X);
                         // telemetry.addData("Gold Mineral silverMineral2X ",silverMineral2X );
                          
                          
                          if (goldMineralX < silverMineral1X) {
                            telemetry.addData("Gold Mineral Position", "Center");
                            position = CENTER;
                            
                          } else {
                            telemetry.addData("Gold Mineral Position", "Right");
                            position = RIGHT;
                          }
                        }
                        else if(silverMineral2X != -1 && silverMineral1X != -1) {
                          
                          //telemetry.addData("Gold Mineral goldMineralX ",goldMineralX);
                          telemetry.addData("Gold Mineral silverMineral1X ", silverMineral1X);
                          telemetry.addData("Gold Mineral silverMineral2X ",silverMineral2X );
                  
                            telemetry.addData("Gold Mineral Position", "Left");
                            position = LEFT;
                          }
                        }
                      telemetry.update();
                    }
                }
    }
        private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters();
        //Use above code in game to save battery
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        //tfodParameters.useObjectTracker = true;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    private void moveToMineral(){
      if (position == LEFT){
        leftPath();
      }
      else if (position == CENTER){
        centerPath();
      }else if (position == RIGHT){
        rightPath();
      }else{
        defaultPath();
      }
      }
	
	private void unhang() {
        hanging1.setPower(-0.7);
        hanging2.setPower(-0.7);
        sleep(4800);
        stopMotor();
        TurnFromCurrentPos();
        
    }
    
     private void unhook() {
        //unhang();
        DistanceFormula(0.75, 0.75, 0.8, 0.8, 'l', 's');
        DistanceFormula(0.8, 0.8, 1, 1.1, 'f', 'n');
        DistanceFormula(1, 1.1, 1, 1, 'r','s');
        // telemetry.addLine("this works");
        // telemetry.update();
        
     }
     
    private void centerPath(){
        DistanceFormula(1, 1, 2.5, 2.5, 'f', '*');
        rotate(-45, power);
        dropmarker();
        sleep(1000);
        DistanceFormula(2.5, 2.5, 2.5, 2.8, 'l', 's');
        DistanceFormula(2.9, 2.9, -1, 2.5, 'b', 'n');
    }
    
    private void rightPath(){
        DistanceFormula(1, 1, 1.2, 1.2, 'f', '*');
        DistanceFormula(1.2, 1.2, 1.5, 0.75, 'r', 's');
        DistanceFormula(1.5, 0.75, 2.7, 1.8, 'f', 'n');
        
        DistanceFormula(2.5, 1.6, 2.5, 2.5, 'f', '*');
        rotate(-90, power);
        dropmarker();
        sleep(1000);
        DistanceFormula(2.5, 2.5, -1, 2.5, 'b', 'n');
    }
    
    private void leftPath(){
        DistanceFormula(1, 1, 1.2, 1.2, 'f', '*');
        DistanceFormula(1.2, 1.2, 0.95, 0.8, 'l', 's');
        DistanceFormula(0.95, 0.8, 1.8, 2.5, 'f', 'n');
        rotate(-45, power);
        DistanceFormula(1.8, 2.5, 2.5, 2.5, 'f', 'n');
        DistanceFormula(2.5, 2.5, 2.5, 2, 'r', 's');
        dropmarker();
        sleep(1000);
        DistanceFormula(2.8, 2, 2.8, 2.5, 'l', 's');
        DistanceFormula(2.8, 2.7, -1, 2.7, 'b', 'n');
        
        
        
    }
    
     private void dropmarker() {
         output.setPosition(1.0);
         sleep(1000);
         output.setPosition(0.2);
    }
	private static void DistanceFormula(double x2, double y2, double x3, double y3, char direction) {
      //Calculating Distance from points (x2, y2) & (x3, y3)
      double final_x23 = Math.pow(x3-x2, 2);
      double final_y23 = Math.pow(y3-y2, 2);
      double distance23 = (Math.sqrt(final_x23+final_y23));
      double distance = Math.rint(distance23 * 24);
      //*******************************************************
	  //*******************************************************
      //Calculating coordinates of (x1, y1)
      double degrees = getAngle(); //gyro gives angle
      double robotlength = 0.75;
      double radians = Math.toRadians(degrees+180);
      double x1 = x2 - (Math.sin(radians))*robotlength;
      double y1 = y2 - (Math.cos(radians))*robotlength;  
      //*******************************************************
      //Calculating Distance from points (x1, y1) & (x2, y2)
      double final_x12 = Math.pow(x2-x1, 2);
      double final_y12 = Math.pow(y2-y1, 2);
      double distance12 = (Math.sqrt(final_x12+final_y12));
      //********************************************************
      //Calculating Distance from points (x1, y1) & (x3, y3)
      double final_x13 = Math.pow(x3-x1, 2);
      double final_y13 = Math.pow(y3-y1, 2);
      double distance13 = (Math.sqrt(final_x13+final_y13));
      //********************************************************
      //Calculating angle from points (x1, y1) & (x2, y3) & (x3, y3)
      //coordinate (x2, y2) is vertex of the angle      
      double numerator = (Math.pow(distance12, 2) + Math.pow(distance13, 2) - Math.pow(distance23, 2));
      double denominator = (2 * distance12 * distance13);
      double angle = Math.rint(Math.toDegrees(Math.acos(numerator / denominator)));
      //*********************************************************
      //Condtitional Statements to determine smaller or larger angle
      double turn_angle = 0;
      if (x3 > x2) {
        turn_angle = 180 - angle;
      }
      if (x3 < x2) {
        turn_angle = angle - 180;
      }
      if (x3 == x2) {
        if (y3 < y2) {
          turn_angle = angle - 180;
        }

        if (y3 > y2) {
          turn_angle = 180 - angle;
        }
      }
      //*********************************************************
      //Printing all statements
	  if (direction == 'f' || direction == 'b'){
		  forward_backward(direction, angle, distance);
		  }
	  else if (direction == 'l' || direction == 'r') {
		  strafeDirection(direction, distance);
	  }
      System.out.println(distance);
      System.out.println(x1);
      System.out.println(y1);
      System.out.println(turn_angle);
	  }
	  
//**************************************************************************************	  
	
	
	
	public void gyroDrive (double distance) {
		double speed = DRIVE_SPEED;
		double angle = getAngle();
        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.leftDrive.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            frontleftmotor.setTargetPosition(newLeftTarget);
			backleftmotor.setTargetPosition(newLeftTarget);
            frontrightmotor.setTargetPosition(newRightTarget);
			backrightmotor.setTargetPosition(newRightTarget);

            frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            frontleftmotor.setPower(-speed);
            frontrightmotor.setPower(speed);
			backleftmotor.setPower(-speed);
            backrightmotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                   (frontleftmotor.isBusy() && frontrightmotor.isBusy() && backleftmotor.isBusy() && backrightmotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                frontleftmotor.setPower(-leftSpeed);
				backleftmotor.setPower(-leftSpeed);
                frontrightmotor.setPower(rightSpeed);
				backrightmotor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      frontleftmotor.getCurrentPosition(), frontrightmotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            stopMotor();

            // Turn off RUN_TO_POSITION
            frontleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			frontrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			backleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			backrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

//*******************************************************************************************	

	  public void gyroTurn (double angle) {
		double speed = TURN_SPEED;
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }
	
	   boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 	 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        frontleftmotor.setPower(leftSpeed);
		backleftmotor.setPower(leftSpeed);
        backrightmotor.setPower(rightSpeed);
		frontrightmotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
	
//*******************************************************************************
	
	public void gyroStrafe ( double distance) { //positive is right and negative is left
		double speed = DRIVE_SPEED;
		double angle = getAngle();
        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftVector;
        double  rightVector;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * STRAFE_PER_INCH);
            newLeftTarget = robot.leftDrive.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            frontleftmotor.setTargetPosition(newLeftTarget);
			backleftmotor.setTargetPosition(newLeftTarget);
            frontrightmotor.setTargetPosition(newRightTarget);
			backrightmotor.setTargetPosition(newRightTarget);

            frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            frontleftmotor.setPower(-speed);
            frontrightmotor.setPower(-speed);
			backleftmotor.setPower(speed);
            backrightmotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                   (frontleftmotor.isBusy() && frontrightmotor.isBusy() && backleftmotor.isBusy() && backrightmotor.isBusy())) {


                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error1, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer1 *= -1.0;

                leftVector = speed - steer;
                rightVector = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftVector), Math.abs(rightVector));
                if (max > 1.0)
                {
                    leftVector /= max;
                    rightVector /= max;
                }

                frontleftmotor.setPower(-leftVector);
				backleftmotor.setPower(leftVector);
                frontrightmotor.setPower(-rightVector);
				backrightmotor.setPower(rightVector);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      frontleftmotor.getCurrentPosition(), frontrightmotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftVector, rightVector);
                telemetry.update();
            }

            // Stop all motion;
            stopMotor();

            // Turn off RUN_TO_POSITION
            frontleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			frontrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			backleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			backrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }	

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    
//*********************************************************************************************	
	public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getAngle();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
//***********************************************************************************
   
   public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

	
//***********************************************************************************	
	 private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
	
//***********************************************************************************	
	 private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
		Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
		double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
		 if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
		lastAngles = angles;
		return globalAngle;
		
    }
	
	
	 private void moveForward(int target){
        frontrightmotor.setTargetPosition(target);
		frontleftmotor.setTargetPosition(target);
		backrightmotor.setTargetPosition(target);
		backleftmotor.setTargetPosition(target);
		frontrightmotor.setPower(frontrightpower);
        frontleftmotor.setPower(-frontleftpower);
        backrightmotor.setPower(backrightpower);
        backleftmotor.setPower(-backleftpower);
        stopMotor();
		}

    private void moveBackward(int target){
		frontrightmotor.setTargetPosition(target);
		frontleftmotor.setTargetPosition(target);
		backrightmotor.setTargetPosition(target);
		backleftmotor.setTargetPosition(target);
        frontrightmotor.setPower(-frontrightpower);
        frontleftmotor.setPower(frontleftpower);
        backrightmotor.setPower(-backrightpower);
        backleftmotor.setPower(backrightpower);
        stopMotor();
		}
	
	private void forward_backward(char direction, int angle, int distance) {
		if (direction == 'f') {
			gyroTurn(angle);
			gyroDrive(distance);}
		
		if (direction == 'b') {
			gyroTurn(angle);
			gyroDrive(-distance);}
	}
    private void spinleft(int target){
        frontrightmotor.setTargetPosition(target);
		frontleftmotor.setTargetPosition(target);
		backrightmotor.setTargetPosition(target);
		backleftmotor.setTargetPosition(target);
		frontrightmotor.setPower(frontrightpower);
        frontleftmotor.setPower(frontleftpower);
        backrightmotor.setPower(backrightpower);
        backleftmotor.setPower(backleftpower);
		stopMotor();
		}

    private void spinright(int targetp){
        frontrightmotor.setTargetPosition(target);
		frontleftmotor.setTargetPosition(target);
		backrightmotor.setTargetPosition(target);
		backleftmotor.setTargetPosition(target);
		frontrightmotor.setPower(-frontrightpower);
        frontleftmotor.setPower(-frontleftpower);
        backrightmotor.setPower(-backrightpower);
        backleftmotor.setPower(-backleftpower);
        stopMotor();
		}

    private void straferight(int target) {
        frontrightmotor.setTargetPosition(target);
		frontleftmotor.setTargetPosition(target);
		backrightmotor.setTargetPosition(target);
		backleftmotor.setTargetPosition(target);
		frontleftmotor.setPower(-frontleftpower);
        backleftmotor.setPower(backleftpower);
        frontrightmotor.setPower(-frontrightpower);
        backrightmotor.setPower(backrightpower);
        stopMotor();
		}

    private void strafeleft(int target) {
        frontrightmotor.setTargetPosition(target);
		frontleftmotor.setTargetPosition(target);
		backrightmotor.setTargetPosition(target);
		backleftmotor.setTargetPosition(target);
		frontleftmotor.setPower(frontleftpower);
        backleftmotor.setPower(-backleftpower);
        frontrightmotor.setPower(frontrightpower);
        backrightmotor.setPower(-backrightpower);
        stopMotor();
		}
		
	private void strafeDirection(char direction, int distance) {
		if (direction == 'l') {
			gyroStrafe(-distance);}
		
		if (direction == 'r') {
			gyroDrive(distance);}
	}
	private void stopMotor() {
        frontleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		backleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		frontleftmotor.setPower(0.0);
        backleftmotor.setPower(0.0);
        frontrightmotor.setPower(0.0);
        backrightmotor.setPower(0.0);
        frontleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		resetAngle();
		}

}
// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved. */
// /* Open Source Software - may be modified and shared by FRC teams. The code
// */
// /* must be accompanied by the FIRST BSD license file in the root directory of
// */
// /* the project. */
// /*----------------------------------------------------------------------------*/

// package org.frogforce503.robot2022.subsystems.swerve;

// import com.ctre.phoenix.sensors.PigeonIMU;
// import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
// import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

// import org.frogforce503.robot2022.Robot;
// import org.frogforce503.robot2022.subsystems.Subsystem;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// /**
// * Add your docs here.
// */
// public class Pigeon extends Subsystem {
// private static Pigeon instance = null;
// private double gyroOffset = 0.0;
// private PigeonIMU pigeon;

// double lastYaw, currentYaw;
// double robotYawRate;

// private Pigeon() {
// try {
// // pigeon can id = 22
// // pigeon = new PigeonIMU(BallIntake.getInstance().getPigeonTalon());
// pigeon = new PigeonIMU(22/* Ports.PIGEON_ID */);
// } catch (Exception e) {
// // System.out.println(e);
// }
// }

// public static Pigeon getInstance() {
// if (instance == null) {
// instance = new Pigeon();
// }
// return instance;
// }

// public static double zeroOffset() {
// return -Pigeon.getInstance().getYaw();
// }

// public void setOffset(double offset) {
// this.gyroOffset = offset;
// }

// public boolean isGood() {
// return (pigeon.getState() == PigeonState.Ready);
// }

// public double getYaw() {
// double[] ypr = new double[3];
// pigeon.getYawPitchRoll(ypr);
// PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
// double heading = (Robot.bot.requestPigeonFlipped *
// pigeon.getFusedHeading(fusionStatus));

// currentYaw = heading;
// robotYawRate = (currentYaw - lastYaw) / 0.02;

// lastYaw = currentYaw;

// heading = boundTo360(heading + gyroOffset);
// SmartDashboard.putNumber("Pigeon Heading", heading);

// return heading;/*-ypr[0]*/
// }

// public double getUnitCircleHeading() {
// return 90 - getYaw();
// }

// public void calibrate() {
// pigeon.enterCalibrationMode(CalibrationMode.Accelerometer);
// while (pigeon.getState() == PigeonState.UserCalibration) {
// System.out.println("calibrating accelerometer");
// }

// }

// public double getPitch() {
// double[] ypr = new double[3];
// pigeon.getYawPitchRoll(ypr);
// return ypr[1];
// }

// public int getUpTime() {

// return pigeon.getUpTime();
// }

// public PigeonState getPigeonState() {
// return pigeon.getState();
// }

// public double getRoll() {
// double[] ypr = new double[3];
// pigeon.getYawPitchRoll(ypr);
// return ypr[2];
// }

// public double getAccumulatedZValue() {
// double[] xyz = new double[3];
// pigeon.getAccumGyro(xyz);
// return xyz[2];
// }

// public double[] getYPR() {
// double[] ypr = new double[3];
// pigeon.getYawPitchRoll(ypr);
// return ypr;
// }

// public void zero() {
// setAngle(0);
// Swerve.getInstance().zeroSwerveStabilization();
// }

// public void setAngle(double angle) {
// pigeon.setFusedHeading(-angle * 64.0, 10);
// pigeon.setYaw(-angle, 10);
// // .println("Pigeon angle set to: " + angle);
// }

// public void setReversed() {
// setAngle(180.0);
// Swerve.getInstance().reverseSwerveStabilization();
// }

// private double boundTo360(double angle_degrees) {
// while (angle_degrees >= 360.0)
// angle_degrees -= 360.0;
// while (angle_degrees < 0.0)
// angle_degrees += 360.0;
// return angle_degrees;
// }

// private void outputToSmartDashboard() {
// SmartDashboard.putString("Pigeon Good", pigeon.getState().toString());
// getYaw();
// }

// @Override
// public void outputTelemetry() {

// outputToSmartDashboard();
// }

// public void printStateStatus() {
// PigeonState state = pigeon.getState();
// if (state != PigeonState.Ready) {
// System.err.println("PIGEON NOT READY. PIGEON IS IN " + state);
// }
// }

// @Override
// public void zeroSensors() {
// zero();
// }

// @Override
// public void stop() {
// }

// @Override
// public void onStart(double timestamp) {

// }

// @Override
// public void onLoop(double timestamp) {

// }

// @Override
// public void onStop(double timestamp) {

// }
// }

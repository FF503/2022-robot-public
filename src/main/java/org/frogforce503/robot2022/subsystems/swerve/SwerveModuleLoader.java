package org.frogforce503.robot2022.subsystems.swerve;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj.Filesystem;

public class SwerveModuleLoader {

    private static SwerveModuleLoader instance = null;

    public static SwerveModuleLoader getInstance() {
        if (instance == null)
            instance = new SwerveModuleLoader();

        return instance;
    }

    private JSONObject moduleSet;

    private SwerveModuleLoader() {
        try {
            loadModuleInfo();
        } catch (IOException | ParseException e) {
            e.printStackTrace();
        }
    }

    private void loadModuleInfo() throws FileNotFoundException, IOException, ParseException {
        moduleSet = (JSONObject) new JSONParser().parse(new FileReader(
                Filesystem.getDeployDirectory().getAbsolutePath() + "/SwerveModules/SwerveMasterFile.json"));
    }

    //@formatter:off
    public void setupModule(SwerveModule module) {
        JSONObject moduleObject = (JSONObject) moduleSet.get(module.moduleName);

        int driveMotorID = Math.toIntExact((Long) moduleObject.get("driveMotorID"));
        int turnMotorID = Math.toIntExact((Long) moduleObject.get("turnMotorID"));
        int turnEncoderID = Math.toIntExact((Long) moduleObject.get("turnEncoderID"));
        double turn_kP = (double) moduleObject.get("turn_kP");
        double turn_kI = (double) moduleObject.get("turn_kI");
        double turn_kD = (double) moduleObject.get("turn_kD");
        double turn_kF = (double) moduleObject.get("turn_kF");
        double drive_kP = (double) moduleObject.get("drive_kP");
        double drive_kI = (double) moduleObject.get("drive_kI");
        double drive_kD = (double) moduleObject.get("drive_kD");
        double drive_KF = (double) moduleObject.get("drive_kF");
        double absoluteZeroDegrees = (double) moduleObject.get("absoluteZeroDegrees");
        boolean driveInvert = (boolean) moduleObject.get("driveInverted");
        boolean turnInvert = (boolean) moduleObject.get("turnInverted");

        module.configure(
                driveMotorID,
                turnMotorID,
                turnEncoderID,
                turn_kP,
                turn_kI,
                turn_kD,
                turn_kF,
                drive_kP,
                drive_kI,
                drive_kD,
                drive_KF,
                absoluteZeroDegrees,
                driveInvert,
                turnInvert
        );
    }
}
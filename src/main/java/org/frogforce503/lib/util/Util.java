package org.frogforce503.lib.util;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.geometry.Translation2d;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.frogforce503.robot2022.Robot;
import org.frogforce503.robot2022.RobotState;
import org.frogforce503.robot2022.subsystems.swerve.SwerveModule;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.List;

/**
 * Contains basic functions that are used often.
 */
public class Util {
    public static final double kEpsilon = 1e-12;
    // private static DMatrixRMaj invertedWheelPositionMatrix;

    /**
     * Prevent this class from being instantiated.
     */
    private Util() {
    }

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static String joinStrings(String delim, List<?> strings) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean allCloseTo(List<Double> list, double value, double epsilon) {
        boolean result = true;
        for (Double value_in : list) {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }

    /**
     * Normalizing a vector scales it so that its norm is 1 while maintaining its direction.
     * If input is a zero vector, return a zero vector.
     *
     * @param vector Vector to get the normalized form of (magnitude = 1).
     * @return r / norm(r) or (0,0)
     */
    public static Translation2d normalize(Translation2d vector) {
        double magnitude = vector.getNorm();
        return (magnitude != 0.0) ? vector.times(1.0 / magnitude) : vector;
    }

    public static double normalize(double current, double test) {
        return Math.max(current, test);
    }

    public static double boundAngleNeg180to180Degrees(double angle) {
        while (angle >= 180.0) {
            angle -= 360.0;
        }
        while (angle < -180.0) {
            angle += 360.0;
        }
        return angle;
    }

    public static double boundAngle0to360Degrees(double angle) {
        while (angle >= 360.0) {
            angle -= 360.0;
        }
        while (angle < 0.0) {
            angle += 360.0;
        }
        return angle;
    }

    public static double boundToScope(double scopeFloor, double scopeCeiling, double argument) {
        double stepSize = scopeCeiling - scopeFloor;
        while (argument >= scopeCeiling) {
            argument -= stepSize;
        }
        while (argument < scopeFloor) {
            argument += stepSize;
        }
        return argument;
    }

    public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    public static boolean shouldReverse(double goalAngle, double currentAngle) {
        goalAngle = boundAngle0to360Degrees(goalAngle);
        currentAngle = boundAngle0to360Degrees(currentAngle);
        double reversedAngle = boundAngle0to360Degrees(currentAngle + 180);
        double angleDifference = Math.abs(goalAngle - currentAngle);
        double reversedAngleDifference = Math.abs(goalAngle - reversedAngle);
        angleDifference = (angleDifference > 180) ? 360 - angleDifference : angleDifference;
        reversedAngleDifference = (reversedAngleDifference > 180) ? 360 - reversedAngleDifference
                : reversedAngleDifference;
        if (reversedAngle < angleDifference) {
            System.out.println(alternateShouldReverse(goalAngle, currentAngle));
        }
        return reversedAngleDifference < angleDifference;
    }

    /**
     * written as a possible alternative to test logic (when tested in a test class,
     * it produced the exact same outputs as the other method so this is not the
     * problem)
     *
     * @param goal Target module heading
     * @param cur Current module heading
     * @return Whether or not the module should reverse drive and azimuth for the maneuver
     */
    public static boolean alternateShouldReverse(double goal, double cur) {
        goal = boundAngle0to360Degrees(goal);
        cur = boundAngle0to360Degrees(cur);
        if (Math.abs(goal - cur) <= 90.0) {
            return false;
        }
        return 360 - Math.abs(goal - cur) > 90.0;
    }

    public static double deadBand(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }


    // public static SwerveModule readSwerveJSON(String file) throws Exception {
    //     file += (!file.endsWith(".json")) ? ".json" : "";
    //     // parsing file "JSONExample.json"
    //     Object obj = new JSONParser()
    //             .parse(new FileReader(Filesystem.getDeployDirectory().getAbsolutePath() + "/SwerveModules/" + file));

    //     // typecasting obj to JSONObject
    //     JSONObject jo = (JSONObject) obj;

    //     // getting firstName and lastName
    //     String moduleName = (String) jo.get("name");
    //     int driveMotorID = Math.toIntExact((Long) jo.get("driveMotorID"));
    //     int turnMotorID = Math.toIntExact((Long) jo.get("turnMotorID"));
    //     double p = (double) jo.get("P");
    //     double i = (double) jo.get("I");
    //     double d = (double) jo.get("D");
    //     double f = (double) jo.get("F");
    //     int startEnc = Math.toIntExact((Long) jo.get("startingEncoderClick"));
    //     int cv = Math.toIntExact((Long) jo.get("cruiseVelocity"));
    //     int ca = Math.toIntExact((Long) jo.get("cruiseAccel"));
    //     boolean tcd = (boolean) jo.get("turnCountsDecreasing");
    //     boolean driveInvert = (boolean) jo.get("DriveInverted");
    //     boolean driveEncInvert = (boolean) jo.get("DriveEncoderInverted");
    //     boolean turnMotorInvert = (boolean) jo.get("TurnMotorInverted");
    //     boolean turnEncInvert = (boolean) jo.get("TurnEncoderInverted");

    //     return new SwerveModule(moduleName, driveMotorID, turnMotorID, p, i, d, f, startEnc, cv, ca, tcd, driveInvert,
    //             driveEncInvert, turnMotorInvert, turnEncInvert);
    // }

    public static String readRobotName() {
        final String dir = "/home/lvuser/robotInfo.txt";
        String robotName;
        try (BufferedReader in = new BufferedReader(new FileReader(dir))) {
            robotName = in.readLine();
        } catch (Exception e) {
            System.err.println("Unable to fetch Robot name automatically.");
            robotName = "";
        }

        return robotName;

    }

    public static RobotState.Bot parseRobotNameToEnum(final String robotName) {
        return RobotState.Bot.valueOf(robotName);
    }

    public static DMatrixRMaj setPseudoInverseForwardKinematicsMatrix() {
        double r1x = Robot.bot.kWheelbaseWidth / 2;
        double r1y = Robot.bot.kWheelbaseLength / 2;
        double r2x = -r1x;
        double r3y = -r1y;
        double[][] wheelPositions = {{1, 0, -r1y}, {0, 1, r1x}, {1, 0, -r1y}, {0, 1, r2x}, {1, 0, -r3y},
                {0, 1, r2x}, {1, 0, -r3y}, {0, 1, r1x}};

        DMatrixRMaj wheelPositionMatrix = new DMatrixRMaj(wheelPositions);
        DMatrixRMaj output = new DMatrixRMaj(wheelPositions);
        CommonOps_DDRM.pinv(wheelPositionMatrix, output);
        // Util.invertedWheelPositionMatrix = output;
        return output;
    }

    // public static Translation2d getVelocity(SimpleMatrix wheelVelocities) {
    //     SimpleMatrix simpleMatrix = invertedWheelPositionMatrix.createLike();
    //     SimpleMatrix cardinalVelocities = simpleMatrix.mult(wheelVelocities);
    //     return new Translation2d(cardinalVelocities.get(0, 0), cardinalVelocities.get(1, 0));
    // }

    // converts angle from 0 forwards and clockwise to 0 to the right and counter
    // clockwise
    public static double toUnitCircle(double angle) {
        return 90 - angle;
    }

    /**
     * @param cSystem     coordinate system to convert to
     * @param translation translation
     * @param theta       input theta
     * @return converted coordinates (x,y,theta)
     */
    public static double[] convertCoordinateSystem(CoordinateSystem cSystem, Translation2d translation, double theta) {
        double x = translation.getX();
        double y = translation.getY();
        return convertCoordinateSystem(cSystem, x, y, theta);
    }

    /**
     * @param cSystem coordinate system to convert to
     * @param x       input x
     * @param y       input y
     * @param theta   input theta
     * @return converted coordinates (x,y,theta)
     */
    public static double[] convertCoordinateSystem(CoordinateSystem cSystem, double x, double y, double theta) {
        double[] converted = new double[3];
        if (cSystem.equals(CoordinateSystem.UNIT_CIRCLE)) {
            // Converts rotated to unit circle
            converted[0] = y;
            converted[1] = -x;
            converted[2] = 90.0 + theta;
        } else if (cSystem.equals(CoordinateSystem.ROTATED)) {
            // Converts unit circle to rotated
            converted[0] = -y;
            converted[1] = x;
            converted[2] = theta - 90.0;
        }

        return converted;
    }

    /**
     * https://stackoverflow.com/a/1167047/6627273
     * A point D is considered "within" an angle ABC when
     * cos(DBM) > cos(ABM)
     * where M is the midpoint of AC, so ABM is half the angle ABC.
     * The cosine of an angle can be computed as the dot product of two normalized
     * vectors in the directions of its sides.
     * Note that this definition of "within" does not include points that lie on
     * the sides of the given angle.
     * If `vertical` is true, then check not within the given angle, but within the
     * image of that angle rotated by pi about its vertex.
     *
     * @param test     The translation object to test.
     * @param A        A point on one side of the angle.
     * @param B        The vertex of the angle.
     * @param C        A point on the other side of the angle.
     * @param vertical Whether to check in the angle vertical to the one given
     * @return Whether this translation is within the given angle.
     * @author Joshua Huang
     */
    public static boolean vectorIsWithinAngle(Translation2d test, Translation2d A, Translation2d B, Translation2d C, boolean vertical) {
        Translation2d M = vectorInterpolate(A, C, 0.5); // midpoint
        Translation2d m = normalize((new Translation2d(M.getX() - B.getX(), M.getY() - B.getY()))); // mid-vector
        Translation2d a = normalize((new Translation2d(A.getX() - B.getX(), A.getY() - B.getY()))); // side vector
        Translation2d d = normalize((new Translation2d(test.getX() - B.getX(), test.getY() - B.getY()))); // vector to here
        if (vertical) {
            m = vectorInverse(m);
            a = vectorInverse(a);
        }
        return vectorDotProduct(d, m) > vectorDotProduct(a, m);
    }

    public static boolean vectorIsWithinAngle(Translation2d test, Translation2d A, Translation2d B, Translation2d C) {
        return vectorIsWithinAngle(test, A, B, C, false);
    }

    /**
     * Assumes an angle centered at the origin.
     */
    public static boolean vectorIsWithinAngle(Translation2d test, Translation2d A, Translation2d C, boolean vertical) {
        return vectorIsWithinAngle(test, A, new Translation2d(), C, vertical);
    }

    public static boolean vectorIsWithinAngle(Translation2d test, Translation2d A, Translation2d C) {
        return vectorIsWithinAngle(test, A, C, false);
    }

    /**
     * The inverse simply means a Translation2d that "undoes" this object.
     *
     * @param vector Vector to take the inverse of (subtracted value)
     * @return Translation by -x and -y.
     */
    public static Translation2d vectorInverse(Translation2d vector) {
        return new Translation2d(-vector.getX(), -vector.getY());
    }

    public static Translation2d vectorInterpolate(Translation2d test, final Translation2d other, double x) {
        if (x <= 0) {
            return test;
        } else if (x >= 1) {
            return other;
        }
        return vectorExtrapolate(test, other, x);
    }

    public static Translation2d vectorExtrapolate(Translation2d test, final Translation2d other, double x) {
        return new Translation2d(x * (other.getX() - test.getX()) + test.getX(), x * (other.getY() - test.getY()) + test.getY());
    }

    public static double vectorDotProduct(final Translation2d a, final Translation2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }

    public enum CoordinateSystem {
        UNIT_CIRCLE, ROTATED
    }
}

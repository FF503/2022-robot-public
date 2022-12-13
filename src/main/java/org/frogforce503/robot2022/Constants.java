package org.frogforce503.robot2022;

import org.frogforce503.robot2022.subsystems.Hood;
import org.frogforce503.robot2022.subsystems.Shooter;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Constants {

    public enum ShotPresets {
        BATTER(Robot.bot.kShooterBatterRPM, Robot.bot.kBatterHoodAngle, 2.0),
        TARMAC_LINE(Shooter.getInstance().getShooterSpeedForDistance(111.0),
                Hood.getInstance().getHoodSetpointForDistance(111.0), 90.0),
        WALL(2500.0, 102.0, 145.0),
        LAUNCHPAD(2700, 144, 16.2);

        double shooterRPM, hoodSetpoint, turretAngle;

        ShotPresets(double shooterRPM, double hoodSetpoint, double turretAngle) {
            this.shooterRPM = shooterRPM;
            this.hoodSetpoint = hoodSetpoint;
            this.turretAngle = turretAngle;
        }

        ShotPresets(double shooterRPM, double hoodSetpoint, double turretAngle, boolean turretOverride) {
            this.shooterRPM = shooterRPM;
            this.hoodSetpoint = hoodSetpoint;
            this.turretAngle = turretAngle;
        }

        public double getShooterRPM() {
            return shooterRPM;
        }

        public double getHoodSetpoint() {
            return hoodSetpoint;
        }

        public double getTurretAngle() {
            return turretAngle;
        }
    }

    // Telemetry referenced in spindexer and shooter
    public static final boolean kOutputTelemetry = true;

    public static final double HEIGHT_OF_HUB = 102.0; // Old Value: 98.25 - 8.625 (Current value is in inches)
    public static final double DEPTH_OF_HUB = 107.0; // Old Value: 29.25 (Current value is in inches)

    // Gamespec

    public static final PneumaticsModuleType PCMType = PneumaticsModuleType.CTREPCM;

    // Shooter constants
    public static final double falconClicksperRevolution = 2048;
    public static final double kShooterManualPower = 0.5;
    public static final double kMaxShootingDist = 300.0;
    public static final double kMinShootingDist = 0.0;
    public static final int kShooterCurrentLimit = 75;
    public static final int kShooterStopOnTargetRpm = 150;
    public static final int kShooterStartOnTargetRpm = 50;
    public static final int kShooterMinOnTargetSamples = 20;

    // Intake constants
    public static final int kIntakeCurrentLimit = 60;
    public static final double kConveyorPower = 0.85;
    public static final double kIntakingPower = 0.63;
    public static final double kIntakeRatio = 1000.0 / 4;

    // Conveyor Impacts
    public static final int kConveryorCurrentLimit = 60;

    // Climber constants

    // Neutral/resting pitch where climber hangs without swinging
    public static final double kNeutralPitch = -10.0;
}
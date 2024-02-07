from phoenix6.signals import InvertedValue
from phoenix6.signals import NeutralModeValue
from phoenix6.signals import SensorDirectionValue
from wpimath.geometry import Rotation2d
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfile
import lib.mathlib.units as Units
from lib.util.COTSTalonFXSwerveConstants import COTSTalonFXSwerveConstants
from lib.util.SwerveModuleConstants import SwerveModuleConstants
import math

class Constants:
    stickDeadband = 0.1

    class Swerve:
        navxID = 0

        chosenModule = COTSTalonFXSwerveConstants.MK4i.Falcon500(COTSTalonFXSwerveConstants.MK4i.driveRatios.L2)

        # Drivetrain Constants
        trackWidth = Units.inchesToMeters(29.0)
        wheelBase = Units.inchesToMeters(29.0)
        wheelCircumference = chosenModule.wheelCircumference

        # Swerve Kinematics 
        swerveKinematics = SwerveDrive4Kinematics(
            Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        )

        # Module Gear Ratios
        driveGearRatio = chosenModule.driveGearRatio
        angleGearRatio = chosenModule.angleGearRatio

        # Motor Inverts
        angleMotorInvert = chosenModule.angleMotorInvert
        driveMotorInvert = chosenModule.driveMotorInvert

        # Angle Encoder Invert
        cancoderInvert = chosenModule.cancoderInvert

        # Swerve Current Limiting
        angleCurrentLimit = 25
        angleCurrentThreshold = 40
        angleCurrentThresholdTime = 0.1
        angleEnableCurrentLimit = True

        driveCurrentLimit = 35
        driveCurrentThreshold = 60
        driveCurrentThresholdTime = 0.1
        driveEnableCurrentLimit = True

        openLoopRamp = 0.25
        closedLoopRamp = 0.0

        # Angle Motor PID Values
        angleKP = chosenModule.angleKP
        angleKI = chosenModule.angleKI
        angleKD = chosenModule.angleKD

        # Drive Motor PID Values
        driveKP = 0.1
        driveKI = 0.0
        driveKD = 0.0
        driveKF = 0.0

        driveKS = 0.32
        driveKV = 1.51
        driveKA = 0.27

        # Swerve Profiling Values
        # Meters per Second
        maxSpeed = 5.0
        # Radians per Second
        maxAngularVelocity = 2.5 * math.pi

        # Neutral Modes
        angleNeutralMode = NeutralModeValue.BRAKE
        driveNeutralMode = NeutralModeValue.BRAKE

        # Module Specific Constants
        # Front Left Module - Module 0
        class Mod0:
            driveMotorID = 3
            angleMotorID = 1
            canCoderID = 2
            angleOffset = Rotation2d.fromDegrees(0.0)
            constants = SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)

        # Front Right Module - Module 1
        class Mod1:
            driveMotorID = 6
            angleMotorID = 4
            canCoderID = 5
            angleOffset = Rotation2d.fromDegrees(0.0)
            constants = SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)
        
        # Back Left Module - Module 2
        class Mod2:
            driveMotorID = 12
            angleMotorID = 10
            canCoderID = 11
            angleOffset = Rotation2d.fromDegrees(0.0)
            constants = SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)

        # Back Right Module - Module 3
        class Mod3:
            driveMotorID = 9
            angleMotorID = 7
            canCoderID = 8
            angleOffset = Rotation2d.fromDegrees(0.0)
            constants = SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)

    class AutoConstants:
        kMaxSpeedMetersPerSecond = 3
        kMaxAccelerationMetersPerSecondSquared = 3
        kMaxAngularSpeedRadiansPerSecond = math.pi
        kMaxAngularSpeedRadiansPerSecondSquared = math.pi
        kPXController = 4.0
        kPYController = 4.0
        kPThetaController = 1.5
    
        kThetaControllerConstraints = TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, 
            kMaxAngularSpeedRadiansPerSecondSquared
        )
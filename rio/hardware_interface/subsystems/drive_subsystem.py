from commands2 import Subsystem
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Odometry, SwerveModuleState
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from hardware_interface.drivetrain import DriveTrain
import math

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
from wpilib import DriverStation

class DriveSubsystem(Subsystem):
    def __init__(self, drivetrain: DriveTrain):
        super().__init__()
        self.drivetrain = drivetrain

        self.commanded_vel = ChassisSpeeds(0, 0, 0)
        
        self.odometer = SwerveDrive4Odometry(
            self.drivetrain.kinematics, 
            Rotation2d(0),
            (
                self.drivetrain.front_left.getPosition(),
                self.drivetrain.front_right.getPosition(),
                self.drivetrain.rear_left.getPosition(),
                self.drivetrain.rear_right.getPosition()
            ),
            Pose2d(0, 0, Rotation2d(0))
        )

        self.max_module_speed = 4.5 # m/s

        AutoBuilder.configureHolonomic(
            self.getPose, # Robot pose supplier
            self.resetOdometry,
            self.getRobotRelativeChassisSpeeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.driveRobotRelativePathPlanner, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            HolonomicPathFollowerConfig( # HolonomicPathFollowerConfig, this should likely live in your Constants class
                PIDConstants(4.0, 0.0, 0.0), # Translation PID constants
                PIDConstants(5.0, 0.0, 0.0), # Rotation PID constants
                self.max_module_speed, # Max module speed, in m/s
                0.3683, # Drive base radius in meters. Distance from robot center to furthest module.
                ReplanningConfig() # Default path replanning config. See the API for the options here
            ),
            self.pathFlip, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )

        self.robot_center = Translation2d(0, 0)

    def pathFlip(self):
        return False
        # return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def swerve_drive(self, x, y, z, field_oriented):
        self.updateOdometry()
        if field_oriented:
            self.drivetrain.swerveDriveAutonFieldOriented(x, y, z)
        else:
            self.drivetrain.swerveDriveAuton(x, y, z)

    def driveRobotRelativePathPlanner(self, speeds: ChassisSpeeds):
        self.commanded_vel = speeds
        self.drivetrain.swerveDrivePath(speeds.vx, speeds.vy, speeds.omega, self.max_module_speed)

    def getCommandedVel(self):
        return self.commanded_vel
            
    def setModuleStates(self, states: list[SwerveModuleState]):
        self.updateOdometry()
        self.drivetrain.front_left.set(states[0])
        self.drivetrain.front_right.set(states[1])
        self.drivetrain.rear_left.set(states[2])
        self.drivetrain.rear_right.set(states[3])
            
    def getPose(self):
        pos = self.odometer.getPose()
        return Pose2d(
            pos.X(),
            -pos.Y(),
            pos.rotation()
        )
    
    def getRobotRelativeChassisSpeeds(self):
        return self.drivetrain.kinematics.toChassisSpeeds(
            (
                self.drivetrain.front_left.getState(),
                self.drivetrain.front_right.getState(),
                self.drivetrain.rear_left.getState(),
                self.drivetrain.rear_right.getState()
            )
        )
    
    def resetOdometry(self, pose):
        self.odometer.resetPosition(
            -self.drivetrain.navx.getRotation2d(),
            (
                self.drivetrain.front_left.getPosition(),
                self.drivetrain.front_right.getPosition(),
                self.drivetrain.rear_left.getPosition(),
                self.drivetrain.rear_right.getPosition()
            ),
            pose
        )
            
    def updateOdometry(self):
        self.odometer.update(
            self.drivetrain.navx.getRotation2d().__mul__(-1),
            (
                self.drivetrain.front_left.getPosition(),
                self.drivetrain.front_right.getPosition(),
                self.drivetrain.rear_left.getPosition(),
                self.drivetrain.rear_right.getPosition()
            )
        )

    def getWheelEncoderPositions(self):
        return [
            self.drivetrain.front_left.getEncoderData()[0]["position"],
            self.drivetrain.front_right.getEncoderData()[0]["position"],
            self.drivetrain.rear_left.getEncoderData()[0]["position"],
            self.drivetrain.rear_right.getEncoderData()[0]["position"]
        ]
        
    def getWheelEncoderVelocities(self):
        return [
            self.drivetrain.front_left.getEncoderData()[0]["velocity"],
            self.drivetrain.front_right.getEncoderData()[0]["velocity"],
            self.drivetrain.rear_left.getEncoderData()[0]["velocity"],
            self.drivetrain.rear_right.getEncoderData()[0]["velocity"]
        ]
        
    def metersToShaftTicks(self, meters):
        return self.drivetrain.metersToShaftTicks(meters)
    
    def shaftTicksToMeters(self, ticks):
        return self.drivetrain.shaftTicksToMeters(ticks)
            
    def getKinematics(self):
        return self.drivetrain.kinematics
    
    def resetGyro(self):
        if self.drivetrain.is_sim:
            self.drivetrain.navx_sim.zeroYaw()
        self.drivetrain.navx.zeroYaw()
        
    def hardResetGyro(self):
        if self.drivetrain.is_sim:
            self.drivetrain.navx_sim.zeroYaw()
        self.drivetrain.navx.reset()
        
    def recalibrateGyro(self):
        self.drivetrain.navx.calibrate()
        
    def getEncoderData(self):
        return self.drivetrain.getEncoderData()
    
    def getGyroAngle180(self):
        if self.drivetrain.is_sim:
            return self.drivetrain.navx_sim.getYawDegrees()
        return self.drivetrain.navx.getYaw()
    
    def getGyroRoll180(self):
        if self.drivetrain.is_sim:
            return self.drivetrain.navx_sim.getRollDegrees()
        return self.drivetrain.navx.getRoll()
    
    def getGyroPitch180(self):
        if self.drivetrain.is_sim:
            return self.drivetrain.navx_sim.getPitchDegrees()
        return self.drivetrain.navx.getPitch()
    
    def getVelocity(self):
        return self.drivetrain.speeds
    
    def lockDrive(self):
        self.drivetrain.lockDrive()
        
    def unlockDrive(self):
        self.drivetrain.unlockDrive()
        
    def stop(self):
        self.drivetrain.stop()
        
from robot.SwerveModule import SwerveModule
from robot.constants import Constants

from wpimath.kinematics import ChassisSpeeds
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.kinematics import SwerveDrive4Odometry
from wpimath.kinematics import SwerveModulePosition

from navx import AHRS

from wpimath.geometry import Pose2d
from wpimath.geometry import Rotation2d
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveModuleState
from wpilib import SmartDashboard
from commands2.subsystem import Subsystem

class Swerve(Subsystem):
    swerveOdometry: SwerveDrive4Odometry
    mSwerveMods: list[SwerveModule, SwerveModule, SwerveModule, SwerveModule]
    gyro: AHRS

    def __init__(self):
        self.gyro = AHRS.create_spi()
        self.gyro.calibrate()
        self.gyro.zeroYaw()

        self.mSwerveMods = [
            SwerveModule(0, Constants.Swerve.Mod0.constants),
            SwerveModule(1, Constants.Swerve.Mod1.constants),
            SwerveModule(2, Constants.Swerve.Mod2.constants),
            SwerveModule(3, Constants.Swerve.Mod3.constants)
        ]

        self.swerveOdometry = SwerveDrive4Odometry(Constants.Swerve.swerveKinematics, self.getGyroYaw(), self.getModulePositions())

    def drive(self, translation: Translation2d, rotation: Rotation2d, fieldRelative, isOpenLoop):
        swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.X(), 
                translation.Y(), 
                rotation, 
                self.getHeading()
            )
        ) if fieldRelative else Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            ChassisSpeeds(translation.getX(), translation.getY(), rotation)
        )
        SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed)

        for mod in self.mSwerveMods:
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop)    

    # Used by SwerveControllerCommand in Auto
    def setModuleStates(self, desiredStates):
        SwerveDrive4Kinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed)
        
        for mod in self.mSwerveMods:
            mod.setDesiredState(desiredStates[mod.moduleNumber], False)

    def getModuleStates(self):
        states = [SwerveModuleState]*4
        for mod in self.mSwerveMods:
            states[mod.moduleNumber] = mod.getState()
        return states

    def getModulePositions(self):
        positions = [SwerveModulePosition]*4
        for mod in self.mSwerveMods:
            positions[mod.moduleNumber] = mod.getPosition()
        return positions

    def getPose(self):
        return self.swerveOdometry.getPose()

    def setPose(self, pose):
        self.swerveOdometry.resetPosition(self.getGyroYaw(), tuple(self.getModulePositions()), pose)

    def getHeading(self):
        return self.getPose().rotation()

    def setHeading(self, heading):
        self.swerveOdometry.resetPosition(self.getGyroYaw(), tuple(self.getModulePositions()), Pose2d(self.getPose().translation(), heading))

    def zeroHeading(self):
        self.swerveOdometry.resetPosition(self.getGyroYaw(), tuple(self.getModulePositions()), Pose2d(self.getPose().translation(), Rotation2d()))

    def getGyroYaw(self):
        return Rotation2d.fromDegrees(self.gyro.getYaw())

    def resetModulesToAbsolute(self):
        for mod in self.mSwerveMods:
            mod.resetToAbsolute()

    def periodic(self):
        self.swerveOdometry.update(self.getGyroYaw(), tuple(self.getModulePositions()))

        for mod in self.mSwerveMods:
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().degrees())
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.degrees())
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speed)
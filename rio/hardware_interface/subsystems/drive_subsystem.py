from commands2 import Subsystem
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Odometry, SwerveModuleState
from wpimath.geometry import Rotation2d, Pose2d
from hardware_interface.drivetrain import DriveTrain

class DriveSubsystem(Subsystem):
    def __init__(self, drivetrain: DriveTrain):
        super().__init__()
        self.drivetrain = drivetrain
        
        self.odometer = SwerveDrive4Odometry(
            self.drivetrain.kinematics, 
            Rotation2d(0),
            (
                self.drivetrain.front_left.getState(),
                self.drivetrain.front_right.getState(),
                self.drivetrain.rear_left.getState(),
                self.drivetrain.rear_right.getState()
            ),
            Pose2d(0, 0, Rotation2d(0))
        )
        
    def swerve_drive(self, x, y, z, field_oriented):
        self.odometer.update(
            self.drivetrain.navx.getRotation2d(),
            self.drivetrain.front_left.getState(),
            self.drivetrain.front_right.getState(),
            self.drivetrain.rear_left.getState(),
            self.drivetrain.rear_right.getState()
        )
        if field_oriented:
            self.drivetrain.swerveDriveAutonFieldOriented(x, y, z)
        else:
            self.drivetrain.swerveDriveAuton(x, y, z)
            
    def setModuleStates(self, states: list[SwerveModuleState]):
        self.odometer.update(
            self.drivetrain.navx.getRotation2d(),
            self.drivetrain.front_left.getState(),
            self.drivetrain.front_right.getState(),
            self.drivetrain.rear_left.getState(),
            self.drivetrain.rear_right.getState()
        )
        self.drivetrain.front_left.set(states[0])
        self.drivetrain.front_right.set(states[1])
        self.drivetrain.rear_left.set(states[2])
        self.drivetrain.rear_right.set(states[3])
            
    def getPose(self):
        return self.odometer.getPose()
    
    def resetOdometry(self, pose):
        self.odometer.resetPosition(
            self.drivetrain.navx.getRotation2d(),
            pose,
            self.drivetrain.front_left.getState(),
            self.drivetrain.front_right.getState(),
            self.drivetrain.rear_left.getState(),
            self.drivetrain.rear_right.getState()
        )
            
    def updateOdometry(self):
        self.odometer.update(
            self.drivetrain.navx.getRotation2d(),
            self.drivetrain.front_left.getState(),
            self.drivetrain.front_right.getState(),
            self.drivetrain.rear_left.getState(),
            self.drivetrain.rear_right.getState()
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
        
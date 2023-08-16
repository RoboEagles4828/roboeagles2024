from commands2 import SubsystemBase
from hardware_interface.drivetrain import DriveTrain

class DriveSubsystem(SubsystemBase):
    def __init__(self, drivetrain: DriveTrain):
        super().__init__()
        self.drivetrain = drivetrain
        
    def swerve_drive(self, x, y, z, field_oriented):
        if field_oriented:
            self.drivetrain.swerveDriveAutonFieldOriented(x, y, z)
        else:
            self.drivetrain.swerveDriveAuton(x, y, z)
    
    def resetGyro(self):
        if self.drivetrain.is_sim:
            self.drivetrain.navx_sim.zeroYaw()
        self.drivetrain.navx.zeroYaw()
        
    def getEncoderData(self):
        return self.drivetrain.getEncoderData()
    
    def getGyroAngle180(self):
        if self.drivetrain.is_sim:
            return self.drivetrain.navx_sim.getYawDegrees()
        return self.drivetrain.navx.getYaw()
    
    def getGyroPitch180(self):
        if self.drivetrain.is_sim:
            return self.drivetrain.navx_sim.getPitchDegrees()
        return self.drivetrain.navx.getPitch()
    
    def lockDrive(self):
        self.drivetrain.lockDrive()
        
    def unlockDrive(self):
        self.drivetrain.unlockDrive()
        
    def stop(self):
        self.drivetrain.stop()
        
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState
from wpimath.geometry import Translation2d
from inverse_kinematics.motion_magic_control import MotionMagic
import math

class InverseKinematics():
    def __init__(self):
        self.speeds = ChassisSpeeds()
        self.module_config = {
            "front_left": {
                "wheel_joint_name": "front_left_wheel_joint",
                "wheel_motor_port": 3, #9
                "axle_joint_name": "front_left_axle_joint",
                "axle_motor_port": 1, #7
                "axle_encoder_port": 2, #8
                "encoder_offset": 18.721, # 248.203,
                "location" : Translation2d(-0.52085486, 0.52085486) # Translation2d(0.52085486, 0.52085486)
            },
            "front_right": {
                "wheel_joint_name": "front_right_wheel_joint",
                "wheel_motor_port": 6, #12
                "axle_joint_name": "front_right_axle_joint",
                "axle_motor_port": 4, #10
                "axle_encoder_port": 5, #11
                "encoder_offset": 45.439 + 180.0, #15.908, TODO: REDO ENCODER OFFSET
                "location" : Translation2d(-0.52085486, -0.52085486)# Translation2d(-0.52085486, 0.52085486)
            },
            "rear_left": {
                "wheel_joint_name": "rear_left_wheel_joint",
                "wheel_motor_port": 12, #6
                "axle_joint_name": "rear_left_axle_joint",
                "axle_motor_port": 10, #4
                "axle_encoder_port": 11, #5
                "encoder_offset": 16.084 + 180.0, #327.393, TODO: REDO ENCODER OFFSET
                "location" : Translation2d(0.52085486, 0.52085486) #Translation2d(0.52085486, -0.52085486)
            },
            "rear_right": {
                "wheel_joint_name": "rear_right_wheel_joint",
                "wheel_motor_port": 9, #3
                "axle_joint_name": "rear_right_axle_joint",
                "axle_motor_port": 7, #1
                "axle_encoder_port": 8, #2
                "encoder_offset": -9.141, #201.094,
                "location" : Translation2d(0.52085486, -0.52085486) # Translation2d(-0.52085486, -0.52085486)
            }
        }
        self.front_left_location = self.module_config["front_left"]["location"]
        self.front_right_location = self.module_config["front_right"]["location"]
        self.rear_left_location = self.module_config["rear_left"]["location"]
        self.rear_right_location = self.module_config["rear_right"]["location"]
        self.ROBOT_MAX_TRANSLATIONAL = 5.0
        self.kinematics = SwerveDrive4Kinematics(self.front_left_location, self.front_right_location, self.rear_left_location, self.rear_right_location)
        
        self.positionCoefficient = 2.0 * math.pi / 2048.0
        self.velocityCoefficient = self.positionCoefficient * 10.0
        
        self.MAX_VEL = 18000
        self.MAX_ACCEL = 14000
        
        self.motion_magic = MotionMagic(self.ticksToRadians(self.MAX_ACCEL), self.ticksToRadians(self.MAX_VEL))
    
    def metersToRadians(self, meters):
        wheel_rad = 0.0508
        return meters/wheel_rad
    
    def ticksToRadians(self, ticks, displacementType):
        if displacementType == "position":
            return ticks * self.positionCoefficient
        elif displacementType == "velocity":
            return ticks * self.velocityCoefficient
        else:
            return 0
    
    def getDriveJointStates(self, x, y, z, module_angles: list[float, float, float, float]):
        self.speeds = ChassisSpeeds(x, y, z)
        module_states = self.kinematics.toSwerveModuleStates(self.speeds)      
        self.kinematics.desaturateWheelSpeeds(module_states, self.ROBOT_MAX_TRANSLATIONAL)
        
        front_left_state = module_states[0]
        front_right_state = module_states[1]
        rear_left_state = module_states[2]
        rear_right_state = module_states[3]
        
        front_left_state = SwerveModuleState.optimize(front_left_state, module_angles[0])
        front_right_state = SwerveModuleState.optimize(front_right_state, module_angles[1])
        rear_left_state = SwerveModuleState.optimize(rear_left_state, module_angles[2])
        rear_right_state = SwerveModuleState.optimize(rear_right_state, module_angles[3])
        
        return [self.metersToRadians(front_left_state.speed), self.metersToRadians(front_right_state.speed), self.metersToRadians(rear_left_state.speed), self.metersToRadians(rear_right_state.speed), front_left_state.angle.radians(), front_right_state.angle.radians(), rear_left_state.angle.radians(), rear_right_state.angle.radians()]
    
    def getArmJointStates(self, names: list, target_positions: list, current_positions: list):
        vel = [0.0]*len(names)
        for j, i in enumerate(names):
            target_position = target_positions[j]
            current_position = current_positions[j]
            output_velocity = self.motion_magic.getNextVelocity(target_position, current_position)
            vel[j] = output_velocity
            
        return vel
            
            
            
            
        
        
        
        
from enum import Enum

class CommandType(Enum):
    '''
    Defines the commands recognized by the comms node that are received from the
    base station.
    '''
    DRIVE_TRAIN = 'DRIVE_TRAIN'
    ARM = 'ARM'

class CommandParser():
    '''
    This class is responsible for parsing JSON objects for supported commands. 
    Ensures command parsing logic is not expressed in connection_server for e.g., 
    as Flask's main responsibility is to relay the command.
    '''
 
    @staticmethod
    def parse_drive_train_command(parameters_json):
        parameters = {
            'left_speed': int(parameters_json['left_speed']),
            'right_speed': int(parameters_json['right_speed']),
            'left_direction': int(parameters_json['left_direction']),
            'right_direction': int(parameters_json['right_direction'])
        }
        return parameters

    @staticmethod
    def parse_arm_command(parameters_json):
        parameters = {
            'x_axis_velocity': float(parameters_json['x_axis_velocity']),
            'y_axis_velocity': float(parameters_json['y_axis_velocity']),
            'z_axis_velocity': float(parameters_json['z_axis_velocity']),
            'gripper_velocity': float(parameters_json['gripper_velocity']),
            'gripper_rotation_velocity': float(parameters_json['gripper_rotation_velocity'])
        }
        return parameters

    @staticmethod
    def parse_command(command_type, parameters):
        '''
        Returns the parsed command based on the command_type provided and
        the parameters dict which determine argument values.
        '''

        assert type(command_type) == CommandType

        if command_type == CommandType.DRIVE_TRAIN:
            return CommandParser.parse_drive_train_command(parameters)
        elif command_type == CommandType.ARM:
            return CommandParser.parse_arm_command(parameters)

import subprocess

class WifiManager():
    def __init__(self):
        pass

    def connect_to_wifi_network(self, ssid, password):
        '''
        Attempts to connect to the WiFi network specified by the SSID and password parameters.

        Parameters:
            ssid (str): The network SSID
            password (str): The network's password
        Returns:
            (bool): True if the connection has been established successfully, False otherwise
        Raises:
            subprocess.CalledProcessError: Raised when process returns a non-zero exit status.
        '''
        assert type(ssid) == str
        assert type(password) == str
        
        try:
            command_to_execute = 'nmcli --wait 20 device wifi connect {} password "{}"'.format(ssid, password)
            subprocess.check_output(command_to_execute, shell=True)
        except subprocess.CalledProcessError as err:
            print('Error: connection to WiFi network failed. Return code: {}, Output: {}'.format(err.returncode, err.output))
            return False
        return True

    def disconnect_from_wifi_network(self, ssid):
        '''
        Disconnects from the WiFi network specified by the SSID.

        Parameters:
            ssid (str): The network SSID
        Returns:
            (bool): True if the connection has been deactivated successfully, False otherwise
        Raises:
            subprocess.CalledProcessError: Raised when process returns a non-zero exit status.
        '''
        assert type(ssid) == str

        try:
            command_to_execute = 'nmcli connection down {}'.format(ssid)
            subprocess.check_output(command_to_execute, shell=True)
        except subprocess.CalledProcessError as err:
            print('Error: failed to disconnect from network. Return code: {}, Output: {}'.format(err.returncode, err.output))
            return False
        return True

    def list_current_active_connections(self):
        '''
        Lists all current active connections by calling the 'nmcli' tool. Information returned for each active connection:
            * NAME: Connection name
            * UUID: Connection UUID
            * TYPE: Type of connection (e.g. wifi, ethernet)
            * DEVICE: Network device used for connection

        Paramters:
            None
        Returns:
            current_active_connections_list (list of dictionaries): List of dictionaries, where each list item represents a single
                active connection.
        '''

        unparsed_output_by_line = str(subprocess.check_output('nmcli connection show --active', shell=True))[2:].split('\\n')[:-1]
        headers = unparsed_output_by_line[0]
        headers_keys = headers.split()

        # Assert header keys are expected output, and get key indexes to allow for correct parsing
        self.__assert_expected_keys_list(headers_keys, 4, ['NAME', 'UUID', 'TYPE', 'DEVICE'])
        headers_key_indexes = self.__get_header_key_indexes(headers, ['NAME', 'UUID', 'TYPE', 'DEVICE'])

        current_active_connections_list = []
        for line in unparsed_output_by_line[1:]:
            # Append each connection detail. Each value is parsed by getting substring corresponding to key indexes as delimiters
            current_active_connections_list.append({
                'NAME': line[headers_key_indexes['NAME']: headers_key_indexes['UUID']].strip(),
                'UUID': line[headers_key_indexes['UUID']: headers_key_indexes['TYPE']].strip(),
                'TYPE': line[headers_key_indexes['TYPE']: headers_key_indexes['DEVICE']].strip(),
                'DEVICE': line[headers_key_indexes['DEVICE']:].strip()
            })
        return current_active_connections_list

    def list_available_wifi_networks(self):
        '''
        Lists available WiFi networks by calling the 'nmcli' tool. Information returned for each detected WiFi network:
            * IN-USE: Is the host currently connected to that network
            * BSSID: Basic Service Set Identifier (access point)
            * SSID: Service Set Identifier. Name of network
            * MODE, CHAN, RATE, SIGNAL
        NOTE: SECURITY column is not returned as parsing was too cumbersome due to BARS characters preceding it, and do not need
              that information. 

        Parameters:
            None
        Returns:
            available_wifi_list (list of dictionaries): List of dictionaries, where each list item represents a single
                detected WiFi network. If no WiFi networks found, returns empty list.
        Raises:
            subprocess.CalledProcessError: Raised when process returns a non-zero exit status.
        '''

        try:
            # First slice [2:] removes "b'" left over from bytes to string conversion, [:-1] removes last line that is empty character
            unparsed_output_by_line = str(subprocess.check_output('nmcli device wifi list', shell=True))[2:].split('\\n')[:-1]
        except subprocess.CalledProcessError as err:
            print('Error: failed to list available wifi networks. Return code: {}, Output: {}'.format(err.returncode, err.output))
            raise err

        # If no networks provided, return empty list
        if len(unparsed_output_by_line) == 0:
            print('Warning: no available WiFi networks were found.')
            return []

        headers = unparsed_output_by_line[0]
        headers_keys = headers.split()
        
        # Assert header keys are expected output, and get key indexes to allow for correct parsing
        self.__assert_expected_keys_list(headers_keys, 9, ['IN-USE', 'BSSID', 'SSID', 'MODE', 'CHAN', 'RATE', 'SIGNAL', 'BARS', 'SECURITY'])
        # NOTE: ' SSID' is deliberately pre-pended by space since otherwise it would match BSSID
        headers_key_indexes = self.__get_header_key_indexes(headers, ['IN-USE', 'BSSID', ' SSID', 'MODE', 'CHAN', 'RATE', 'SIGNAL', 'BARS', 'SECURITY'])

        available_wifi_list = []
        for line in unparsed_output_by_line[1:]:
            # Append wifi network details. Each value is parsed by getting substring corresponding to key indexes as delimiters
            available_wifi_list.append({
                'IN-USE': True if line[0] == '*' else False,
                'BSSID': line[headers_key_indexes['BSSID']: headers_key_indexes[' SSID']].strip(),
                'SSID': line[headers_key_indexes[' SSID']: headers_key_indexes['MODE']].strip(),
                'MODE': line[headers_key_indexes['MODE']: headers_key_indexes['CHAN']].strip(),
                'CHAN': line[headers_key_indexes['CHAN']: headers_key_indexes['RATE']].strip(),
                'RATE': line[headers_key_indexes['RATE']: headers_key_indexes['SIGNAL']].strip(),
                'SIGNAL': line[headers_key_indexes['SIGNAL']: headers_key_indexes['BARS']].strip()
            })
        return available_wifi_list

    def list_network_interfaces_status(self):
        '''
        Returns a list of the device's network interfaces, along with their status.

        Parameters:
            None
        Returns:
            (list): List of dictionaries, each of which are a device's network interface.
        Raises:
            subprocess.CalledProcessError: Raised when process returns a non-zero exit status.
        '''

        try:
            unparsed_output_by_line = str(subprocess.check_output('nmcli device status', shell=True))[2:].split('\\n')[:-1]
        except subprocess.CalledProcessError as err:
            print('Error: failed to get network interfaces status. Return code: {}, Output: {}'.format(err.returncode, err.output))
            raise err

        headers = unparsed_output_by_line[0]
        headers_keys = headers.split()

        # Assert header keys are expected output, and get key indexes to allow for correct parsing
        self.__assert_expected_keys_list(headers_keys, 4, ['DEVICE', 'TYPE', 'STATE', 'CONNECTION'])
        headers_key_indexes = self.__get_header_key_indexes(headers, ['DEVICE', 'TYPE', 'STATE', 'CONNECTION'])

        network_interfaces_list = []
        for line in unparsed_output_by_line[1:]:
            # Append network interface details to list
            network_interfaces_list.append({
                'DEVICE': line[headers_key_indexes['DEVICE']: headers_key_indexes['TYPE']].strip(),
                'TYPE': line[headers_key_indexes['TYPE']: headers_key_indexes['STATE']].strip(),
                'STATE': line[headers_key_indexes['STATE']: headers_key_indexes['CONNECTION']].strip(),
                'CONNECTION': line[headers_key_indexes['CONNECTION']:].strip()
            })

        return network_interfaces_list

    def __assert_expected_keys_list(self, actual_keys, expected_length, expected_keys):
        '''
        Asserts that the list keys contains the expected strings at the correct location.

        Parameters:
            actual_keys (list): A list of keys to be analyzed
            expected_length (int): Expected length of actual_keys
            expected_keys (list): A list of keys expected to be in the same order as actual_keys
        Returns:
            None
        Raises:
            AssertionError: An expected string was not found at the expected index (message provides information),
                or the length of the provided list is not as expected.
        '''
        assert type(actual_keys) == list
        assert type(expected_length) == int
        assert type(expected_keys) == list

        assertion_error_message = '{} key not found at correct index'
        assert len(actual_keys) == expected_length, 'Unexpected keys list column length'

        for i, expected_key in enumerate(expected_keys):
            assert actual_keys[i] == expected_key, assertion_error_message.format(expected_key)

    def __get_header_key_indexes(self, headers, expected_strings_list):
        '''
        Returns a dictionary where keys are expected strings, and corresponding values are the index at which
        the expected string was found.

        Parameters:
            headers (str): A string
            expected_strings_list (list): List of expected strings
        Returns:
            indexes (dict of str -> int): Dictionary with keys expected string and corresponding values the index
                in the string where it is found.
        Raises:
            ValueError: If any of the expected strings are not found in headers.
        '''
        assert type(headers) == str
        assert type(expected_strings_list) == list

        indexes = {}
        for expected_string in expected_strings_list:
            indexes[expected_string] = headers.index(expected_string)
        return indexes

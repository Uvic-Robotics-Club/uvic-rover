import subprocess, os

class NetworkConnector():
    def __init__(self):
        print(self.list_available_wifi_networks())

    def list_available_wifi_networks(self):
        '''
        Lists available WiFi networks by calling the 'nmcli' tool. Information returned for each detected WiFi network:
            * IN-USE: Is the host currently connected to that network
            * BSSID: Basic Service Set Identifier (access point)
            * SSID: Service Set Identifier. Name of network
            * MODE: 

        Parameters:
            None
        Returns:
            available_wifi_list (list of dictionaries): List of dictionaries, where each list item represents a single
                detected WiFi network.
        '''
        # TODOs:
        #  - Handle if wi-fi not available
        #  - Include security column as it is more difficult to parse due to BARS column

        # First slice [2:] removes "b'" left over from bytes to string conversion, [:-1] removes last line that is empty character
        unparsed_output_by_line = str(subprocess.check_output('nmcli dev wifi list', shell=True))[2:].split('\\n')[:-1]
        headers = unparsed_output_by_line[0]
        headers_keys = headers.split()
        
        # Assert column keys are expected output, and get key indexes to allow for correct parsing
        self.__assert_nmcli_dev_wifi_list_keys(headers_keys)
        headers_key_indexes = self.__get_header_key_indexes(headers)

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

    def __assert_nmcli_dev_wifi_list_keys(self, keys):
        '''
        Asserts that the list keys contains the expected strings at the correct location.

        Parameters:
            keys (list): A list
        Returns:
            None
        Raises:
            AssertionError: An expected string was not found at the expected index (message provides information),
                or the length of the provided list is not as expected.
        '''
        assert type(keys) == list

        assert len(keys) == 9, 'Unexpected wifi list column length found'

        assertion_error_message = '{} columm not found at correct index'
        assert keys[0] == 'IN-USE', assertion_error_message.format('IN-USE')
        assert keys[1] == 'BSSID', assertion_error_message.format('BSSID')
        assert keys[2] == 'SSID', assertion_error_message.format('SSID')
        assert keys[3] == 'MODE', assertion_error_message.format('MODE')
        assert keys[4] == 'CHAN', assertion_error_message.format('CHAN')
        assert keys[5] == 'RATE', assertion_error_message.format('RATE')
        assert keys[6] == 'SIGNAL', assertion_error_message.format('SIGNAL')
        assert keys[7] == 'BARS', assertion_error_message.format('BARS')
        assert keys[8] == 'SECURITY', assertion_error_message.format('SECURITY')

    def __get_header_key_indexes(self, headers):
        '''
        Returns a dictionary where keys are expected strings, and corresponding values are the index at which
        the expected string was found.

        Parameters:
            headers (str): A string
        Returns:
            indexes (dict of str -> int): Dictionary with keys expected string and corresponding values the index
                in the string where it is found.
        Raises:
            ValueError: If any of the expected strings are not found in headers.
        '''
        assert type(headers) == str

        indexes = {}
        # NOTE: ' SSID' is deliberately pre-pended by space since otherwise it would match BSSID
        expected_key_values = ['IN-USE', 'BSSID', ' SSID', 'MODE', 'CHAN', 'RATE', 'SIGNAL', 'BARS', 'SECURITY']
        
        for expected_key_value in expected_key_values:
            indexes[expected_key_value] = headers.index(expected_key_value)
        return indexes

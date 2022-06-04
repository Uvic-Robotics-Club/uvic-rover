from rover.msg import Coordinates

class ReportHandler:
    """
    This interface declares a method for for building the chain of handlers for reports
    that must be processed into messages. It also declarres a method for executing a 
    request. This implements the Chain of Responsibility behavioral pattern.
    """

    def set_next(self, handler):
        raise NotImplementedError

    def handle(self, report, ros_msg=None):
        raise NotImplementedError


class AbstractReportHandler(ReportHandler):
    """
    This defines the default chaining behavior (if handle() is not implemented), as well
    as the logic to chain handlers together.
    """

    _next_handler = None

    def set_next(self, handler):
        self._next_handler = handler
        return handler

    def handle(self, report, ros_msg=None):
        if self._next_handler:
            return self._next_handler.handle(report, ros_msg)
        return ros_msg


# Concrete report handlers are defined below.

class TPVReportHandler(AbstractReportHandler):

    def handle(self, report, ros_msg=None):
        # Only process if this report is of type 'TPV'
        if report['class'] != 'TPV':
            return super().handle(report, ros_msg)
        
        if not ros_msg:
            ros_msg = Coordinates()
            
        if hasattr(report, 'mode'):
            ros_msg.mode = report.mode
        if hasattr(report, 'lat'):
            ros_msg.latitude = report.lat
        if hasattr(report, 'lon'):
            ros_msg.longitude = report.lon
        if hasattr(report, 'altMSL'):
            ros_msg.altitude_msl = report.altMSL

        return super().handle(report, ros_msg)


class DeviceReportHandler(AbstractReportHandler):

    def handle(self, report, ros_msg=None):
        # Only process if this report is of type 'DEVICE'
        if report['class'] != 'DEVICES':
            return super().handle(report, ros_msg)

        if not ros_msg:
            ros_msg = Coordinates()

        if hasattr(report, 'devices'):
            if len(report.devices) == 0:
                ros_msg.message = 'No connected devices are detected by the GPS daemon.'
            else:
                ros_msg.message = str(report.devices)
        else:
            ros_msg.message = 'Attempted to access device information from DEVICE report, no device attribute found.'

        return super().handle(report, ros_msg)
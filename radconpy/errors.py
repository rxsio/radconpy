class RadConNotConnectedException(Exception):
    """
    RadCon USB Device is not connected
    """
    ...

class RadConUnknownException(Exception):
    """
    Unknown exception with communication with RadCon device
    """
    ...
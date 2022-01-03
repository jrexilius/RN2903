# RN2903 - Basic Wrapper for Microchip RN2903 running factory firmware

# Description:
This class wraps the serial interface to the RN2903 chip and its default firmware. It normalizes  the command interface, handles basic formatting and error handling, and enforces some safeguards. It is intended to be used as a simple library for other scripts or basic reference implementation.

The usage model is a single-threaded, blocking interface.  A better, but more complex, async IO model could be implemented if needed, but the RN2903 chip itself behaves in a blocking, single-channel mode in general. The RN2903 chip is somewhat fragile and can get into a bad state with blind fidling.  For that reason, this class attempts to prevent input errors and enforce some degree of state-aware command management.  This "safe_mode" can be overridden at run-time  if needed.  This module does not provide a low level interface for flashing firmware, so the system command "eraseFW" should not be called without an alternate means to load firmware to the RN2903 chip. This class was designed around the most current official Microchip firmware version. An inital firmware version check is made at runtime and the interface will not operate in safe mode if the versions do not match.

The "radio" mode of operations is typically used for peer-to-peer applications and provides basic control over the radio operations.  It lacks much that the LoRaWAN stack provides such as basic encryption and private networking. To use the radio mode, the "mac" system must be paused. Settings for the radio system are NOT persisted between reboots, unlike mac settings.  For ease of radio operations, this class provides methods for saving and loading configuration settings to a local file.  The RN2903 chip provides 255 bytes of non-volatile storage on the chip.  It is  possible to store key radio settings in that NVM space, rather than in a file on the host but that has not yet been implemented.

The "mac" mode of operations (LoRaWAN) is not currently implemented as it appears that Microchips default firmware really only has class A operations fully implemented.  I'm not clear how to model class B or C operations given the commands listed in their documentation, as of yet. That is likely a failure of understanding on my part.


# ToDo:
 - improve device state tracking
 - enforce more command rules around state
 - implement LoRaWAN commands & management
 - possibly re-implement as async IO model


# Class Design
LoRaDevice(config_file)

"public" methods:
- (status, data) : send_command : (command_string) 
- (config) : get_device_config : ()
- (bool) : put_device_config : ()

"private" methods:
- (bool) : check_param : (paramter_value, parameter_specification)
- (bool) : validate_command : (command_string)
- (bool) : write_command : (command_string)
- (response) : read_response : ()
- (bool) : read_file_config : (filename)
- (bool) : save_file_config : (filename)
    

# Device Reference: 
https://www.microchip.com/wwwproducts/en/RN2903



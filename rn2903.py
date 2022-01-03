####################################################################################################
# 
# Name:  RN2903 Basic Wrapper
#
#
# Description
# -----------
# This class wraps the serial interface to the RN2903 chip and its default firmware. It normalizes 
# the command interface, handles basic formatting and error handling, and enforces some safeguards.
# It is intended to be used as a simple library for other scripts or basic reference implementation.
# 
# The usage model is a single-threaded, blocking interface.  A better, but more complex, async IO
# model could be implemented if needed, but the RN2903 chip itself behaves in a blocking,
# single-channel mode in general. The RN2903 chip is somewhat fragile and can get into a bad state
# with blind fidling.  For that reason, this class attempts to prevent input errors and enforce
# some degree of state-aware command management.  This "safe_mode" can be overridden at run-time 
# if needed.  This module does not provide a low level interface for flashing firmware, so the
# system command "eraseFW" should not be called without an alternate means to load firmware to the
# RN2903 chip. This class was designed around the most current official Microchip firmware version.
# An inital firmware version check is made at runtime and the interface will not operate in safe
# mode if the versions do not match.
#
# The "radio" mode of operations is typically used for peer-to-peer applications and provides basic
# control over the radio operations.  It lacks much that the LoRaWAN stack provides such as basic
# encryption and private networking. To use the radio mode, the "mac" system must be paused.
# Settings for the radio system are NOT persisted between reboots, unlike mac settings.  For ease
# of radio operations, this class provides methods for saving and loading configuration settings to
# a local file.  The RN2903 chip provides 255 bytes of non-volatile storage on the chip.  It is 
# possible to store key radio settings in that NVM space, rather than in a file on the host but
# that has not yet been implemented.
#
# The "mac" mode of operations (LoRaWAN) is not currently implemented as it appears that Microchips
# default firmware really only has class A operations fully implemented.  I'm not clear how to
# model class B or C operations given the commands listed in their documentation, as of yet. That
# is likely a failure of understanding on my part.
#
#
# TODO
# ----
# - improve device state tracking
# - enforce more command rules around state
# - implement LoRaWAN commands & management
# - possibly re-implement as async IO model
#
#
# Class Design
# ------------
# LoRaDevice(config_file)
#
#  "public" methods:
#    (status, data) : send_command : (command_string) 
#    (config) : get_device_config : ()
#    (bool) : put_device_config : ()
#
#  "private" methods:
#    (bool) : check_param : (paramter_value, parameter_specification)
#    (bool) : validate_command : (command_string)
#    (bool) : write_command : (command_string)
#    (response) : read_response : ()
#    (bool) : read_file_config : (filename)
#    (bool) : save_file_config : (filename)
#    
#
# Device Reference:  https://www.microchip.com/wwwproducts/en/RN2903
# 
#
# Original Author: Jason Rexilius, https://www.jasonrexilius.com/
#
#
# License: To the extent possible under law, Jason Rexilius has waived all copyright and related
#          or neighboring rights to RN2903 Basic Wrapper.
#          This work is published from: United States. 
#          This software is donated to the public domain.  Full license can be seen here:
#          http://creativecommons.org/publicdomain/zero/1.0/
#
#
# Warranty: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
#           INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
#           PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR 
#           ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
#           OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
#           DEALINGS IN THE SOFTWARE.
#
#
####################################################################################################
import sys
import serial
import logging
import re

import os
import os.path
import json


####################################################################################################
# RN2903 - class encapsulating the serial command interface to the LoRa chip
#

class rn2903():


  ##################################################################################################
  # device constants

  # chip specs
  firmware_name = 'RN2903'
  target_version = '1.0.5'

  # serial device settings
  ser_baud = 57600
  ser_bits = 8
  ser_parity = serial.PARITY_NONE
  ser_stopb = serial.STOPBITS_ONE
  ser_flowctl = 'none'

  # possible device conditions
  device_states = ['sleep',
                   'idle',
                   'wating_fw',
                   'booting',
                   'mac_active',
                   'mac_paused',
                   'radio_tx',
                   'radio_rx',
                   'mac_saving',
                   'mac_booting',
                   'error']

  # command/response terminator sequence
  data_termintaor = "\r\n"

  # command response codes
  response_codes = ['ok',
                    'busy',
                    'fram_counter_err_rejoin_needed',
                    'invalid_class',
                    'invalid_data_len',
                    'invalid_param',
                    'keys_not_init',
                    'mac_paused',
                    'multicast_keys_not_set',
                    'no_free_ch',
                    'not_joined',
                    'silent',
                    'err']

  # persistent memory address range
  nvm_adr_range = ['300','3FF']

  # digtial GPIO pins
  dpin_names = ['GPIO0',
                'GPIO1',
                'GPIO2',
                'GPIO3',
                'GPIO4',
                'GPIO5',
                'GPIO6',
                'GPIO7',
                'GPIO8',
                'GPIO9',
                'GPIO10',
                'GPIO11',
                'GPIO12',
                'GPIO13',
                'UART_CTS',
                'UART_RTS',
                'TEST0',
                'TEST1']

  # analog GPIO pins
  apin_names = ['GPIO0',
                'GPIO1',
                'GPIO2',
                'GPIO3',
                'GPIO5',
                'GPIO6',
                'GPIO7',
                'GPIO8',
                'GPIO9',
                'GPIO10',
                'GPIO11',
                'GPIO12',
                'GPIO13']

  # pin modes
  pin_modes = ['digout','digin','ana']

  # message size limit, 120 bytes
  max_tx_msg = 'FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF'

  # automatic frequency correction bands, in kHz
  auto_freq_band = ['250','125','62.5','31.3','15.6','7.8','3.9','200','100','50','25','12.5','6.3','3.1','166.7','83.3','41.7','20.8','10.4','5.2','2.6']

  # the Gaussian baseband data shaping, enabling GFSK modulation
  gfsk_modulation_bt = ['none','1.0','0.5','0.3']

  # the signal bandwidth, in kHz
  rx_bandwidth = ['250','125','62.5','31.3','15.6','7.8','3.9','200','100','50','25','12.5','6.3','3.1','166.7','83.3','41.7','20.8','10.4','5.2','2.6']

  # spreading factor
  spread_factors = ['sf7','sf8','sf9','sf10','sf11','sf12']

  # command types
  commands = {'sys':{},'mac':{},'radio':{}}

  # sys commands
  commands['sys'] = {'sleep':{},
                     'reset':{},
                     'eraseFW':{},
                     'factoryRESET':{},
                     'set':{},
                     'get':{}}
  commands['sys']['sleep']['user_input'] = [{'type':'int','range':[100,4294967296]}]
  commands['sys']['set'] = {'nvm':{},'pindig':{},'pinmode':{}}
  commands['sys']['set']['nvm']['user_input'] = [{'type':'hex','range':nvm_adr_range},
                                                 {'type':'hex','range':['00','FF']}]
  commands['sys']['set']['pindig']['user_input'] = [{'type':'set','range':dpin_names},
                                                    {'type':'int','range':[0,1]}]
  commands['sys']['set']['pinmode']['user_input'] = [{'type':'set','range':dpin_names},
                                                     {'type':'set','range':pin_modes}]
  commands['sys']['get'] = {'ver':{},'nvm':{},'vdd':{},'hweui':{},'pindig':{},'pinana':{}}
  commands['sys']['get']['nvm']['user_input'] = [{'type':'hex','range':nvm_adr_range}]
  commands['sys']['get']['pindig']['user_input'] = [{'type':'set','range':dpin_names}]
  commands['sys']['get']['pinana']['user_input'] = [{'type':'set','range':apin_names}]

  # mac commands
  # TODO: LoRaWAN not yet implemented. Default RN2903 firmware doesn't seem to support??
  # Class B or C fully. 
  commands['mac'] = {'reset':{},'pause':{},'resume':{},'save':{},'get':{}}
  commands['mac']['get'] = { 'adr':{},
                             'appeui':{},
                             'ar':{},
                             'ch':{},
                             'class':{},
                             'dcycleps':{},
                             'devaddr':{},
                             'deveui':{},
                             'dnctr':{},
                             'dr':{},
                             'gwnb':{},
                             'mcast':{},
                             'mcastdevaddr':{},
                             'mcastdnctr':{},
                             'mrgn':{},
                             'pwridx':{},
                             'retx':{},
                             'rx2':{},
                             'rxdelay1':{},
                             'rxdelay2':{},
                             'status':{},
                             'sync':{},
                             'upctr':{} }
  commands['mac']['get']['ch'] = {'freq':{},'drrange':{},'status':{}}
  commands['mac']['get']['ch']['freq']['user_input'] = [{'type':'int','range':[0,71]}]
  commands['mac']['get']['ch']['drrange']['user_input'] = [{'type':'int','range':[0,71]}]
  commands['mac']['get']['ch']['status']['user_input'] = [{'type':'int','range':[0,71]}]

  # radio commands
  commands['radio'] = {'rx':{},
                       'tx':{},
                       'cw':{},
                       'rxstop':{},
                       'set':{},
                       'get':{}}
  commands['radio']['rx']['user_input'] = [{'type':'int','range':[0,65535]}]
  commands['radio']['tx']['user_input'] = [{'type':'hex','range':['0',max_tx_msg]}]
  commands['radio']['set'] = {'afcbw':{},
                              'bitrate':{},
                              'bt':{},
                              'bw':{},
                              'cr':{},
                              'crc':{},
                              'fdev':{},
                              'freq':{},
                              'iqi':{},
                              'mod':{},
                              'prlen':{},
                              'pwr':{},
                              'rxbw':{},
                              'sf':{},
                              'sync':{},
                              'wdt':{} }
  commands['radio']['set']['afcbw']['user_input'] = [{'type':'set','range':auto_freq_band}]
  commands['radio']['set']['bitrate']['user_input'] = [{'type':'int','range':[1,300000]}]
  commands['radio']['set']['bt']['user_input'] = [{'type':'set','range':gfsk_modulation_bt}]
  commands['radio']['set']['bw']['user_input'] = [{'type':'set','range':['125','250','500']}]
  commands['radio']['set']['cr']['user_input'] = [{'type':'set','range':['4/5','4/6','4/7','4/8']}]
  commands['radio']['set']['crc']['user_input'] = [{'type':'set','range':['on','off']}]
  commands['radio']['set']['fdev']['user_input'] = [{'type':'int','range':[0,200000]}]
  commands['radio']['set']['freq']['user_input'] = [{'type':'int','range':[902000000,928000000]}]
  commands['radio']['set']['iqi']['user_input'] = [{'type':'set','range':['on','off']}]
  commands['radio']['set']['mod']['user_input'] = [{'type':'set','range':['lora','fsk']}]
  commands['radio']['set']['prlen']['user_input'] = [{'type':'int','range':[0,65535]}]
  commands['radio']['set']['pwr']['user_input'] = [{'type':'int','range':[2,20]}]
  commands['radio']['set']['rxbw']['user_input'] = [{'type':'set','range':rx_bandwidth}]
  commands['radio']['set']['sf']['user_input'] = [{'type':'set','range':spread_factors}]
  commands['radio']['set']['sync']['user_input'] = [{'type':'hex','range':['0','FFFFFFFFFFFFFFFF']}]
  commands['radio']['set']['wdt']['user_input'] = [{'type':'int','range':[0,4294967295]}]
  commands['radio']['get'] = {'afcbw':{},
                              'bitrate':{},
                              'bt':{},
                              'bw':{},
                              'cr':{},
                              'crc':{},
                              'fdev':{},
                              'freq':{},
                              'iqi':{},
                              'mod':{},
                              'prlen':{},
                              'pwr':{},
                              'rssi':{},
                              'rxbw':{},
                              'sf':{},
                              'snr':{},
                              'sync':{},
                              'wdt':{} }


  # end device constants
  ##################################################################################################


  ##################################################################################################
  # init -  on instantiation, establish handle on serial port and initialize instance variables.
  # Default settings are 57600 bps, 8 bits, no parity, 1 Stop bit, no flow control.
  # The baud rate can be changed by triggering the auto-baud detection sequence of the module.
  # To do this, the host system needs to transmit to the module a break condition followed by a 0x55
  # character at the new baud rate. The auto-baud detection mechanism can also be triggered during
  # Sleep to wake the module up before the predetermined time has expired. Auto-detect sequence is
  # not currently implemented.
  #

  def __init__(self,conf_file='rn2903_config.json'):
    logging.debug('rn2903.init started with %s',str(conf_file))

    # configuration file
    self.config_file = str(conf_file)

    # instance vars
    self.dev = '/dev/ttyACM0'
    self.ser_timeout = 5  # seconds

    # serial device handle
    self.dev_object = None

    # current state of RN2903
    self.dev_status = None

    # safe mode blocks eraseFW and factoryRESET system calls, choices ['on','off']
    self.safe_mode = 'on' 

    # default configuration
    self.device_config = { 'sys': {'ver': '',
                                   'nvm': '',
                                   'hweui': ''},
                           'mac': {'adr': 'off',
                                   'appeui': '0000000000000000',
                                   'ar': 'off',
                                   'class': 'A',
                                   'dcycleps': '1',
                                   'devaddr': '00000000',
                                   'deveui': '0000000000000000',
                                   'dnctr': '0',
                                   'dr': 'XXX',
                                   'gwnb': '0',
                                   'mcast': 'off',
                                   'mcastdevaddr': '00000000',
                                   'mcastdnctr': '0',
                                   'mrgn': '255',
                                   'pwridx': '5',
                                   'retx': '7',
                                   'rx2': '8 923300000',
                                   'rxdelay1': '1000',
                                   'rxdelay2': '2000',
                                   'status': '00000100',
                                   'sync': '34',
                                   'upctr': '0'},
                           'radio': {'afcbw': '41.7',
                                     'bitrate': '50000',
                                     'bt': '0.5',
                                     'bw': '125',
                                     'cr': '4/5',
                                     'crc': 'on',
                                     'fdev': '25000',
                                     'freq': '923300000',
                                     'iqi': 'off',
                                     'mod': 'lora',
                                     'prlen': '8',
                                     'pwr': '2',
                                     'rxbw': '25',
                                     'sf': 'sf12',
                                     'sync': '34',
                                     'wdt': '15000'} }

    # try to pull local config
    config = self.read_file_config(conf_file)

    # grab local config overrides if exist
    if config:
      if 'dev' in config:
        self.dev = str(config['dev'])

    # open serial device
    try:
      self.dev_object = serial.Serial(self.dev,
                                      self.ser_baud,
                                      timeout=self.ser_timeout)
      self.dev_status = 'idle'
    except Exception as err_msg:
      self.dev_status = 'error'
      logging.error('rn2903.init serial failed: %s %s',str(self.dev),str(err_msg))

    # check running firmware against target
    if self.dev_status == 'idle':
      status = ''
      running_firmware = ''
      module = ''
      version = ''
      try:
        (status,running_firmware) = self.send_command('sys get ver')
        if status == 'ok':
          (module,version,mm,dd,yy,tm) = running_firmware.split(' ')
      except Exception as err_msg:
        self.dev_status = 'error'
        logging.error('rn2903.init failed to get firmware: %s',str(err_msg))
      if not module == self.firmware_name or not version == self.target_version:
        self.dev_status = 'error'
        logging.error('rn2903.init target firmware not match running: %s-%s %s',
                      str(self.firmware_name),str(self.target_version),str(running_firmware))
      else:
        logging.debug('rn2903.init firmware good match: %s %s %s',
                      str(self.firmware_name),str(self.target_version),str(running_firmware))

    # get current running device config
    if self.dev_status == 'idle':
      self.device_config = self.get_device_config()
      if not self.device_config:
        self.dev_status = 'error'

    # grab local config overrides if exist
    if config:
      if 'device_config' in config:
        self.device_config.clear()
        self.device_config = config['device_config']
        # push config to device
        self.put_device_config()

    # else save a default config to file
    else:
      config = {}
      config.clear()
      config['dev'] = str(self.dev)
      config['device_config'] = {}
      config['device_config'] = self.device_config
      if not self.save_file_config(conf_file,config):
        logging.error('rn2903.init failed to save default config file: %s',str(conf_file))


    # log device used
    logging.info('rn2903.init opened serial %s in state %s',str(self.dev),str(self.dev_status))

  #
  # end init
  ##################################################################################################


  ##################################################################################################
  # start in context manager
  ##################################################################################################
  def __enter__(self):
    logging.debug('rn2903.enter started')
    return self
    

  ##################################################################################################
  # assumed case is use within context manager, call close on serial port
  ##################################################################################################
  def __exit__(self, exc_type, exc_value, exc_traceback):
    logging.debug('rn2903.exit started')

    logging.debug('rn2903.exit calling del')
    self.__del__()


  ##################################################################################################
  # if used outside a context manager, try and close serial port
  ##################################################################################################
  def __del__(self):
    if self.dev_object:
      try:
        self.dev_object.close()
      except Exception as err_msg:
        logging.error('rn2903.del failed close dev: %s',str(err_msg))


  ##################################################################################################
  # check_param - validate command parameter against device specification
  #

  def check_param(self,ps_param,ps_param_def):
    logging.debug('rn2903.check_param called with: %s %s',str(ps_param),str(ps_param_def))

    # validate parameter
    if ps_param:
      param = str(ps_param)
    else:
      logging.error('rn2903.check_param missing required parameter')
      return False

    # validate specification structure
    if isinstance(ps_param_def,dict):
      if not 'type' in ps_param_def or not 'range' in ps_param_def:
        logging.error('rn2903.check_param called with bad param_def missing keys: %s',
                      str(ps_param_def))
        return False
      if not isinstance(ps_param_def['range'],list):
        logging.error('rn2903.check_param called with bad param_def, range is not list: %s',
                      str(type(ps_param_def['range'])))
        return False
      if not len(ps_param_def['range']) > 0:
        logging.error('rn2903.check_param called with bad param_def, range is empty list')
        return False
      param_def = ps_param_def
    else:
      logging.error('rn2903.check_param called with bad param_def type: %s',
                    str(type(ps_param_def)))
      return False

    # check int
    if param_def['type'] == 'int':
      try:
        if not param.isdigit():
          logging.error('rn2903.check_param parameter must be digit: %s',param)
          return False
        if int(param) < int(param_def['range'][0]):
          logging.error('rn2903.check_param parameter below range: %s',
                        str(list(param_def['range'])))
          return False
        elif int(param) > int(param_def['range'][1]):
          logging.error('rn2903.check_param parameter above range: %s',
                        str(list(param_def['range'])))
          return False
      except Exception as err_msg:
        logging.error('rn2903.check_param int check failed: %s',str(err_msg))
        return False

    # check hex
    elif param_def['type'] == 'hex':
      try:
        param = param.lower()
        pattern = re.compile("[^0-9a-f]")
        if pattern.search(param):
          logging.error('rn2903.check_param parameter must be hex: %s',param)
          return False
        pval = int(param,16)
        rmin = int(param_def['range'][0],16)
        rmax = int(param_def['range'][1],16)
        if pval < rmin:
          logging.error('rn2903.check_param parameter below range: %s',
                        str(list(param_def['range'])))
          return False
        elif pval > rmax:
          logging.error('rn2903.check_param parameter above range: %s',
                        str(list(param_def['range'])))
          return False
      except Exception as err_msg:
        logging.error('rn2903.check_param hex check failed: %s',str(err_msg))
        return False

    # check set, assumes elements in 'range' definition are type str
    elif param_def['type'] == 'set':
      try:
        if not param in param_def['range']:
          logging.error('rn2903.check_param parameter "%s" must be in: %s',
                        param,str(list(param_def['range'])))
          return False
      except Exception as err_msg:
        logging.error('rn2903.check_param set check failed: %s',str(err_msg))
        return False

    # definition broken
    else:
      logging.error('rn2903.check_param definition error, bad user_input.type')
      return False

    return True

  # end check_param
  ##################################################################################################


  ##################################################################################################
  # validate_command - validates a passed string as a supported RN2903 command
  #
  def validate_command(self,ps_cmd):
    logging.debug('rn2903.validate_command called with %s',str(ps_cmd))

    send_str = ''
    cmd = str(ps_cmd)

    # should be at least two space-separated parameters
    if ' ' not in cmd:
      logging.error('rn2903.validate_command malformed, needs at least two args: %s',str(cmd))
      return False

    # parse command parameters. no supported commands extend beyond five parameters 
    # currently unsupported mac subcommands can have up to seven parameters
    cmd_set = cmd.split()
    cmd_type = cmd_set[0].lower()
    cmd_base = cmd_set[1]
    cmd_par1 = ''
    cmd_par2 = ''
    cmd_par3 = ''
    if len(cmd_set) > 2:
      cmd_par1 = cmd_set[2]
    if len(cmd_set) > 3:
      cmd_par2 = cmd_set[3]
    if len(cmd_set) > 4:
      cmd_par3 = cmd_set[4]

    # check command type
    if cmd_type in self.commands:
      # append to command
      send_str += cmd_type
    else:
      logging.error('rn2903.validate_command invalid command type: %s',str(cmd_type))
      return False

    # check command base
    if cmd_base in self.commands[cmd_type]:
      # append to command
      send_str += ' '
      send_str += cmd_base
    else:
      logging.error('rn2903.validate_command invalid command base: %s',str(cmd_base))
      return False

    # check parameters
    if self.commands[cmd_type][cmd_base]:

      # check if param is user input
      if 'user_input' in self.commands[cmd_type][cmd_base]:
        logging.debug('rn2903.validate_command third parm is user input')

        # validate user input against specification
        if self.check_param(cmd_par1,
                            self.commands[cmd_type][cmd_base]['user_input'][0]):
          # append to command
          send_str += ' '
          send_str += cmd_par1
        else:
          logging.error('rn2903.validate_command user input invalid: %s',str(cmd_par1))
          return False
        # check if second param is required and valid
        if len(self.commands[cmd_type][cmd_base]['user_input']) == 2:
          if self.check_param(cmd_par2,
                              self.commands[cmd_type][cmd_base]['user_input'][1]):
            # append to command
            send_str += ' '
            send_str += cmd_par2
          else:
            logging.error('rn2903.validate_command additional input invalid: %s',str(cmd_par2))
            return False

      # check if param is sub-command
      elif cmd_par1 in self.commands[cmd_type][cmd_base]:
        logging.debug('rn2903.validate_command third parm is subcommand')

        send_str += ' '
        send_str += cmd_par1

        # check additional parameters
        if self.commands[cmd_type][cmd_base][cmd_par1]:
          if 'user_input' in self.commands[cmd_type][cmd_base][cmd_par1]:
            if self.check_param(cmd_par2,
                                self.commands[cmd_type][cmd_base][cmd_par1]['user_input'][0]):
              # append to command
              send_str += ' '
              send_str += cmd_par2
            else:
              logging.error('rn2903.validate_command par2 invalid : %s',str(cmd_par2))
              return False
            # check if second param is required and valid
            if len(self.commands[cmd_type][cmd_base][cmd_par1]['user_input']) == 2:
              if self.check_param(cmd_par3,
                                  self.commands[cmd_type][cmd_base][cmd_par1]['user_input'][1]):
                send_str += ' '
                send_str += cmd_par3
              else:
                logging.error('rn2903.validate_command par3 invalid : %s',str(cmd_par3))
                return False
          # or subcommand
          elif cmd_par2 in self.commands[cmd_type][cmd_base][cmd_par1]:
            logging.debug('rn2903.validate_command fourth parm is subcommand')
            if 'user_input' in self.commands[cmd_type][cmd_base][cmd_par1][cmd_par2]:
              if self.check_param(cmd_par3,self.commands[cmd_type][cmd_base][cmd_par1][cmd_par2]['user_input'][0]):
                send_str += ' '
                send_str += cmd_par3

      # else invalid param supplied
      else:
        logging.error('rn2903.validate_command invalid subcommand: %s',str(cmd_par1))
        return False

    # log and return
    logging.debug('rn2903.validate_command finished with: %s',str(send_str))
    return send_str

  #
  # end validate_command
  ##################################################################################################


  ##################################################################################################
  # write_command - writes a string to the serial device
  #

  def write_command(self,cmd_str):
    logging.debug('rn2903.write_command started with %s',str(cmd_str))
 
    # ensure arg is string
    send_str = str(cmd_str)

    # ensure we have more than an empty string
    if not send_str:
      logging.error('rn2903.write_command invalid cmd_str: %s',str(cmd_str))
      return False

    # format command for write
    send_str += self.data_termintaor

    # send command
    try:
      self.dev_object.write(str.encode(send_str))
    except Exception as err_msg:
      logging.error('rn2903.write_command write errored: %s',str(err_msg))
      return False

    return True

  #
  # end write_command
  ##################################################################################################


  ##################################################################################################
  # read_response - listens on serial port until a termintaor sequence is recieved or timeout
  #

  def read_response(self):
    logging.debug('rn2903.read_response started')

    # local vars
    resp = ''
    s = ''
    count = 0
    max_ch = 256

    # reader loop
    while count < max_ch:
      try:
        s = self.dev_object.read().decode("utf-8")
        resp += s
      except Exception as err_msg:
        logging.error('rn2903.read_response read errored: %s',str(err_msg))
        return False
      if s == "\n":
        break
      count = (count + 1)
      logging.debug('rn2903.read_response read loop %s with %s',str(count),str(s))

    # strip message terminator
    resp = resp.replace(self.data_termintaor,'')

    # return message
    logging.debug('rn2903.read_response finshed with: %s',str(resp))
    return resp

  #
  # end read_response
  ##################################################################################################


  ##################################################################################################
  # send_command - writes the passed string to the serial port then listens for response from the 
  # module. Not all commands return a response, while some return two responses.  Responses are:
  # - ok
  # - busy
  # - fram_counter_err_rejoin_needed
  # - invalid_class
  # - invalid_data_len
  # - invalid_param
  # - keys_not_init
  # - mac_paused
  # - multicast_keys_not_set
  # - no_free_ch
  # - not_joined
  # - silent
  # - err
  # - [ascii string data]
  # - [RN2903 X.Y.Z MMM DD YYYY HH:MM:SS] (firmware version)
  # This function normalizes these various calls and always returns a two-element set with the 
  # first element being the logical result, and the second element containing data, if any.
  # 

  def send_command(self,ps_cmd):
    logging.debug('rn2903.send_command called with %s',str(ps_cmd))

    # ignore command request if device is in bad state
    if self.dev_status == 'error':
      logging.error('rn2903.send_command device in error state, ignoring command: %s',str(ps_cmd))
      return (False,False)

    # local vars
    send_str = ''
    status = ''
    response = ''
    err_status = ['invalid_param','busy','radio_err']
    
    # ensure we have valid command
    send_str = self.validate_command(str(ps_cmd))
    if not send_str:
      logging.error('rn2903.send_command called with invalid command')
      return (False,False)

    # sys sleep, doesn't return until done sleeping, or invalid_param
    if send_str.startswith('sys sleep'):
      mils = send_str.split(' ')[2]
      logging.debug('rn2903.send_command sys sleep called, will block for %s milliseconds',mils)
      # send command
      if self.write_command(send_str):
        # read response
        status = self.read_response()
        if status == 'ok':
          response = mils
        else:
          logging.error('rn2903.send_command sys sleep errored: %s',str(status))
          return (False,False)

    # sys reset, response == sys get ver
    elif send_str.startswith('sys reset'):
      logging.debug('rn2903.send_command sys reset called, rebooting')
      # send command
      if self.write_command(send_str):
        # read response
        status = self.read_response()
        # expecting version as response: RN2903 X.Y.Z MMM DD YYYY HH:MM:SS
        try:
          (module,version,mm,dd,yy,tm) = status.split(' ')
          if module == self.firmware_name:
            response = status
            status = 'ok'
          else:
            logging.error('rn2903.send_command sys reset errored: %s',str(status))
            return (False,False)
        except Exception as err_msg:
          logging.error('rn2903.send_command sys reset errored: %s',str(status))
          return (False,False)

    # sys eraseFW, NO RESPONSE
    # !!! WARNING !!! do not call this method unless able to load firmware via other means
    elif send_str.startswith('sys eraseFW'):
      if self.safe_mode == 'off':
        logging.error('rn2903.send_command sys eraseFW, device will hang until user loads firmware')
        if self.write_command(send_str):
          status = 'ok'
          response = 'awaiting firmware'
      else:
        logging.info('rn2903.send_command sys eraseFW call ignored in safe_mode')
        return (False,False)

    # sys factoryRESET response == sys get ver
    elif send_str.startswith('sys factoryRESET'):
      if self.safe_mode == 'off':
        logging.error('rn2903.send_command sys factoryRESET, device will hang until rebooted')
        # send command
        if self.write_command(send_str):
          # read response
          status = self.read_response()
          # expecting version as response: RN2903 X.Y.Z MMM DD YYYY HH:MM:SS
          try:
            (module,version,mm,dd,yy,tm) = status.split(' ')
            if module == self.firmware_name:
              response = status
              status = 'ok'
            else:
              logging.error('rn2903.send_command sys reset errored: %s',str(status))
              return (False,False)
          except Exception as err_msg:
            logging.error('rn2903.send_command sys reset errored: %s',str(status))
            return (False,False)
      else:
        logging.info('rn2903.send_command sys factoryRESET call ignored in safe_mode')
        return (False,False)

    # radio rx, two responses: status, radio_rx <data> || radio_err
    elif send_str.startswith('radio rx '):
      syms = send_str.split(' ')[2]
      if syms == '0':
        logging.debug('rn2903.send_command radio rx blocking until message received')
      else:
        logging.debug('rn2903.send_command radio rx blocking until %s symbols received',syms)

    # radio tx, two responses: status, radio_tx_ok || radio_err
    elif send_str.startswith('radio tx'):
      logging.debug('rn2903.send_command radio tx called')

    # all other commands
    else:
      logging.debug('rn2903.send_command other %s called',str(send_str))
      # send command
      if self.write_command(send_str):
        # read response
        status = self.read_response()
        if status in err_status:
          logging.error('rn2903.send_command returned error: %s',str(status))
          return (False,str(status))
        elif status != 'ok':
          response = status
          status = 'ok'

    # return results
    logging.debug('rn2903.send_command finished with: %s %s',str(status),str(response))
    return (status,response)

  #
  # end send_commnd
  ##################################################################################################


  ##################################################################################################
  # get_device_config - retrieves all settings from the RN2903 device
  #

  def get_device_config(self):
    logging.debug('rn2903.get_device_config started')

    # commands to run to get all settings:
    config_cmds = ['sys get ver',
                   'sys get nvm',
                   'sys get hweui',
                   'mac get adr',
                   'mac get appeui',
                   'mac get ar',
                   'mac get class',
                   'mac get dcycleps',
                   'mac get devaddr',
                   'mac get deveui',
                   'mac get dnctr',
                   'mac get dr',
                   'mac get gwnb',
                   'mac get mcast',
                   'mac get mcastdevaddr',
                   'mac get mcastdnctr',
                   'mac get mrgn',
                   'mac get pwridx',
                   'mac get retx',
                   'mac get rx2',
                   'mac get rxdelay1',
                   'mac get rxdelay2',
                   'mac get status',
                   'mac get sync',
                   'mac get upctr',
                   'radio get afcbw',
                   'radio get bitrate',
                   'radio get bt',
                   'radio get bw',
                   'radio get cr',
                   'radio get crc',
                   'radio get fdev',
                   'radio get freq',
                   'radio get iqi',
                   'radio get mod',
                   'radio get prlen',
                   'radio get pwr',
                   'radio get rxbw',
                   'radio get sf',
                   'radio get sync',
                   'radio get wdt']

    (status,response) = self.send_command('mac pause')

    # config struct
    config = {'sys':{},'mac':{},'radio':{}}

    # run all config read commands
    for cmd in config_cmds:

      # parse command for logging
      (ctype,cbase,cparm) = cmd.split(' ')

      # iterate range of memory addresses
      if cmd == 'sys get nvm':
        adr = 0
        nvm_data = ''
        while adr < 256:
          cmd = 'sys get nvm'
          nvm_adr = format(adr, 'x').upper()
          if len(nvm_adr) < 2:
            nvm_adr = '30'+str(nvm_adr)
          else:
            nvm_adr = '3'+str(nvm_adr)
          cmd = cmd + ' ' + nvm_adr
          (status,response) = self.send_command(cmd)
          if status == 'ok':
            nvm_data += response
          else:
            print('ERROR: ',cmd)
          adr = ( adr + 1)
        config['sys']['nvm'] = nvm_data

      # iterate all (freq,drrange,status) for channels
      elif cmd == 'mac get ch':
        logging.debug('rn2903.get_device_config "mac get ch" not yet supported')

      # run command
      else:
        (status,response) = self.send_command(cmd)
        if status != 'ok':
          print(cmd)
        else:
          config[ctype][cparm] = response

    (status,response) = self.send_command('mac resume')

    return config

  #
  # end get_device_config
  ##################################################################################################


  ##################################################################################################
  # put_device_config - pushes the in-memory configuration onto the RN2903 and saves to the device
  #

  def put_device_config(self):
    logging.debug('rn2903.put_device_config started')

    # validate object config settings
    config_valid = True 
    for setting in self.device_config['radio']:
      val = self.device_config['radio'][setting]
      cmd = 'radio set '+setting+' '+val
      if not self.validate_command(cmd):
        logging.error('rn2903.put_device_config invalid config parameter: %s',str(cmd))
        config_valid = False

    # if have valid config
    if config_valid:

      # pause LoRa WAN system
      (status,response) = self.send_command('mac pause')
      if status != 'ok':
        logging.error('rn2903.put_device_config mac pause failed: %s %s',str(status),str(response))
        return False

      # iterate self.device_config
      set_config = True
      for setting in self.device_config['radio']:
        val = self.device_config['radio'][setting]
        cmd = 'radio set '+setting+' '+val
        (status,response) = self.send_command(cmd)
        if status != 'ok':
          logging.error('rn2903.put_device_config set config failed: %s',str(cmd))
          set_config = False
          break

      # save config
      if set_config:
        (status,response) = self.send_command('mac save')
        if status == 'ok':
          logging.debug('rn2903.put_device_config mac save updated config OK')
        else:
          logging.error('rn2903.put_device_config mac save failed: %s %s',str(status),str(response))

      # resume LoRa WAN system
      (status,response) = self.send_command('mac resume')
      if status != 'ok':
        logging.error('rn2903.put_device_config mac resume failed: %s %s',str(status),str(response))
        return False


    return True

  #
  # end put_device_config
  ##################################################################################################


  ##################################################################################################
  # read_file_config - reads configuration settings from a JSON file
  #

  def read_file_config(self,ps_filename):
    logging.debug('rn2903.read_file_config started with: %s',str(ps_filename))
    config = {}
    filename = str(ps_filename)
    file_str = ''
    if not os.path.isfile(filename):
      logging.error('rn2903.read_file_config file not exist: %s',filename)
      return False
    if not os.access(filename, os.R_OK):
      logging.error('rn2903.read_file_config file not readable: %s',filename)
      return False
    try:
      with open(filename,'r') as conf_file:
        file_str = conf_file.read()
    except Exception as err_msg:
      logging.error('rn2903.read_file_config read file error: %s',str(err_msg))
      return False
    if file_str:
      try:
        config = json.loads(file_str)
      except Exception as err_msg:
        logging.error('rn2903.read_file_config json decode error: %s',str(err_msg))
        return False

    logging.debug('rn2903.read_file_config ended with: %s',str(config))
    return config

  #
  # end read_file_config
  ##################################################################################################


  ##################################################################################################
  # save_file_config - writes configuration settings to a JSON file
  #

  def save_file_config(self,ps_filename,config):
    logging.debug('rn2903.save_file_config started with: %s',str(ps_filename))
    filename = str(ps_filename)
    file_str = ''
    try:
      file_str = json.dumps(config,indent=2)
    except Exception as err_msg:
      logging.error('rn2903.save_file_config json encode error: %s',str(err_msg))
      return False
    try:
      fp = open(ps_filename, "w")
      fp.write(str(file_str))
      fp.close()
      return True
    except Exception as e:
      logging.error('rn2903.save_file_config save error: %s',str(e))

    return True

  #
  # end save_file_config
  ##################################################################################################

#
# end RN2903
####################################################################################################



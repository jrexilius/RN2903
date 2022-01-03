#!/usr/bin/python3
####################################################################################################
# Description
# -----------
# This is a barebones script to test basic connectivity and functionality of the Microchip RN2903
# LoRa module.  This is intended to be run at the command line and support manual testing and 
# diagnostics. It can be used to do basic sending and receiving of messages via LoRa P2P radio.
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
import logging
from rn2903 import *


####################################################################################################
# main
####################################################################################################

# local config file to save/edit radio settings
# you can run multiple devices by passing different config files
# with different tty serial devices specified in each config file
# if no config file name is passed, default is used 'rn2903_config.json'
config_file = 'MY-rn2903_config.json'


# remove self from args list, construct command string (if any)
# example execution: "./example.py sys get ver"
# full list of commands can be read from the rn2903 class or docs at:
# https://www.microchip.com/en-us/product/RN2903
cmd = ''
args = sys.argv
args.pop(0)
if len(args) > 1: # require at least two args
  for arg in args:
    cmd+= str(arg)
    cmd+= ' '
  cmd = cmd[:-1]

# instantiate a rn2903 object
# upon initialization it will read and load config settings if file exists
# otherwise it will read the defaults from the device and save a local copy
with rn2903(config_file) as modem1:
  if cmd:
    (sta,res) = modem1.send_command(cmd)
    print(sta,res)
  else:
    print(json.dumps(modem1.device_config,indent=2))


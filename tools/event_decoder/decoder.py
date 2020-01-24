##########################################################################################
# Copyright (c) 2019 - 2020, Nordic Semiconductor ASA
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from this
#    software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
##########################################################################################

import os
import re

"""
The script goes through all driver source files and searches for DEBUG_LOG_FUNCTION_RE pattern.
Then it prints macro constant for `nrf_802154_debug.h` and `decode.html` entries.
"""

DEBUG_LOG_FUNCTION_RE = re.compile(r'nrf_802154_log_entry\(\s*(\w+)\s*,\s*\d+\s*\);', re.MULTILINE)

DECODER_AND_DEFINES_TEMPLATE = \
'''
{decoder}

{defines}
'''

DRV_SRC_PATH = os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../src'))

print('Searching sources in "{}"'.format(DRV_SRC_PATH))

MODULE_HEX_ID_MAP = {
    'nrf_802154_rsch.c': '1000'
}

for dir_path, dirs, files in os.walk(DRV_SRC_PATH):
    for file in files:
        with open(os.path.join(dir_path, file)) as file_handler:
            module_id = MODULE_HEX_ID_MAP.get(file, 'FFFF')
            print('{} ({})'.format(file, module_id))

            decoder = []
            defines = []

            for index, function in enumerate(DEBUG_LOG_FUNCTION_RE.findall(file_handler.read())):
                hex_id = hex(int(module_id, 16) + index)
                decoder.append('            {{id: "FUNCTION_{}", val: {}, from: "RSCH", to: "RSCH", text: "{}()"}},'.format(function, hex_id, function))
                defines.append('#define FUNCTION_{:<40} {}UL'.format(function, hex_id))

            if decoder:
                print(DECODER_AND_DEFINES_TEMPLATE.format(decoder='\n'.join(decoder), defines='\n'.join(defines)))


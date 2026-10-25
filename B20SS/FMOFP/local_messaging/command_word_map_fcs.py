"""
Command Word Map for Flight Control System

This module defines the command word mappings for the Flight Control System
to ensure proper integration with the MIL-STD-1553B protocol.
"""

class CommandWord:
    """
    Defines a 1553B command word with name, value, and description.
    """
    def __init__(self, name, value, description=None):
        self.name = name
        self.value = value  
        self.description = description or ""
from FMOFP.local_messaging.message_types_fcs import (
    FCS_CONTROL_INPUT_REQUEST,
    FCS_CONTROL_INPUT_RESPONSE,
    FCS_ORIENTATION_DATA_REQUEST,
    FCS_ORIENTATION_DATA_RESPONSE,
    FCS_STATUS_REQUEST,
    FCS_STATUS_RESPONSE,
    FCS_MODE_CHANGE_REQUEST,
    FCS_MODE_CHANGE_RESPONSE
)

# Flight Control System command words, derived from the address book.
#
# WHAT THIS REPLACED (story C25.1)
# --------------------------------
# These eight command words used to be invented here:
#
#     FCS_BASE_COMMAND = 0x4000   # "different range from FMS"
#     ... value=FCS_BASE_COMMAND + 0x01 ... + 0x08
#
# A 1553B command word in this codebase is not a free-running counter. It is
# five fields packed into sixteen bits -- RT address (5) | T/R (1) |
# subaddress (5) | word count (5) -- and every other system's command words
# are built from the address book by register_command_word() so those fields
# land where the bus controller and the RT message extractor read them.
#
# BusControllerModule.formatCommand() slices exactly those fields out of
# whatever string it is handed, so the invented values were not inert:
#
#     0x4007 (FCS_MODE_CHANGE_REQUEST)  = 0100000000000111
#                                         01000 -> RT address 8
#                                         0     -> T/R
#                                         00000 -> subaddress 0
#                                         00111 -> word count 7
#
# RT address 8 is Power Management (address_book.xml, and
# rt_message_extractor's own RT-to-system map agrees: 8 -> power_management).
# So every FCS mode change and status request that this handler sent went out
# addressed to Power Management, subaddress 0. Those are the two FCS request
# types that DID reach the bus; the data requests were refused earlier, which
# is the half of C25.1 the story is written about.
#
# The FCS's real address was in the tree the whole time: rtAddressConfig.xml
# has RT 5 for "flightcontrolsys" and subaddress 10 for "flight_control". It
# was unusable only because address_book.xml had left the FCS system entry
# under the duplicate id "unnamed_system" -- fixed in the same change, see
# the comment beside that entry.
#
# WORD COUNT
# ----------
# The count below is the message-type discriminator, exactly as
# register_command_word() uses it for the radars (status 1, mode 2, data 3,
# and a per-data-type count beyond that). It is what keeps the eight FCS
# words distinct from each other, since RT address and subaddress are fixed
# for the whole subsystem and only T/R varies between request and response.
# It is not a payload length: BC_construct.construct_communication_frame()
# recomputes the real count from the data words actually being sent
# (actual_data_word_count) before putting the command word on the wire.
FCS_RT_SYSTEM_ID = 'flightcontrolsys'
FCS_SUBADDRESS_ID = 'flight_control'

_FCS_WORD_COUNTS = {
    'status': 1,
    'mode': 2,
    'data': 3,          # control input
    'orientation': 4,   # orientation data
}


def _fcs_command_word(kind, t_or_r):
    """
    Build an FCS command word with the same field layout every other system
    uses. Returns the 16-bit value as an int, which is the form every
    consumer of FCS_COMMAND_WORDS expects (each does format(value, '016b')).
    """
    from FMOFP.MIL_STD_1553B.address_utils import get_rt_address, get_subaddress

    rt_address = get_rt_address(FCS_RT_SYSTEM_ID)
    subaddress = get_subaddress(FCS_SUBADDRESS_ID)
    word_count = _FCS_WORD_COUNTS[kind]

    binary = (
        format(rt_address, '05b')
        + format(t_or_r, '01b')
        + format(subaddress, '05b')
        + format(word_count, '05b')
    )
    return int(binary, 2)


FCS_COMMAND_WORDS = {}

try:
    FCS_COMMAND_WORDS = {
        FCS_CONTROL_INPUT_REQUEST: CommandWord(
            name=FCS_CONTROL_INPUT_REQUEST,
            value=_fcs_command_word('data', 0),
            description="Request to set control input for flight control surfaces"
        ),
        FCS_CONTROL_INPUT_RESPONSE: CommandWord(
            name=FCS_CONTROL_INPUT_RESPONSE,
            value=_fcs_command_word('data', 1),
            description="Response to control input request"
        ),
        FCS_ORIENTATION_DATA_REQUEST: CommandWord(
            name=FCS_ORIENTATION_DATA_REQUEST,
            value=_fcs_command_word('orientation', 0),
            description="Request for aircraft orientation data"
        ),
        FCS_ORIENTATION_DATA_RESPONSE: CommandWord(
            name=FCS_ORIENTATION_DATA_RESPONSE,
            value=_fcs_command_word('orientation', 1),
            description="Response containing aircraft orientation data"
        ),
        FCS_STATUS_REQUEST: CommandWord(
            name=FCS_STATUS_REQUEST,
            value=_fcs_command_word('status', 0),
            description="Request for flight control system status"
        ),
        FCS_STATUS_RESPONSE: CommandWord(
            name=FCS_STATUS_RESPONSE,
            value=_fcs_command_word('status', 1),
            description="Response with flight control system status information"
        ),
        FCS_MODE_CHANGE_REQUEST: CommandWord(
            name=FCS_MODE_CHANGE_REQUEST,
            value=_fcs_command_word('mode', 0),
            description="Request to change flight control system mode"
        ),
        FCS_MODE_CHANGE_RESPONSE: CommandWord(
            name=FCS_MODE_CHANGE_RESPONSE,
            value=_fcs_command_word('mode', 1),
            description="Response to flight control system mode change request"
        )
    }
except Exception as _exc:   # noqa: BLE001 - any address book problem, same answer
    # Deliberately left EMPTY rather than falling back to the old invented
    # values. Every consumer of FCS_COMMAND_WORDS already handles a missing
    # entry by logging an error and refusing to send (FCSMessageHandler's
    # four request branches, _register_message_handlers, and the map
    # registration in command_word_map.py). Refusing to transmit is the
    # honest failure; transmitting to Power Management's address is not.
    from FMOFP.Utils.logger.sys_logger import get_logger as _get_logger
    _get_logger().error(
        f"[FCS_CMD] Could not derive FCS command words from the address book "
        f"({type(_exc).__name__}: {_exc}). FCS_COMMAND_WORDS is empty, so FCS "
        f"1553B messages will be refused rather than sent to a wrong address. "
        f"Check that address_book.xml has a system entry with id "
        f"'{FCS_RT_SYSTEM_ID}' and a subaddress entry with id "
        f"'{FCS_SUBADDRESS_ID}'."
    )

# Dictionary mapping command word values to command word names
# Used for lookups when decoding 1553B messages
FCS_COMMAND_VALUE_MAP = {cmd.value: name for name, cmd in FCS_COMMAND_WORDS.items()}

def get_fcs_command_word(name):
    """
    Get command word by name
    
    Args:
        name: The name of the command word to retrieve
        
    Returns:
        CommandWord or None if not found
    """
    return FCS_COMMAND_WORDS.get(name)

def get_fcs_command_name(value):
    """
    Get command name by value
    
    Args:
        value: The value of the command word to look up
        
    Returns:
        str: Command name or None if not found
    """
    return FCS_COMMAND_VALUE_MAP.get(value)

def register_fcs_command_words():
    """
    Register FCS command words with the global command registry
    """
    from FMOFP.local_messaging.command_word_map import COMMAND_REGISTRY
    
    # Register each FCS command word in the global registry
    for name, cmd_word in FCS_COMMAND_WORDS.items():
        hex_value = f"0x{cmd_word.value:04X}"
        COMMAND_REGISTRY[name] = hex_value

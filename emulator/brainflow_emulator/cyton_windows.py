import logging
import sys

from brainflow_emulator.emulate_windows_common import get_ports_windows, test_serial
from brainflow_emulator.emulate_common import Listener


def main(cmd_list):
    global Listener
    if not cmd_list:
        raise Exception('No command to execute')

    m_name, s_name = get_ports_windows()
    test_serial(cmd_list, m_name, s_name, Listener)

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main(sys.argv[1:])

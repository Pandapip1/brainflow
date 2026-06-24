#pragma once

#include <string>

#include "board.h"
#include "board_controller.h"
#include "hw_prober.h"
#include "runtime_dll_loader.h"


class BTLibBoard : public Board, public HWProber
{

protected:
    bool initialized;
    DLLLoader *dll_loader;
    // discover() can fail in more than one distinguishable way (autodiscovery unsupported on
    // this OS, device not discoverable, generic failure); HWProber's contract is a plain bool,
    // so the precise BrainFlowExitCodes value is kept here for prepare_session() to surface,
    // the same way BLELibBoard keeps the matched peripheral handle in a member rather than
    // forcing it through the interface's return type.
    int last_discover_error;

    int bluetooth_open_device ();
    int bluetooth_get_data (char *data, int len);
    int bluetooth_write_data (const char *data, int len);
    int bluetooth_close_device ();
    int find_bt_addr (const char *name_selector);

    virtual std::string get_name_selector () = 0;

public:
    BTLibBoard (int board_id, struct BrainFlowInputParams params);
    ~BTLibBoard () override;

    int prepare_session () override;
    int release_session () override;
    int config_board (std::string config, std::string &response) override;

    // HWProber
    bool find_already_connected () override;
    bool discover (int timeout_sec) override;
};

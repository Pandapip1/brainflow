#pragma once

#include <array>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include "bt_lib_board.h"

// Muse2014: support for the original 2014 InteraXon Muse headband (model MU-01).
//
// Unlike every other "Muse" board in BrainFlow (MUSE_2_BOARD, MUSE_S_BOARD,
// MUSE_2016_BOARD, MUSE_S_ATHENA_BOARD - all of which use BLELibBoard / SimpleBLE),
// the original 2014 Muse has no BLE radio at all. It is a classic-Bluetooth (BR/EDR)
// device exposing the standard Serial Port Profile (RFCOMM), confirmed via direct
// SDP inspection (service class "Serial Port" 0x1101, RFCOMM channel 1, plus an
// InteraXon vendor-specific record and an "RN-iAP" record from the Roving Networks
// BR/EDR-to-serial module it uses). This class therefore inherits BTLibBoard (the
// RFCOMM-socket base class also used by Enophone and Mentalab Explore), not
// BLELibBoard.
//
// Protocol notes (reverse-engineered, not from an official spec - see prepare_session
// and read_thread for command bytes, and the .cpp file header comment for the packet
// framing). This has NOT been validated end-to-end against real hardware streaming
// data; treat decoded values as provisional until checked against a known-good
// reference (e.g. Mind Monitor's legacy MU-01 support, if available, or visual
// sanity-checks like an eye-blink artifact showing on the expected channels).
class Muse2014 : public BTLibBoard
{

protected:
    volatile bool keep_alive;
    std::thread streaming_thread;
    std::mutex m;
    std::condition_variable cv;
    volatile int state;

    // Decoder state that must persist across packets (a fresh C0 packet's deltas
    // are relative to the previous packet's final sample), kept in a fixed,
    // never-reordered channel indexing: 0=TP9, 1=AF7, 2=AF8, 3=TP10 (assumed wire
    // order, matching the convention used by BrainFlow's other Muse boards; not
    // independently confirmed for the 2014 unit specifically).
    int carry[4];
    bool have_reference;
    double last_eeg_timestamp;
    unsigned int sample_counter;
    double last_keepalive_timestamp;

    std::string get_name_selector () override;
    void read_thread ();

    bool read_exact (unsigned char *buf, int len);
    bool find_sync ();
    void send_keepalive_if_due ();

    void handle_e0_packet (int num_rows, double *package);
    void handle_c0_packet (int num_rows, double *package);
    void emit_batch (const std::vector<std::array<int, 4>> &samples, int num_rows, double *package);

public:
    Muse2014 (struct BrainFlowInputParams params);
    ~Muse2014 () override;

    int prepare_session () override;
    int start_stream (int buffer_size, const char *streamer_params) override;
    int stop_stream () override;
    int release_session () override;
    int config_board (std::string config, std::string &response) override;
};

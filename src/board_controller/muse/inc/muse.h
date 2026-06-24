#pragma once

#include "ble_lib_board.h"
#include <condition_variable>
#include <mutex>
#include <string>
#include <utility>
#include <vector>


class Muse : public BLELibBoard
{

protected:
    bool is_streaming;
    std::mutex callback_lock;
    std::vector<std::pair<simpleble_uuid_t, simpleble_uuid_t>> notified_characteristics;
    std::pair<simpleble_uuid_t, simpleble_uuid_t> control_characteristics;
    std::vector<std::vector<double>> current_default_buf;
    std::vector<std::vector<double>> current_aux_buf;
    std::vector<std::vector<double>> current_anc_buf;
    std::vector<bool> new_eeg_data;
    std::vector<bool> new_ppg_data;
    double last_fifth_chan_timestamp; // used to determine 4 or 5 channels used
    double last_ppg_timestamp;        // used for timestamp correction
    double last_eeg_timestamp;        // used for timestamp correction
    double last_aux_timestamp;        // used for timestamp correction
    std::string muse_preset;

    int validate_params () override;
    int default_timeout () const override;
    int setup_session () override;
    bool matches_target_name (const char *identifier) override;

public:
    Muse (int board_id, struct BrainFlowInputParams params);
    ~Muse () override;
    int start_stream (int buffer_size, const char *streamer_params) override;
    int stop_stream () override;
    int release_session () override;
    int config_board (std::string config, std::string &response) override;
    int config_board (std::string config);

    void peripheral_on_eeg (simpleble_peripheral_t peripheral, simpleble_uuid_t service,
        simpleble_uuid_t characteristic, const uint8_t *data, size_t size, size_t channel_num);
    void peripheral_on_ppg (simpleble_peripheral_t peripheral, simpleble_uuid_t service,
        simpleble_uuid_t characteristic, const uint8_t *data, size_t size, size_t ppg_num);
    void peripheral_on_accel (simpleble_peripheral_t peripheral, simpleble_uuid_t service,
        simpleble_uuid_t characteristic, const uint8_t *data, size_t size);
    void peripheral_on_gyro (simpleble_peripheral_t peripheral, simpleble_uuid_t service,
        simpleble_uuid_t characteristic, const uint8_t *data, size_t size);
    void peripheral_on_status (simpleble_peripheral_t peripheral, simpleble_uuid_t service,
        simpleble_uuid_t characteristic, const uint8_t *data, size_t size);
};

#include "brainalive.h"
#include "custom_cast.h"
#include "get_dll_dir.h"
#include "timestamp.h"
#include <string.h>

// info about services and chars
#define START_BYTE 0x0A
#define STOP_BYTE 0x0D

#define BRAINALIVE_WRITE_CHAR "0000fe41-8e22-4541-9d4c-21edae82ed19"
#define BRAINALIVE_NOTIFY_CHAR "0000fe42-8e22-4541-9d4c-21edae82ed19"


static void brainalive_read_notifications (simpleble_peripheral_t peripheral,
    simpleble_uuid_t service, simpleble_uuid_t characteristic, const uint8_t *data, size_t size,
    void *board)
{
    if ((size == BrainAlive::brainalive_handshaking_packet_size) && (data[0] == START_BYTE) &&
        (data[size - 1] == STOP_BYTE) && (data[2] == BrainAlive::brainalive_handshaking_command))
    {
        ((BrainAlive *)(board))->set_internal_gain (data[3]);
        ((BrainAlive *)(board))->set_external_gain (data[4]);
        ((BrainAlive *)(board))->set_ref_Voltage (((data[5] << 8) | data[6]));
    }
    else
    {
        ((BrainAlive *)(board))->read_data (service, characteristic, data, size, 0);
    }
}

BrainAlive::BrainAlive (struct BrainFlowInputParams params)
    : BLELibBoard ((int)BoardIds::BRAINALIVE_BOARD, params)
{
    is_streaming = false;
}

BrainAlive::~BrainAlive ()
{
    skip_logs = true;
    release_session ();
}

int BrainAlive::setup_session ()
{
    int res = (int)BrainFlowExitCodes::STATUS_OK;
    bool control_characteristics_found = false;

    size_t services_count = simpleble_peripheral_services_count (target_peripheral);
    for (size_t i = 0; i < services_count; i++)
    {
        simpleble_service_t service;
        if (simpleble_peripheral_services_get (target_peripheral, i, &service) !=
            SIMPLEBLE_SUCCESS)
        {
            safe_logger (spdlog::level::err, "failed to get service");
            res = (int)BrainFlowExitCodes::BOARD_NOT_READY_ERROR;
        }

        safe_logger (spdlog::level::trace, "found servce {}", service.uuid.value);
        for (size_t j = 0; j < service.characteristic_count; j++)
        {
            safe_logger (spdlog::level::trace, "found characteristic {}",
                service.characteristics[j].uuid.value);

            if (strcmp (service.characteristics[j].uuid.value,
                    BRAINALIVE_WRITE_CHAR) == 0) // Write Characteristics
            {
                write_characteristics = std::pair<simpleble_uuid_t, simpleble_uuid_t> (
                    service.uuid, service.characteristics[j].uuid);
                control_characteristics_found = true;
                safe_logger (spdlog::level::info, "found control characteristic");
            }
            if (strcmp (service.characteristics[j].uuid.value,
                    BRAINALIVE_NOTIFY_CHAR) == 0) // Notification Characteristics
            {
                if (simpleble_peripheral_notify (target_peripheral, service.uuid,
                        service.characteristics[j].uuid, ::brainalive_read_notifications,
                        (void *)this) == SIMPLEBLE_SUCCESS)
                {

                    notified_characteristics = std::pair<simpleble_uuid_t, simpleble_uuid_t> (
                        service.uuid, service.characteristics[j].uuid);
                }
                else
                {
                    safe_logger (spdlog::level::err, "Failed to notify for {} {}",
                        service.uuid.value, service.characteristics[j].uuid.value);
                    res = (int)BrainFlowExitCodes::GENERAL_ERROR;
                }
            }
        }
    }

    if ((res == (int)BrainFlowExitCodes::STATUS_OK) && control_characteristics_found)
    {
        res = config_board ("0a036007000d");
        if (res == (int)BrainFlowExitCodes::STATUS_OK)
            safe_logger (spdlog::level::debug, "Get config command send");
        else
            safe_logger (spdlog::level::debug, "Failed to send get config command");
    }
    else if (res == (int)BrainFlowExitCodes::STATUS_OK)
    {
        res = (int)BrainFlowExitCodes::BOARD_NOT_READY_ERROR;
    }
    return res;
}

int BrainAlive::start_stream (int buffer_size, const char *streamer_params)
{
    if (!initialized)
    {
        return (int)BrainFlowExitCodes::BOARD_NOT_CREATED_ERROR;
    }
    int res = prepare_for_acquisition (buffer_size, streamer_params);

    if (res == (int)BrainFlowExitCodes::STATUS_OK)
    {
        res = config_board ("0a038100000d");
    }
    if (res == (int)BrainFlowExitCodes::STATUS_OK)
    {
        safe_logger (spdlog::level::debug, "Start command Send 250sps");
        is_streaming = true;
    }

    return res;
}

int BrainAlive::stop_stream ()
{
    if (target_peripheral == NULL)
    {
        return (int)BrainFlowExitCodes::BOARD_NOT_CREATED_ERROR;
    }
    int res = (int)BrainFlowExitCodes::STATUS_OK;
    if (is_streaming)
    {
        res = config_board ("0a034000000d");
    }
    else
    {
        res = (int)BrainFlowExitCodes::STREAM_ALREADY_RUN_ERROR;
    }
    is_streaming = false;
    return res;
}

int BrainAlive::release_session ()
{
    if (initialized)
    {
        // repeat it multiple times, failure here may lead to a crash
        for (int i = 0; i < 2; i++)
        {
            stop_stream ();
            // need to wait for notifications to stop triggered before unsubscribing, otherwise
            // macos fails inside simpleble with timeout
#ifdef _WIN32
            Sleep (2000);
#else
            sleep (2);
#endif
            if (simpleble_peripheral_unsubscribe (target_peripheral,
                    notified_characteristics.first,
                    notified_characteristics.second) != SIMPLEBLE_SUCCESS)
            {
                safe_logger (spdlog::level::err, "failed to unsubscribe for {} {}",
                    notified_characteristics.first.value, notified_characteristics.second.value);
            }
            else
            {
                break;
            }
        }
        free_packages ();
        initialized = false;
    }
    release_peripheral ();

    return (int)BrainFlowExitCodes::STATUS_OK;
}

int BrainAlive::config_board (std::string config, std::string &response)
{
    return config_board (config);
}

int BrainAlive::config_board (std::string config)
{
    if (!initialized)
    {
        return (int)BrainFlowExitCodes::BOARD_NOT_CREATED_ERROR;
    }
    // Calculate the number of bytes
    size_t num_bytes = config.length () / 2; // Each byte is represented by 2 hex characters
    std::vector<uint8_t> byte_array (num_bytes);
    // Convert hex string to byte array
    for (size_t i = 0; i < num_bytes; ++i)
    {
        std::string byte_string = config.substr (i * 2, 2); // Get 2 characters for each byte
        byte_array[i] =
            static_cast<unsigned char> (std::stoul (byte_string, nullptr, 16)); // Convert to byte
    }

    if (simpleble_peripheral_write_command (target_peripheral, write_characteristics.first,
            write_characteristics.second, byte_array.data (),
            sizeof (byte_array)) != SIMPLEBLE_SUCCESS)
    {
        safe_logger (spdlog::level::err, "failed to send command {} to device", config.c_str ());
        return (int)BrainFlowExitCodes::BOARD_WRITE_ERROR;
    }
    return (int)BrainFlowExitCodes::STATUS_OK;
}

bool BrainAlive::matches_target_name (const char *identifier)
{
    return strncmp (identifier, "BA_FLEX", 7) == 0;
}

void BrainAlive::read_data (simpleble_uuid_t service, simpleble_uuid_t characteristic,
    const uint8_t *data, size_t size, int channel_num)
{
    if (size == brainalive_packet_size)
    {
        for (int i = 0; i < (int)size; i += brainalive_single_packet_size)
        {

            int num_rows = board_descr["default"]["num_rows"];
            double *package = new double[num_rows];
            for (int i = 0; i < num_rows; i++)
            {
                package[i] = 0.0;
            }
            std::vector<int> eeg_channels = board_descr["default"]["eeg_channels"];
            std::vector<int> accel_channels = board_descr["default"]["accel_channels"];
            std::vector<int> gyro_channels = board_descr["default"]["gyro_channels"];

            package[board_descr["default"]["package_num_channel"].get<int> ()] =
                data[brainalive_packet_index + i];

            for (int j = i + brainalive_eeg_Start_index, k = 0; j < i + brainalive_eeg_end_index;
                 j += 3, k++)
            {
                package[eeg_channels[k]] =
                    (float)(((data[j] << 16 | data[j + 1] << 8 | data[j + 2]) << 8) >> 8) *
                    ((((float)get_ref_voltage () * 1000) /
                        (float)(get_internal_gain () * get_external_gain () * FSR_Value)));
            }

            for (int j = i + brainalive_axl_start_index, k = 0; j < i + brainalive_axl_end_index;
                 j += 2, k++)
            {
                package[accel_channels[k]] = (data[j] << 8) | data[j + 1];
                if (package[accel_channels[k]] > 32767)
                    package[accel_channels[k]] = package[accel_channels[k]] - 65535;
            }
            for (int j = i + brainalive_gyro_start_index, k = 0; j < i + brainalive_gyro_end_index;
                 j += 2, k++)
            {
                package[gyro_channels[k]] = (data[j] << 8) | data[j + 1];
                if (package[gyro_channels[k]] > 32767)
                    package[gyro_channels[k]] = package[gyro_channels[k]] - 65535;
            }
            package[board_descr["default"]["marker_channel"].get<int> ()] =
                data[(brainalive_packet_index + 1) + i];
            package[board_descr["default"]["timestamp_channel"].get<int> ()] = get_timestamp ();

            push_package (&package[0]);
        }
    }
    else
    {
        safe_logger (spdlog::level::warn, "unknown size of BrainAlive Data {}", size);
        return;
    }
}

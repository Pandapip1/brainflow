#pragma once

#include <algorithm>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <vector>

#include "board.h"
#include "board_controller.h"
#include "hw_prober.h"
#include "runtime_dll_loader.h"
#include "simplecble/types.h"


// Shared implementation of HWProber for every BLE-based board. Subclasses only implement
// matches_target_name() (the device-specific name-prefix check); matching against
// params.mac_address / params.serial_number, the connected-peripheral lookup, the scan
// itself, and the discovery wait are all handled once, here, instead of once per board.
class BLELibBoard : public Board, public HWProber
{

private:
#ifndef STATIC_SIMPLEBLE
    static DLLLoader *dll_loader;
    static std::mutex mutex;
#endif
    static simpleble_adapter_t shared_adapter;
    // All BLELibBoard instances currently inside discover(). The scan runs while this list
    // is non-empty; the callback fans out to every entry so concurrent discoveries on
    // different board types share one adapter scan rather than racing over start/stop/callback.
    static std::vector<BLELibBoard *> active_discoverers;
    static std::mutex scan_mutex; // guards active_discoverers and scan start/stop
    static void on_scan_found_callback (
        simpleble_adapter_t adapter, simpleble_peripheral_t peripheral, void *userdata);

protected:
    static bool init_dll_loader ();

    bool initialized;

    // Per-instance discovery state. target_peripheral holds the match after a successful
    // find_already_connected() or discover(); owned by the caller from that point on.
    volatile simpleble_peripheral_t target_peripheral;
    std::mutex discovery_mutex;
    std::condition_variable discovery_cv;

    // Device-specific name-prefix recognition, used only when neither params.mac_address nor
    // params.serial_number is set. Some devices advertise under more than one name; return
    // true for any prefix the device is known to use.
    virtual bool matches_target_name (const char *identifier) = 0;

    // prepare_session() template hooks. Boards override only the ones they need.
    virtual int validate_params ();        // called first; return non-OK to abort early
    virtual int default_timeout () const;  // discovery timeout when params.timeout < 1
    virtual void before_discover ();       // called after ensure_adapter(), before discovery

    // Board-specific post-connect setup (service/characteristic discovery, subscription).
    // Called only after a successful connect_peripheral(). Must NOT set initialized or call
    // release_session() on failure — the prepare_session() template handles both.
    virtual int setup_session () = 0;

    // Connects target_peripheral with up to 3 retries, 1 s apart.
    int connect_peripheral ();

    // Disconnects and releases target_peripheral if non-NULL; sets it to NULL.
    void release_peripheral ();

    bool is_target_peripheral (simpleble_peripheral_t peripheral);

    // common
    void simpleble_free (void *handle);
    // adapter
    size_t simpleble_adapter_get_count (void);
    bool simpleble_adapter_is_bluetooth_enabled (void);
    simpleble_adapter_t simpleble_adapter_get_handle (size_t index);
    void simpleble_adapter_release_handle (simpleble_adapter_t handle);
    simpleble_err_t simpleble_adapter_scan_for (simpleble_adapter_t handle, int timeout_ms);
    simpleble_err_t simpleble_adapter_scan_start (simpleble_adapter_t handle);
    simpleble_err_t simpleble_adapter_scan_stop (simpleble_adapter_t handle);
    simpleble_err_t simpleble_adapter_set_callback_on_scan_start (
        simpleble_adapter_t handle, void (*) (simpleble_adapter_t, void *), void *);
    simpleble_err_t simpleble_adapter_set_callback_on_scan_stop (
        simpleble_adapter_t handle, void (*) (simpleble_adapter_t, void *), void *);
    simpleble_err_t simpleble_adapter_set_callback_on_scan_updated (simpleble_adapter_t handle,
        void (*) (simpleble_adapter_t, simpleble_peripheral_t, void *), void *);
    simpleble_err_t simpleble_adapter_set_callback_on_scan_found (simpleble_adapter_t handle,
        void (*) (simpleble_adapter_t, simpleble_peripheral_t, void *), void *);
    size_t simpleble_adapter_get_connected_peripherals_count (simpleble_adapter_t handle);
    simpleble_peripheral_t simpleble_adapter_get_connected_peripherals_handle (
        simpleble_adapter_t handle, size_t index);
    // Looks for a device the OS/BlueZ already has connected on this adapter and returns the
    // first one for which `matches` returns true, without starting a scan. Peripheral handles
    // that don't match are released internally. Returns NULL if no match was found; the
    // caller owns the returned handle otherwise (release it with
    // simpleble_peripheral_release_handle once done).
    simpleble_peripheral_t find_connected_peripheral (simpleble_adapter_t adapter,
        const std::function<bool (simpleble_peripheral_t)> &matches);
    // peripheral
    char *simpleble_peripheral_address (simpleble_peripheral_t handle);
    char *simpleble_peripheral_identifier (simpleble_peripheral_t handle);
    void simpleble_peripheral_release_handle (simpleble_peripheral_t handle);
    simpleble_err_t simpleble_peripheral_connect (simpleble_peripheral_t handle);
    simpleble_err_t simpleble_peripheral_disconnect (simpleble_peripheral_t handle);
    size_t simpleble_peripheral_services_count (simpleble_peripheral_t handle);
    simpleble_err_t simpleble_peripheral_services_get (
        simpleble_peripheral_t handle, size_t index, simpleble_service_t *services);
    simpleble_err_t simpleble_peripheral_write_request (simpleble_peripheral_t handle,
        simpleble_uuid_t service, simpleble_uuid_t characteristic, uint8_t *data,
        size_t data_length);
    simpleble_err_t simpleble_peripheral_write_command (simpleble_peripheral_t handle,
        simpleble_uuid_t service, simpleble_uuid_t characteristic, uint8_t *data,
        size_t data_length);
    simpleble_err_t simpleble_peripheral_notify (simpleble_peripheral_t handle,
        simpleble_uuid_t service, simpleble_uuid_t characteristic,
        void (*callback) (simpleble_peripheral_t handle, simpleble_uuid_t service,
            simpleble_uuid_t characteristic, const uint8_t *data, size_t data_length,
            void *userdata),
        void *userdata);
    simpleble_err_t simpleble_peripheral_unsubscribe (
        simpleble_peripheral_t handle, simpleble_uuid_t service, simpleble_uuid_t characteristic);
    size_t simpleble_peripheral_manufacturer_data_count (simpleble_peripheral_t handle);
    simpleble_err_t simpleble_peripheral_manufacturer_data_get (simpleble_peripheral_t handle,
        size_t index, simpleble_manufacturer_data_t *manufacturer_data);
    simpleble_err_t simpleble_peripheral_read (simpleble_peripheral_t handle,
        simpleble_uuid_t service, simpleble_uuid_t characteristic, uint8_t **data,
        size_t *data_length);
    simpleble_err_t simpleble_peripheral_is_connected (
        simpleble_peripheral_t handle, bool *connected);

public:
    BLELibBoard (int board_id, struct BrainFlowInputParams params);
    ~BLELibBoard () override;

    // Initialises the BLE DLL and acquires the shared adapter handle (adapter index 0) if
    // not already done. Safe to call multiple times; a no-op after the first success. Must
    // be called before find_already_connected() or discover() can be used. Returns false if
    // the DLL failed to load or no BLE adapter is present.
    static bool ensure_adapter ();

    int prepare_session () override;

    // HWProber
    bool find_already_connected () override;
    bool discover (int timeout_sec) override;
};

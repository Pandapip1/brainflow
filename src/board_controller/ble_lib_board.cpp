#include <chrono>
#include <string.h>
#include <string>
#ifndef _WIN32
#include <unistd.h>
#endif

#include "ble_lib_board.h"

#include "bluetooth_types.h"
#include "get_dll_dir.h"

#ifndef STATIC_SIMPLEBLE
DLLLoader *BLELibBoard::dll_loader = NULL;
std::mutex BLELibBoard::mutex;
#endif
simpleble_adapter_t BLELibBoard::shared_adapter = NULL;
std::vector<BLELibBoard *> BLELibBoard::active_discoverers;
std::mutex BLELibBoard::scan_mutex;
#ifndef STATIC_SIMPLEBLE
#include "simplecble/adapter.h"
#include "simplecble/peripheral.h"
#include "simplecble/simplecble.h"
#include "simplecble/utils.h"
#endif

void BLELibBoard::on_scan_found_callback (
    simpleble_adapter_t /*adapter*/, simpleble_peripheral_t peripheral, void * /*userdata*/)
{
    std::lock_guard<std::mutex> lk (scan_mutex);
    for (auto *board : active_discoverers)
    {
        if (board->is_target_peripheral (peripheral))
        {
            {
                std::lock_guard<std::mutex> board_lk (board->discovery_mutex);
                board->target_peripheral = peripheral;
            }
            board->discovery_cv.notify_one ();
            return; // peripheral ownership transferred to the matched board
        }
    }
    simpleble_peripheral_release_handle (peripheral);
}

BLELibBoard::BLELibBoard (int board_id, struct BrainFlowInputParams params)
    : Board (board_id, params)
{
    initialized = false;
    target_peripheral = NULL;
}

int BLELibBoard::validate_params ()
{
    return (int)BrainFlowExitCodes::STATUS_OK;
}

int BLELibBoard::default_timeout () const
{
    return 5;
}

void BLELibBoard::before_discover ()
{
}

int BLELibBoard::prepare_session ()
{
    if (initialized)
    {
        safe_logger (spdlog::level::info, "Session is already prepared");
        return (int)BrainFlowExitCodes::STATUS_OK;
    }

    int res = validate_params ();
    if (res != (int)BrainFlowExitCodes::STATUS_OK)
        return res;

    if (params.timeout < 1)
        params.timeout = default_timeout ();
    safe_logger (spdlog::level::info, "Use timeout for discovery: {}", params.timeout);

    if (!ensure_adapter ())
        return (int)BrainFlowExitCodes::UNABLE_TO_OPEN_PORT_ERROR;

    before_discover ();

    if (!find_already_connected () && !discover (params.timeout))
    {
// https://github.com/OpenBluetoothToolbox/SimpleBLE/issues/26#issuecomment-955606799
#ifdef __linux__
        usleep (1000000);
#endif
        return (int)BrainFlowExitCodes::BOARD_NOT_READY_ERROR;
    }

    res = connect_peripheral ();
    if (res != (int)BrainFlowExitCodes::STATUS_OK)
        return res;

    res = setup_session ();
    if (res == (int)BrainFlowExitCodes::STATUS_OK)
        initialized = true;
    else
        release_session ();
    return res;
}

int BLELibBoard::connect_peripheral ()
{
    for (int i = 0; i < 3; i++)
    {
        if (simpleble_peripheral_connect (target_peripheral) == SIMPLEBLE_SUCCESS)
        {
            safe_logger (spdlog::level::info, "Connected to BLE device");
            return (int)BrainFlowExitCodes::STATUS_OK;
        }
        safe_logger (spdlog::level::warn, "Failed to connect to BLE device: {}/3", i);
#ifdef _WIN32
        Sleep (1000);
#else
        sleep (1);
#endif
    }
    return (int)BrainFlowExitCodes::BOARD_NOT_READY_ERROR;
}

void BLELibBoard::release_peripheral ()
{
    if (target_peripheral != NULL)
    {
        bool is_connected = false;
        if (simpleble_peripheral_is_connected (target_peripheral, &is_connected) ==
            SIMPLEBLE_SUCCESS)
        {
            if (is_connected)
                simpleble_peripheral_disconnect (target_peripheral);
        }
        simpleble_peripheral_release_handle (target_peripheral);
        target_peripheral = NULL;
    }
}

bool BLELibBoard::ensure_adapter ()
{
    if (!init_dll_loader ())
    {
        Board::board_logger->error ("Failed to init dll_loader");
        return false;
    }
    if (shared_adapter != NULL)
        return true;
    size_t count = simpleble_adapter_get_count ();
    if (count == 0)
    {
        Board::board_logger->error ("No BLE adapters found");
        return false;
    }
    Board::board_logger->info ("found {} BLE adapter(s)", count);
    shared_adapter = simpleble_adapter_get_handle (0);
    if (shared_adapter == NULL)
    {
        Board::board_logger->error ("Adapter is NULL");
        return false;
    }
    return true;
}

BLELibBoard::~BLELibBoard ()
{
}

bool BLELibBoard::init_dll_loader ()
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        char blelib_dir[1024];
        bool res = get_dll_path (blelib_dir);
        std::string blelib_path = "";
#ifdef _WIN32
        std::string lib_name;
        if (sizeof (void *) == 4)
        {
            lib_name = "simpleble-c32.dll";
        }
        else
        {
            lib_name = "simpleble-c.dll";
        }
#elif defined(__APPLE__)
        std::string lib_name = "libsimpleble-c.dylib";
#else
        std::string lib_name = "libsimpleble-c.so";
#endif
        if (res)
        {
            blelib_path = std::string (blelib_dir) + lib_name;
        }
        else
        {
            blelib_path = lib_name;
        }

        Board::board_logger->debug ("use dyn lib: {}", blelib_path.c_str ());
        BLELibBoard::dll_loader = new DLLLoader (blelib_path.c_str ());
        if (!BLELibBoard::dll_loader->load_library ())
        {
            Board::board_logger->error ("failed to load lib");
            delete BLELibBoard::dll_loader;
            BLELibBoard::dll_loader = NULL;
        }
    }

    return BLELibBoard::dll_loader != NULL;
#else
    return true;
#endif
}

void BLELibBoard::simpleble_free (void *handle)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return;
    }
    void (*func) (void *) =
        (void (*) (void *))BLELibBoard::dll_loader->get_address ("simpleble_free");
    if (func == NULL)
    {
        safe_logger (spdlog::level::err, "failed to get function address for simpleble_free");
        return;
    }

    return func (handle);
#else
    return ::simpleble_free (handle);
#endif
}

size_t BLELibBoard::simpleble_adapter_get_count (void)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return 0;
    }
    size_t (*func) (void) =
        (size_t (*) (void))BLELibBoard::dll_loader->get_address ("simpleble_adapter_get_count");
    if (func == NULL)
    {
        safe_logger (
            spdlog::level::err, "failed to get function address for simpleble_adapter_get_count");
        return 0;
    }

    return func ();
#else
    return ::simpleble_adapter_get_count ();
#endif
}

bool BLELibBoard::simpleble_adapter_is_bluetooth_enabled (void)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return false;
    }
    bool (*func) (void) = (bool (*) (void))BLELibBoard::dll_loader->get_address (
        "simpleble_adapter_is_bluetooth_enabled");
    if (func == NULL)
    {
        safe_logger (spdlog::level::err,
            "failed to get function address for simpleble_adapter_is_bluetooth_enabled");
        return false;
    }

    return func ();
#else
    return ::simpleble_adapter_is_bluetooth_enabled ();
#endif
}

simpleble_adapter_t BLELibBoard::simpleble_adapter_get_handle (size_t index)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return NULL;
    }
    simpleble_adapter_t (*func) (size_t) = (simpleble_adapter_t (*) (
        size_t))BLELibBoard::dll_loader->get_address ("simpleble_adapter_get_handle");
    if (func == NULL)
    {
        safe_logger (
            spdlog::level::err, "failed to get function address for simpleble_adapter_get_handle");
        return NULL;
    }

    return func (index);
#else
    return ::simpleble_adapter_get_handle (index);
#endif
}

void BLELibBoard::simpleble_adapter_release_handle (simpleble_adapter_t handle)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return;
    }
    void (*func) (simpleble_adapter_t) =
        (void (*) (simpleble_adapter_t))BLELibBoard::dll_loader->get_address (
            "simpleble_adapter_release_handle");
    if (func == NULL)
    {
        safe_logger (spdlog::level::err,
            "failed to get function address for simpleble_adapter_release_handle");
        return;
    }

    func (handle);
#else
    return ::simpleble_adapter_release_handle (handle);
#endif
}

simpleble_err_t BLELibBoard::simpleble_adapter_scan_for (simpleble_adapter_t handle, int timeout_ms)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return SIMPLEBLE_FAILURE;
    }
    simpleble_err_t (*func) (simpleble_adapter_t, int) = (simpleble_err_t (*) (simpleble_adapter_t,
        int))BLELibBoard::dll_loader->get_address ("simpleble_adapter_scan_for");
    if (func == NULL)
    {
        safe_logger (
            spdlog::level::err, "failed to get function address for simpleble_adapter_scan_for");
        return SIMPLEBLE_FAILURE;
    }

    return func (handle, timeout_ms);
#else
    return ::simpleble_adapter_scan_for (handle, timeout_ms);
#endif
}

simpleble_err_t BLELibBoard::simpleble_adapter_scan_start (simpleble_adapter_t handle)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return SIMPLEBLE_FAILURE;
    }
    simpleble_err_t (*func) (simpleble_adapter_t) = (simpleble_err_t (*) (
        simpleble_adapter_t))BLELibBoard::dll_loader->get_address ("simpleble_adapter_scan_start");
    if (func == NULL)
    {
        safe_logger (
            spdlog::level::err, "failed to get function address for simpleble_adapter_scan_start");
        return SIMPLEBLE_FAILURE;
    }

    return func (handle);
#else
    return ::simpleble_adapter_scan_start (handle);
#endif
}

simpleble_err_t BLELibBoard::simpleble_adapter_scan_stop (simpleble_adapter_t handle)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return SIMPLEBLE_FAILURE;
    }
    simpleble_err_t (*func) (simpleble_adapter_t) = (simpleble_err_t (*) (
        simpleble_adapter_t))BLELibBoard::dll_loader->get_address ("simpleble_adapter_scan_stop");
    if (func == NULL)
    {
        safe_logger (
            spdlog::level::err, "failed to get function address for simpleble_adapter_scan_stop");
        return SIMPLEBLE_FAILURE;
    }

    return func (handle);
#else
    return ::simpleble_adapter_scan_stop (handle);
#endif
}

simpleble_err_t BLELibBoard::simpleble_adapter_set_callback_on_scan_updated (
    simpleble_adapter_t handle,
    void (*callback) (
        simpleble_adapter_t adapter, simpleble_peripheral_t peripheral, void *userdata),
    void *userdata)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return SIMPLEBLE_FAILURE;
    }
    simpleble_err_t (*func) (simpleble_adapter_t,
        void (*) (simpleble_adapter_t, simpleble_peripheral_t, void *), void *) =
        (simpleble_err_t (*) (simpleble_adapter_t,
            void (*) (simpleble_adapter_t, simpleble_peripheral_t, void *), void *))
            BLELibBoard::dll_loader->get_address ("simpleble_adapter_set_callback_on_scan_updated");
    if (func == NULL)
    {
        safe_logger (spdlog::level::err,
            "failed to get function address for simpleble_adapter_set_callback_on_scan_updated");
        return SIMPLEBLE_FAILURE;
    }

    return func (handle, callback, userdata);
#else
    return ::simpleble_adapter_set_callback_on_scan_updated (handle, callback, userdata);
#endif
}

simpleble_err_t BLELibBoard::simpleble_adapter_set_callback_on_scan_start (
    simpleble_adapter_t handle, void (*callback) (simpleble_adapter_t, void *), void *userdata)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return SIMPLEBLE_FAILURE;
    }
    simpleble_err_t (*func) (simpleble_adapter_t, void (*) (simpleble_adapter_t, void *), void *) =
        (simpleble_err_t (*) (simpleble_adapter_t, void (*) (simpleble_adapter_t, void *), void *))
            BLELibBoard::dll_loader->get_address ("simpleble_adapter_set_callback_on_scan_start");
    if (func == NULL)
    {
        safe_logger (spdlog::level::err,
            "failed to get function address for simpleble_adapter_set_callback_on_scan_start");
        return SIMPLEBLE_FAILURE;
    }

    return func (handle, callback, userdata);
#else
    return ::simpleble_adapter_set_callback_on_scan_start (handle, callback, userdata);
#endif
}

simpleble_err_t BLELibBoard::simpleble_adapter_set_callback_on_scan_stop (
    simpleble_adapter_t handle, void (*callback) (simpleble_adapter_t, void *), void *userdata)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return SIMPLEBLE_FAILURE;
    }
    simpleble_err_t (*func) (simpleble_adapter_t, void (*) (simpleble_adapter_t, void *), void *) =
        (simpleble_err_t (*) (simpleble_adapter_t, void (*) (simpleble_adapter_t, void *), void *))
            BLELibBoard::dll_loader->get_address ("simpleble_adapter_set_callback_on_scan_stop");
    if (func == NULL)
    {
        safe_logger (spdlog::level::err,
            "failed to get function address for simpleble_adapter_set_callback_on_scan_stop");
        return SIMPLEBLE_FAILURE;
    }

    return func (handle, callback, userdata);
#else
    return ::simpleble_adapter_set_callback_on_scan_stop (handle, callback, userdata);
#endif
}

simpleble_err_t BLELibBoard::simpleble_adapter_set_callback_on_scan_found (
    simpleble_adapter_t handle,
    void (*callback) (
        simpleble_adapter_t adapter, simpleble_peripheral_t peripheral, void *userdata),
    void *userdata)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return SIMPLEBLE_FAILURE;
    }
    simpleble_err_t (*func) (simpleble_adapter_t,
        void (*) (simpleble_adapter_t, simpleble_peripheral_t, void *), void *) =
        (simpleble_err_t (*) (simpleble_adapter_t,
            void (*) (simpleble_adapter_t, simpleble_peripheral_t, void *), void *))
            BLELibBoard::dll_loader->get_address ("simpleble_adapter_set_callback_on_scan_found");
    if (func == NULL)
    {
        safe_logger (spdlog::level::err,
            "failed to get function address for simpleble_adapter_set_callback_on_scan_found");
        return SIMPLEBLE_FAILURE;
    }

    return func (handle, callback, userdata);
#else
    return ::simpleble_adapter_set_callback_on_scan_found (handle, callback, userdata);
#endif
}

size_t BLELibBoard::simpleble_adapter_get_connected_peripherals_count (simpleble_adapter_t handle)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return 0;
    }
    size_t (*func) (simpleble_adapter_t) = (size_t (*) (simpleble_adapter_t))
        BLELibBoard::dll_loader->get_address ("simpleble_adapter_get_connected_peripherals_count");
    if (func == NULL)
    {
        safe_logger (spdlog::level::err,
            "failed to get function address for "
            "simpleble_adapter_get_connected_peripherals_count");
        return 0;
    }

    return func (handle);
#else
    return ::simpleble_adapter_get_connected_peripherals_count (handle);
#endif
}

simpleble_peripheral_t BLELibBoard::simpleble_adapter_get_connected_peripherals_handle (
    simpleble_adapter_t handle, size_t index)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return NULL;
    }
    simpleble_peripheral_t (*func) (simpleble_adapter_t, size_t) =
        (simpleble_peripheral_t (*) (simpleble_adapter_t, size_t))
            BLELibBoard::dll_loader->get_address (
                "simpleble_adapter_get_connected_peripherals_handle");
    if (func == NULL)
    {
        safe_logger (spdlog::level::err,
            "failed to get function address for "
            "simpleble_adapter_get_connected_peripherals_handle");
        return NULL;
    }

    return func (handle, index);
#else
    return ::simpleble_adapter_get_connected_peripherals_handle (handle, index);
#endif
}

simpleble_peripheral_t BLELibBoard::find_connected_peripheral (simpleble_adapter_t adapter,
    const std::function<bool (simpleble_peripheral_t)> &matches)
{
    size_t connected_count = simpleble_adapter_get_connected_peripherals_count (adapter);
    for (size_t i = 0; i < connected_count; i++)
    {
        simpleble_peripheral_t candidate =
            simpleble_adapter_get_connected_peripherals_handle (adapter, i);
        if (candidate == NULL)
        {
            continue;
        }
        if (matches (candidate))
        {
            return candidate;
        }
        simpleble_peripheral_release_handle (candidate);
    }
    return NULL;
}

bool BLELibBoard::is_target_peripheral (simpleble_peripheral_t peripheral)
{
    char *peripheral_identified = simpleble_peripheral_identifier (peripheral);
    char *peripheral_address = simpleble_peripheral_address (peripheral);
    bool found = false;
    if (!params.mac_address.empty ())
    {
        found = (strcmp (peripheral_address, params.mac_address.c_str ()) == 0);
    }
    else if (!params.serial_number.empty ())
    {
        found = (strcmp (peripheral_identified, params.serial_number.c_str ()) == 0);
    }
    else
    {
        found = matches_target_name (peripheral_identified);
    }

    safe_logger (spdlog::level::trace, "address {}", peripheral_address);
    simpleble_free (peripheral_address);
    safe_logger (spdlog::level::trace, "identifier {}", peripheral_identified);
    simpleble_free (peripheral_identified);

    return found;
}


bool BLELibBoard::find_already_connected ()
{
    target_peripheral = find_connected_peripheral (
        shared_adapter, [this] (simpleble_peripheral_t p) { return is_target_peripheral (p); });
    if (target_peripheral != NULL)
    {
        safe_logger (spdlog::level::info, "Found already connected device, skipping BLE scan");
    }
    return target_peripheral != NULL;
}

bool BLELibBoard::discover (int timeout_sec)
{
    target_peripheral = NULL;

    {
        std::lock_guard<std::mutex> lk (scan_mutex);
        active_discoverers.push_back (this);
        if (active_discoverers.size () == 1)
        {
            // First caller: arm the shared callback and start the adapter scan. Subsequent
            // concurrent callers join the already-running scan without restarting it.
            simpleble_adapter_set_callback_on_scan_found (
                shared_adapter, BLELibBoard::on_scan_found_callback, nullptr);
            if (!simpleble_adapter_is_bluetooth_enabled ())
            {
                safe_logger (spdlog::level::warn, "Probably bluetooth is disabled.");
                // dont throw an exception because of this
                // https://github.com/OpenBluetoothToolbox/SimpleBLE/issues/115
            }
            safe_logger (spdlog::level::trace, "Scan started");
            simpleble_adapter_scan_start (shared_adapter);
        }
    }

    std::unique_lock<std::mutex> lk (discovery_mutex);
    auto sec = std::chrono::seconds (1);
    bool found = discovery_cv.wait_for (
        lk, timeout_sec * sec, [this] { return this->target_peripheral != NULL; });
    lk.unlock ();

    {
        std::lock_guard<std::mutex> lk (scan_mutex);
        active_discoverers.erase (
            std::remove (active_discoverers.begin (), active_discoverers.end (), this),
            active_discoverers.end ());
        if (active_discoverers.empty ())
        {
            // Last caller finished: clear the callback before stopping the scan (avoids the
            // macOS race where a scan-found event re-enters BrainFlow during scan teardown).
            simpleble_adapter_set_callback_on_scan_found (shared_adapter, NULL, NULL);
            simpleble_adapter_scan_stop (shared_adapter);
            safe_logger (spdlog::level::trace, "Scan stopped");
        }
    }

    if (found)
    {
        safe_logger (spdlog::level::info, "Found target device");
    }
    else
    {
        safe_logger (spdlog::level::err, "Failed to find target device");
    }
    return found;
}

char *BLELibBoard::simpleble_peripheral_address (simpleble_peripheral_t handle)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return NULL;
    }
    char *(*func) (simpleble_peripheral_t) =
        (char *(*)(simpleble_peripheral_t))BLELibBoard::dll_loader->get_address (
            "simpleble_peripheral_address");
    if (func == NULL)
    {
        safe_logger (
            spdlog::level::err, "failed to get function address for simpleble_peripheral_address");
        return NULL;
    }

    return func (handle);
#else
    return ::simpleble_peripheral_address (handle);
#endif
}

char *BLELibBoard::simpleble_peripheral_identifier (simpleble_peripheral_t handle)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return NULL;
    }
    char *(*func) (simpleble_peripheral_t) =
        (char *(*)(simpleble_peripheral_t))BLELibBoard::dll_loader->get_address (
            "simpleble_peripheral_identifier");
    if (func == NULL)
    {
        safe_logger (spdlog::level::err,
            "failed to get function address for simpleble_peripheral_identifier");
        return NULL;
    }

    return func (handle);
#else
    return ::simpleble_peripheral_identifier (handle);
#endif
}

simpleble_err_t BLELibBoard::simpleble_peripheral_connect (simpleble_peripheral_t handle)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return SIMPLEBLE_FAILURE;
    }
    simpleble_err_t (*func) (simpleble_peripheral_t) =
        (simpleble_err_t (*) (simpleble_peripheral_t))BLELibBoard::dll_loader->get_address (
            "simpleble_peripheral_connect");
    if (func == NULL)
    {
        safe_logger (
            spdlog::level::err, "failed to get function address for simpleble_peripheral_connect");
        return SIMPLEBLE_FAILURE;
    }

    return func (handle);
#else
    return ::simpleble_peripheral_connect (handle);
#endif
}

simpleble_err_t BLELibBoard::simpleble_peripheral_disconnect (simpleble_peripheral_t handle)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return SIMPLEBLE_FAILURE;
    }
    simpleble_err_t (*func) (simpleble_peripheral_t) =
        (simpleble_err_t (*) (simpleble_peripheral_t))BLELibBoard::dll_loader->get_address (
            "simpleble_peripheral_disconnect");
    if (func == NULL)
    {
        safe_logger (spdlog::level::err,
            "failed to get function address for simpleble_peripheral_disconnect");
        return SIMPLEBLE_FAILURE;
    }

    return func (handle);
#else
    return ::simpleble_peripheral_disconnect (handle);
#endif
}

void BLELibBoard::simpleble_peripheral_release_handle (simpleble_peripheral_t handle)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return;
    }
    void (*func) (simpleble_peripheral_t) =
        (void (*) (simpleble_peripheral_t))BLELibBoard::dll_loader->get_address (
            "simpleble_peripheral_release_handle");
    if (func == NULL)
    {
        safe_logger (spdlog::level::err,
            "failed to get function address for simpleble_peripheral_release_handle");
        return;
    }

    func (handle);
#else
    return ::simpleble_peripheral_release_handle (handle);
#endif
}

size_t BLELibBoard::simpleble_peripheral_services_count (simpleble_peripheral_t handle)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return 0;
    }
    size_t (*func) (simpleble_peripheral_t) =
        (size_t (*) (simpleble_peripheral_t))BLELibBoard::dll_loader->get_address (
            "simpleble_peripheral_services_count");
    if (func == NULL)
    {
        safe_logger (spdlog::level::err,
            "failed to get function address for simpleble_peripheral_services_count");
        return 0;
    }

    return func (handle);
#else
    return ::simpleble_peripheral_services_count (handle);
#endif
}

simpleble_err_t BLELibBoard::simpleble_peripheral_services_get (
    simpleble_peripheral_t handle, size_t index, simpleble_service_t *services)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return SIMPLEBLE_FAILURE;
    }
    simpleble_err_t (*func) (simpleble_peripheral_t, size_t, simpleble_service_t *) =
        (simpleble_err_t (*) (simpleble_peripheral_t, size_t, simpleble_service_t *))
            BLELibBoard::dll_loader->get_address ("simpleble_peripheral_services_get");
    if (func == NULL)
    {
        safe_logger (spdlog::level::err,
            "failed to get function address for simpleble_peripheral_services_get");
        return SIMPLEBLE_FAILURE;
    }

    return func (handle, index, services);
#else
    return ::simpleble_peripheral_services_get (handle, index, services);
#endif
}


simpleble_err_t BLELibBoard::simpleble_peripheral_write_request (simpleble_peripheral_t handle,
    simpleble_uuid_t service, simpleble_uuid_t characteristic, uint8_t *data, size_t data_length)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return SIMPLEBLE_FAILURE;
    }
    simpleble_err_t (*func) (
        simpleble_peripheral_t, simpleble_uuid_t, simpleble_uuid_t, uint8_t *, size_t) =
        (simpleble_err_t (*) (simpleble_peripheral_t, simpleble_uuid_t, simpleble_uuid_t, uint8_t *,
            size_t))BLELibBoard::dll_loader->get_address ("simpleble_peripheral_write_request");
    if (func == NULL)
    {
        safe_logger (spdlog::level::err,
            "failed to get function address for simpleble_peripheral_write_request");
        return SIMPLEBLE_FAILURE;
    }

    return func (handle, service, characteristic, data, data_length);
#else
    return ::simpleble_peripheral_write_request (
        handle, service, characteristic, data, data_length);
#endif
}

simpleble_err_t BLELibBoard::simpleble_peripheral_write_command (simpleble_peripheral_t handle,
    simpleble_uuid_t service, simpleble_uuid_t characteristic, uint8_t *data, size_t data_length)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return SIMPLEBLE_FAILURE;
    }
    simpleble_err_t (*func) (
        simpleble_peripheral_t, simpleble_uuid_t, simpleble_uuid_t, uint8_t *, size_t) =
        (simpleble_err_t (*) (simpleble_peripheral_t, simpleble_uuid_t, simpleble_uuid_t, uint8_t *,
            size_t))BLELibBoard::dll_loader->get_address ("simpleble_peripheral_write_command");
    if (func == NULL)
    {
        safe_logger (spdlog::level::err,
            "failed to get function address for simpleble_peripheral_write_command");
        return SIMPLEBLE_FAILURE;
    }

    return func (handle, service, characteristic, data, data_length);
#else
    return ::simpleble_peripheral_write_command (
        handle, service, characteristic, data, data_length);
#endif
}

simpleble_err_t BLELibBoard::simpleble_peripheral_notify (simpleble_peripheral_t handle,
    simpleble_uuid_t service, simpleble_uuid_t characteristic,
    void (*callback) (simpleble_peripheral_t handle, simpleble_uuid_t service,
        simpleble_uuid_t characteristic, const uint8_t *data, size_t data_length, void *userdata),
    void *userdata)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return SIMPLEBLE_FAILURE;
    }
    simpleble_err_t (*func) (simpleble_peripheral_t, simpleble_uuid_t, simpleble_uuid_t,
        void (*) (simpleble_peripheral_t, simpleble_uuid_t, simpleble_uuid_t, const uint8_t *,
            size_t, void *),
        void *) = (simpleble_err_t (*) (simpleble_peripheral_t, simpleble_uuid_t, simpleble_uuid_t,
        void (*) (simpleble_peripheral_t, simpleble_uuid_t, simpleble_uuid_t, const uint8_t *,
            size_t, void *),
        void *))BLELibBoard::dll_loader->get_address ("simpleble_peripheral_notify");
    if (func == NULL)
    {
        safe_logger (
            spdlog::level::err, "failed to get function address for simpleble_peripheral_notify");
        return SIMPLEBLE_FAILURE;
    }

    return func (handle, service, characteristic, callback, userdata);
#else
    return ::simpleble_peripheral_notify (handle, service, characteristic, callback, userdata);
#endif
}

simpleble_err_t BLELibBoard::simpleble_peripheral_unsubscribe (
    simpleble_peripheral_t handle, simpleble_uuid_t service, simpleble_uuid_t characteristic)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return SIMPLEBLE_FAILURE;
    }
    simpleble_err_t (*func) (simpleble_peripheral_t, simpleble_uuid_t,
        simpleble_uuid_t) = (simpleble_err_t (*) (simpleble_peripheral_t, simpleble_uuid_t,
        simpleble_uuid_t))BLELibBoard::dll_loader->get_address ("simpleble_peripheral_unsubscribe");
    if (func == NULL)
    {
        safe_logger (spdlog::level::err,
            "failed to get function address for simpleble_peripheral_unsubscribe");
        return SIMPLEBLE_FAILURE;
    }

    return func (handle, service, characteristic);
#else
    return ::simpleble_peripheral_unsubscribe (handle, service, characteristic);
#endif
}

size_t BLELibBoard::simpleble_peripheral_manufacturer_data_count (simpleble_peripheral_t handle)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return 0;
    }
    size_t (*func) (simpleble_peripheral_t) =
        (size_t (*) (simpleble_peripheral_t))BLELibBoard::dll_loader->get_address (
            "simpleble_peripheral_manufacturer_data_count");
    if (func == NULL)
    {
        safe_logger (spdlog::level::err,
            "failed to get function address for simpleble_peripheral_manufacturer_data_count");
        return 0;
    }

    return func (handle);
#else
    return ::simpleble_peripheral_manufacturer_data_count (handle);
#endif
}

simpleble_err_t BLELibBoard::simpleble_peripheral_manufacturer_data_get (
    simpleble_peripheral_t handle, size_t index, simpleble_manufacturer_data_t *manufacturer_data)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return SIMPLEBLE_FAILURE;
    }
    simpleble_err_t (*func) (simpleble_peripheral_t, size_t, simpleble_manufacturer_data_t *) =
        (simpleble_err_t (*) (simpleble_peripheral_t, size_t, simpleble_manufacturer_data_t *))
            BLELibBoard::dll_loader->get_address ("simpleble_peripheral_manufacturer_data_get");
    if (func == NULL)
    {
        safe_logger (spdlog::level::err,
            "failed to get function address for simpleble_peripheral_manufacturer_data_get");
        return SIMPLEBLE_FAILURE;
    }

    return func (handle, index, manufacturer_data);
#else
    return ::simpleble_peripheral_manufacturer_data_get (handle, index, manufacturer_data);
#endif
}

simpleble_err_t BLELibBoard::simpleble_peripheral_read (simpleble_peripheral_t handle,
    simpleble_uuid_t service, simpleble_uuid_t characteristic, uint8_t **data, size_t *data_length)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return SIMPLEBLE_FAILURE;
    }
    simpleble_err_t (*func) (simpleble_peripheral_t, simpleble_uuid_t, simpleble_uuid_t, uint8_t **,
        size_t *) = (simpleble_err_t (*) (simpleble_peripheral_t, simpleble_uuid_t,
        simpleble_uuid_t, uint8_t **,
        size_t *))BLELibBoard::dll_loader->get_address ("simpleble_peripheral_read");
    if (func == NULL)
    {
        safe_logger (
            spdlog::level::err, "failed to get function address for simpleble_peripheral_read");
        return SIMPLEBLE_FAILURE;
    }

    return func (handle, service, characteristic, data, data_length);
#else
    return ::simpleble_peripheral_read (handle, service, characteristic, data, data_length);
#endif
}

simpleble_err_t BLELibBoard::simpleble_peripheral_is_connected (
    simpleble_peripheral_t handle, bool *connected)
{
#ifndef STATIC_SIMPLEBLE
    std::lock_guard<std::mutex> lock (BLELibBoard::mutex);
    if (BLELibBoard::dll_loader == NULL)
    {
        safe_logger (spdlog::level::err, "BLELibBoard::dll_loader is not initialized");
        return SIMPLEBLE_FAILURE;
    }
    simpleble_err_t (*func) (simpleble_peripheral_t, bool *) =
        (simpleble_err_t (*) (simpleble_peripheral_t, bool *))BLELibBoard::dll_loader->get_address (
            "simpleble_peripheral_is_connected");
    if (func == NULL)
    {
        safe_logger (spdlog::level::err,
            "failed to get function address for simpleble_peripheral_is_connected");
        return SIMPLEBLE_FAILURE;
    }

    return func (handle, connected);
#else
    return ::simpleble_peripheral_is_connected (handle, connected);
#endif
}

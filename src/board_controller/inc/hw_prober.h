#pragma once

// Generic two-phase device-discovery contract for any transport that needs to find a piece
// of hardware before BrainFlow can talk to it (BLE, classic Bluetooth, Wi-Fi broadcast, ...).
//
// The two phases exist because, for most of these transports, a device that's already
// attached/connected elsewhere often can't be found by active probing alone: a BLE peripheral
// stops advertising once it has a connection, a Wi-Fi board already bound to a session may not
// answer a fresh broadcast, etc. Implementations - and the prepare_session() methods that use
// them - should always give find_already_connected() a chance before falling back to discover().
//
// What "found" actually means (a connect-ready handle, a MAC address, an IP, ...) is
// transport-specific and is left to the implementing class to expose via its own accessor;
// this interface only reports whether a match was found.
class HWProber
{
public:
    virtual ~HWProber () = default;

    // Looks for a device matching the board's target criteria (params.mac_address,
    // params.serial_number, or a device-specific name/identity) that the transport already
    // considers connected/attached, without actively probing for it. Returns true if found.
    virtual bool find_already_connected () = 0;

    // Actively probes for a matching device for up to timeout_sec seconds (radio scan,
    // broadcast, inquiry, ...). Returns true if found.
    virtual bool discover (int timeout_sec) = 0;
};

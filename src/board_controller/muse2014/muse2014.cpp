#include <array>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "muse2014.h"

#include "timestamp.h"

// ---------------------------------------------------------------------------
// Protocol notes
// ---------------------------------------------------------------------------
// The original 2014 Muse (MU-01) streams binary EEG data over a classic
// Bluetooth RFCOMM serial connection (no GATT/BLE involved at all - confirmed
// independently via direct SDP inspection of a real unit: Service Class ID
// "Serial Port" (0x1101) over RFCOMM channel 1, alongside InteraXon and
// "RN-iAP"/Roving-Networks vendor records).
//
// Framing, reverse-engineered from David Vivancos' "MuseDotNet" (2014,
// BluetoothMuse.cs / MuseDirect.cs - a from-scratch, non-InteraXon
// implementation of the wire protocol), corroborated on the command-byte
// level by an independent 2016 InteraXon forum post describing a bare
// HC-05+Arduino client:
//
//   sync marker:  FF FF AA 55
//   packet type:  E0  -> "reference" packet, 5 bytes follow:
//                        4 channels x 10 bits each (TP9, AF7, AF8, TP10),
//                        packed MSB-first across the 5 bytes.
//                 C0  -> "delta" packet, follows with:
//                        5 bytes: per-channel header, 10 bits each, split into
//                                 4 bits (quantization step selector) + 6 bits
//                                 (Golomb/Rice "median" parameter)
//                        2 bytes: big-endian bit-length of the payload to follow
//                        N bytes: payload, N = ceil(bit-length / 8)
//
//                        Payload is 4 per-channel blocks of 16 delta-coded
//                        samples each, Golomb/Rice-style: unary-coded quotient
//                        (escapes to a binary-coded value if the run reaches
//                        15 ones), truncated-binary remainder sized to the
//                        channel's median, then a sign bit. Each decoded delta
//                        accumulates onto a running per-channel "carry" value
//                        that persists across packets (seeded by the most
//                        recent E0 reference frame).
//
// IMPORTANT: in MuseDotNet's ParsePacket, the persistent carry array and the
// freshly-decoded per-packet quantization/median arrays are reversed in place
// (Array.Reverse) before use, and the carry array is mutated by that reversal
// across calls (C# arrays are reference types). Tracing this through two
// consecutive packets shows the carry array's channel-order flips on every
// call, while the freshly-decoded quant/median arrays are *always* reversed
// from a fixed physical order - so the two are only correctly aligned on
// every other packet, and mislabel channels on the others. This
// implementation avoids that bug by keeping `carry` in a fixed physical-
// channel index permanently (never reversed across calls) and only mapping
// "bitstream block order" (which genuinely is the reverse of physical
// channel order, confirmed by tracing a single packet in isolation) to
// physical channel index transiently, within a single packet's decode.
//
// Command bytes sent on connect (from MuseDotNet, preset/params differ
// slightly but the single-ASCII-letter-plus-arguments command style matches
// the independently-found forum post):
//   "v 2\r"  - request protocol v2 (enables binary streaming mode)
//   "% 10\r" - select preset 10
//   "s 5\r"  - start streaming
//   "k\r"    - keepalive, sent periodically (every ~8s) or the device
//              disconnects
//
// NONE of this is from an official InteraXon spec (InteraXon's own protocol
// docs no longer exist anywhere this implementation could fetch from). It is
// a best-effort port of independently reverse-engineered, working reference
// code, and has not yet been validated end-to-end against a live device by
// the author of this file. Treat decoded sample values - especially the
// microvolt scale factor in push_batch() - as provisional.
// ---------------------------------------------------------------------------

namespace
{

// Reads bits MSB-first across a byte buffer. Reading past the end of the
// buffer returns 0 bits rather than touching invalid memory - this guarantees
// termination of the unary-code scan below even on truncated/corrupt input,
// at the cost of (only in that failure case) decoding garbage instead of
// crashing.
class BitReader
{
public:
    BitReader (const unsigned char *data, size_t num_bytes)
        : data_ (data), total_bits_ (num_bytes * 8), pos_ (0)
    {
    }

    size_t position () const
    {
        return pos_;
    }

    unsigned int peek_bits_at (size_t start, int n) const
    {
        unsigned int val = 0;
        for (int i = 0; i < n; i++)
        {
            size_t bit_index = start + (size_t)i;
            unsigned int bit = 0;
            if (bit_index < total_bits_)
            {
                size_t byte_idx = bit_index / 8;
                int shift = 7 - (int)(bit_index % 8);
                bit = (unsigned int)((data_[byte_idx] >> shift) & 0x1);
            }
            val = (val << 1) | bit;
        }
        return val;
    }

    unsigned int peek_bits (int n) const
    {
        return peek_bits_at (pos_, n);
    }

    void skip_bits (int n)
    {
        pos_ += (size_t)n;
    }

    unsigned int read_bits (int n)
    {
        unsigned int val = peek_bits (n);
        skip_bits (n);
        return val;
    }

private:
    const unsigned char *data_;
    size_t total_bits_;
    size_t pos_;
};

void sleep_ms (int ms)
{
#ifdef _WIN32
    Sleep (ms);
#else
    usleep (ms * 1000);
#endif
}

} // namespace


Muse2014::Muse2014 (struct BrainFlowInputParams params)
    : BTLibBoard ((int)BoardIds::MUSE_2014_BOARD, params)
{
    keep_alive = false;
    state = (int)BrainFlowExitCodes::SYNC_TIMEOUT_ERROR;
    have_reference = false;
    last_eeg_timestamp = -1.0;
    sample_counter = 0;
    last_keepalive_timestamp = 0.0;
    for (int i = 0; i < 4; i++)
    {
        carry[i] = 0;
    }
}

Muse2014::~Muse2014 ()
{
    skip_logs = true;
    release_session ();
}

std::string Muse2014::get_name_selector ()
{
    return "Muse";
}

int Muse2014::prepare_session ()
{
    if (params.ip_port <= 0)
    {
        // Confirmed via direct SDP inspection of a real unit: Serial Port
        // Profile is served on RFCOMM channel 1.
        params.ip_port = 1;
    }
    return BTLibBoard::prepare_session ();
}

int Muse2014::start_stream (int buffer_size, const char *streamer_params)
{
    if (!initialized)
    {
        safe_logger (spdlog::level::err, "You need to call prepare_session before start_stream");
        return (int)BrainFlowExitCodes::BOARD_NOT_CREATED_ERROR;
    }
    if (keep_alive)
    {
        safe_logger (spdlog::level::err, "Streaming thread already running");
        return (int)BrainFlowExitCodes::STREAM_ALREADY_RUN_ERROR;
    }

    int res = prepare_for_acquisition (buffer_size, streamer_params);
    if (res != (int)BrainFlowExitCodes::STATUS_OK)
    {
        return res;
    }

    res = bluetooth_open_device ();
    if (res != (int)BrainFlowExitCodes::STATUS_OK)
    {
        return res;
    }

    const unsigned char cmd_version[] = {0x76, 0x20, 0x32, 0x0d}; // "v 2\r"
    const unsigned char cmd_preset[] = {0x25, 0x20, 0x31, 0x30, 0x0d}; // "% 10\r"
    const unsigned char cmd_start[] = {0x73, 0x20, 0x35, 0x0d}; // "s 5\r"

    safe_logger (spdlog::level::trace, "Muse2014: sending startup command sequence");
    bluetooth_write_data ((const char *)cmd_version, (int)sizeof (cmd_version));
    sleep_ms (200);
    bluetooth_write_data ((const char *)cmd_preset, (int)sizeof (cmd_preset));
    sleep_ms (200);
    bluetooth_write_data ((const char *)cmd_start, (int)sizeof (cmd_start));

    have_reference = false;
    last_eeg_timestamp = -1.0;
    sample_counter = 0;
    last_keepalive_timestamp = get_timestamp ();
    for (int i = 0; i < 4; i++)
    {
        carry[i] = 0;
    }

    keep_alive = true;
    streaming_thread = std::thread ([this] { this->read_thread (); });

    std::unique_lock<std::mutex> lk (this->m);
    auto sec = std::chrono::seconds (1);
    int num_secs = 8; // give the classic-BT link longer than BLE boards get
    if (cv.wait_for (lk, num_secs * sec,
            [this] { return this->state != (int)BrainFlowExitCodes::SYNC_TIMEOUT_ERROR; }))
    {
        return state;
    }
    else
    {
        safe_logger (spdlog::level::err, "no data received in {} sec, stopping thread", num_secs);
        stop_stream ();
        return (int)BrainFlowExitCodes::BOARD_NOT_READY_ERROR;
    }
}

int Muse2014::stop_stream ()
{
    if (keep_alive)
    {
        keep_alive = false;
        streaming_thread.join ();
        state = (int)BrainFlowExitCodes::SYNC_TIMEOUT_ERROR;
        // No verified ASCII halt command exists for this protocol (only
        // start/version/preset/keepalive were confirmed from the sources this
        // was ported from) - closing the RFCOMM socket is the same approach
        // BrainFlow's other BTLibBoard-based drivers (Enophone) use.
        return bluetooth_close_device ();
    }
    else
    {
        return (int)BrainFlowExitCodes::STREAM_THREAD_IS_NOT_RUNNING;
    }
}

int Muse2014::release_session ()
{
    if (initialized)
    {
        stop_stream ();
        free_packages ();
    }
    return BTLibBoard::release_session ();
}

int Muse2014::config_board (std::string config, std::string &response)
{
    safe_logger (spdlog::level::debug, "config_board is not supported for Muse2014");
    return (int)BrainFlowExitCodes::UNSUPPORTED_BOARD_ERROR;
}

bool Muse2014::read_exact (unsigned char *buf, int len)
{
    int got = 0;
    while (keep_alive && got < len)
    {
        int res = bluetooth_get_data ((char *)(buf + got), 1);
        if (res == 1)
        {
            got++;
        }
        else
        {
            sleep_ms (1);
        }
    }
    return got == len;
}

bool Muse2014::find_sync ()
{
    unsigned char window[4] = {0, 0, 0, 0};
    int filled = 0;
    while (keep_alive)
    {
        unsigned char b;
        if (!read_exact (&b, 1))
        {
            return false;
        }
        window[0] = window[1];
        window[1] = window[2];
        window[2] = window[3];
        window[3] = b;
        filled++;
        if (filled >= 4 && window[0] == 0xFF && window[1] == 0xFF && window[2] == 0xAA &&
            window[3] == 0x55)
        {
            return true;
        }
    }
    return false;
}

void Muse2014::send_keepalive_if_due ()
{
    double now = get_timestamp ();
    if (now - last_keepalive_timestamp >= 8.0)
    {
        const unsigned char cmd[2] = {0x6b, 0x0d}; // "k\r"
        bluetooth_write_data ((const char *)cmd, 2);
        last_keepalive_timestamp = now;
    }
}

void Muse2014::handle_e0_packet (int num_rows, double *package)
{
    unsigned char raw[5];
    if (!read_exact (raw, 5))
    {
        return;
    }

    BitReader reader (raw, 5);
    std::array<int, 4> vals;
    for (int c = 0; c < 4; c++)
    {
        vals[c] = (int)reader.read_bits (10);
        carry[c] = vals[c];
    }
    have_reference = true;

    std::vector<std::array<int, 4>> batch;
    batch.push_back (vals);
    emit_batch (batch, num_rows, package);
}

void Muse2014::handle_c0_packet (int num_rows, double *package)
{
    unsigned char header[5];
    if (!read_exact (header, 5))
    {
        return;
    }

    int quant[4];
    int median[4];
    {
        BitReader reader (header, 5);
        for (int c = 0; c < 4; c++)
        {
            int q = 1;
            for (int d = 0; d < 4; d++)
            {
                if (reader.read_bits (1))
                {
                    switch (d)
                    {
                        case 0:
                            q *= 16;
                            break;
                        case 1:
                            q *= 8;
                            break;
                        case 2:
                            q *= 4;
                            break;
                        case 3:
                            q *= 2;
                            break;
                    }
                }
            }
            quant[c] = q;
            median[c] = (int)reader.read_bits (6);
        }
    }

    unsigned char len_bytes[2];
    if (!read_exact (len_bytes, 2))
    {
        return;
    }
    int num_bits = ((int)len_bytes[0] << 8) | (int)len_bytes[1];
    int num_bytes = num_bits / 8;
    if ((num_bits % 8) > 0)
    {
        num_bytes++;
    }
    if ((num_bytes <= 0) || (num_bytes > 8192))
    {
        safe_logger (spdlog::level::warn,
            "Muse2014: implausible C0 payload length ({} bits), dropping packet", num_bits);
        return;
    }

    std::vector<unsigned char> payload ((size_t)num_bytes);
    if (!read_exact (payload.data (), num_bytes))
    {
        return;
    }

    if (!have_reference)
    {
        // No E0 reference frame seen yet - can't decode deltas meaningfully.
        safe_logger (spdlog::level::trace, "Muse2014: dropping C0 packet before first E0 frame");
        return;
    }

    BitReader reader (payload.data (), (size_t)num_bytes);

    // Bitstream block order is the reverse of physical channel order - see the
    // file-level comment for how this was confirmed. `carry`, `quant`, and
    // `median` are all indexed by physical channel (0=TP9, 1=AF7, 2=AF8,
    // 3=TP10); only the block-iteration order below is reversed.
    int decoded[4][16];
    for (int blk = 0; blk < 4; blk++)
    {
        int phys = 3 - blk;

        if (median[phys] == 0)
        {
            for (int sm = 0; sm < 16; sm++)
            {
                decoded[phys][sm] = carry[phys];
            }
            reader.skip_bits (16 * 3);
            continue;
        }

        int max_remainder_bits = 0;
        while ((1 << (max_remainder_bits + 1)) <= median[phys])
        {
            max_remainder_bits++;
        }

        for (int sm = 0; sm < 16; sm++)
        {
            int quotient = 0;
            while (reader.peek_bits (1) == 1)
            {
                quotient++;
                reader.skip_bits (1);
            }
            if (quotient == 15)
            {
                int elias_bits = 1;
                while (reader.peek_bits (1) == 0)
                {
                    elias_bits++;
                    reader.skip_bits (1);
                }
                quotient = (int)reader.read_bits (elias_bits);
            }
            else
            {
                reader.skip_bits (1); // consume the terminating 0 of the unary code
            }

            int rem_bits = max_remainder_bits;
            unsigned long max1less = (1UL << (rem_bits + 1)) - (unsigned long)median[phys];
            size_t remainder_pos = reader.position ();
            int remainder = (int)reader.peek_bits_at (remainder_pos, rem_bits);
            if ((unsigned long)remainder >= max1less)
            {
                rem_bits++;
                remainder = (int)reader.peek_bits_at (remainder_pos, rem_bits) - (int)max1less;
            }
            if (rem_bits == 0)
            {
                reader.skip_bits (1);
            }
            reader.skip_bits (rem_bits);

            int sign = reader.read_bits (1) ? 1 : -1;

            carry[phys] =
                carry[phys] + (quotient * median[phys] + remainder) * sign * quant[phys];
            decoded[phys][sm] = carry[phys];
        }
    }

    std::vector<std::array<int, 4>> batch (16);
    for (int sm = 0; sm < 16; sm++)
    {
        for (int c = 0; c < 4; c++)
        {
            batch[sm][c] = decoded[c][sm];
        }
    }
    emit_batch (batch, num_rows, package);
}

void Muse2014::emit_batch (
    const std::vector<std::array<int, 4>> &samples, int num_rows, double *package)
{
    if (samples.empty ())
    {
        return;
    }

    // Raw values are 10-bit unsigned ADC codes. Scale factor below is an
    // *approximation by analogy* with BrainFlow's BLE Muse decoder (12-bit
    // samples scaled at 125.0/256.0 uV/LSB, i.e. ~0.488 uV/LSB) assuming the
    // same total input-referred voltage range and just accounting for the
    // 4x-coarser 10-bit resolution. NOT independently verified - validate
    // against a known-good reference once real data is flowing.
    constexpr double UV_PER_LSB = (125.0 / 256.0) * 4.0; // ~1.953 uV/LSB
    constexpr int MIDPOINT_10BIT = 0x200;

    std::vector<int> eeg_channels = board_descr["default"]["eeg_channels"];
    int package_num_channel = board_descr["default"]["package_num_channel"].get<int> ();
    int timestamp_channel = board_descr["default"]["timestamp_channel"].get<int> ();

    double current_timestamp = get_timestamp ();
    size_t n = samples.size ();
    double step =
        (last_eeg_timestamp > 0) ? (current_timestamp - last_eeg_timestamp) / (double)n : 0.0;

    for (size_t sm = 0; sm < n; sm++)
    {
        for (int c = 0; c < 4; c++)
        {
            double value = (samples[sm][c] - MIDPOINT_10BIT) * UV_PER_LSB;
            package[eeg_channels[c]] = value;
        }
        package[package_num_channel] = (double)(sample_counter++);
        package[timestamp_channel] = (last_eeg_timestamp > 0) ?
            (last_eeg_timestamp + step * (double)(sm + 1)) :
            current_timestamp;
        push_package (package);
    }
    last_eeg_timestamp = current_timestamp;
}

void Muse2014::read_thread ()
{
    int num_rows = board_descr["default"]["num_rows"];
    double *package = new double[num_rows];
    for (int i = 0; i < num_rows; i++)
    {
        package[i] = 0.0;
    }

    while (keep_alive)
    {
        send_keepalive_if_due ();

        if (!find_sync ())
        {
            break; // keep_alive went false
        }
        if (!keep_alive)
        {
            break;
        }

        unsigned char marker = 0;
        if (!read_exact (&marker, 1))
        {
            break;
        }

        if (state != (int)BrainFlowExitCodes::STATUS_OK)
        {
            {
                std::lock_guard<std::mutex> lk (m);
                state = (int)BrainFlowExitCodes::STATUS_OK;
            }
            cv.notify_one ();
            safe_logger (spdlog::level::debug, "Muse2014: start streaming");
        }

        if (marker == 0xE0)
        {
            handle_e0_packet (num_rows, package);
        }
        else if (marker == 0xC0)
        {
            handle_c0_packet (num_rows, package);
        }
        // any other byte immediately after sync: not a frame type we handle,
        // loop back around to find_sync() and resync
    }
    delete[] package;
}

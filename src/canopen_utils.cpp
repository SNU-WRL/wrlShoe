#include "motorized_shoe/canopen_utils.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

namespace motorized_shoe {

CANopenMessage create_sdo_download(uint32_t node_id, uint16_t index,
                                   uint8_t subindex, uint32_t value, int length) {
    CANopenMessage msg;
    msg.can_id = encode_canopen_sdo_id(node_id);

    uint8_t cs = 0x23;
    if (length == 1) {
        cs = 0x2F;
    } else if (length == 2) {
        cs = 0x2B;
    }

    msg.data[0] = cs;
    msg.data[1] = static_cast<uint8_t>(index & 0xFF);
    msg.data[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
    msg.data[3] = subindex;

    for (int i = 0; i < length && i < 4; ++i) {
        msg.data[4 + i] = static_cast<uint8_t>((value >> (8 * i)) & 0xFF);
    }
    for (int i = length; i < 4; ++i) {
        msg.data[4 + i] = 0;
    }

    msg.dlc = 8;
    return msg;
}

CANopenMessage create_sdo_upload(uint32_t node_id, uint16_t index, uint8_t subindex) {
    CANopenMessage msg;
    msg.can_id = encode_canopen_sdo_id(node_id);

    msg.data[0] = 0x40;  // initiate upload (read) request
    msg.data[1] = static_cast<uint8_t>(index & 0xFF);
    msg.data[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
    msg.data[3] = subindex;
    // data[4..7] left zero (unused in the request)

    msg.dlc = 8;
    return msg;
}

CANopenMessage create_nmt_message(uint32_t node_id, uint8_t command) {
    CANopenMessage msg;
    msg.can_id = 0x000;
    msg.data[0] = command;
    msg.data[1] = static_cast<uint8_t>(node_id);
    msg.dlc = 2;
    return msg;
}

CANopenMessage create_sync_message() {
    CANopenMessage msg;
    msg.can_id = CANOPEN_SYNC_COB_ID;
    msg.dlc = 0;
    return msg;
}

namespace {

// Waits until `pred(d, len)` accepts a frame from `expect_id`, skipping
// unrelated traffic on the same COB-ID (statusword poll replies, stale
// responses from earlier exchanges). Returns false on timeout/socket error.
template <typename Pred>
bool recv_sdo_frame(CANSocket& sock, uint32_t expect_id, int timeout_ms, Pred pred,
                    uint8_t (&d)[8], size_t& len) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (true) {
        const auto rem = std::chrono::duration_cast<std::chrono::milliseconds>(
                             deadline - std::chrono::steady_clock::now())
                             .count();
        if (rem <= 0) {
            return false;
        }
        uint32_t rx_id = 0;
        std::memset(d, 0, sizeof(d));
        len = 0;
        bool ok = false;
        try {
            ok = sock.recv_message(rx_id, d, len, static_cast<int>(rem));
        } catch (...) {
            return false;
        }
        if (!ok) {
            return false;
        }
        if (rx_id != expect_id || len < 1) {
            continue;
        }
        if (pred(d, len)) {
            return true;
        }
    }
}

bool index_matches(const uint8_t* d, uint16_t index, uint8_t subindex) {
    return static_cast<uint16_t>(d[1] | (d[2] << 8)) == index && d[3] == subindex;
}

uint32_t abort_code_of(const uint8_t* d) {
    return static_cast<uint32_t>(d[4]) | (static_cast<uint32_t>(d[5]) << 8) |
           (static_cast<uint32_t>(d[6]) << 16) | (static_cast<uint32_t>(d[7]) << 24);
}

std::string hex32(uint32_t v) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "0x%08X", v);
    return buf;
}

// SDO download (write) of an arbitrary-length byte string, expedited when it
// fits in 4 bytes, segmented otherwise. Returns empty string on success.
std::string sdo_download_bytes(CANSocket& sock, int node_id, uint16_t index, uint8_t subindex,
                               const std::string& payload, int timeout_ms) {
    const uint32_t tx_id = encode_canopen_sdo_id(static_cast<uint32_t>(node_id));
    const uint32_t rx_id = encode_canopen_sdo_rx_id(static_cast<uint32_t>(node_id));
    const size_t n = payload.size();
    uint8_t d[8] = {0};
    size_t len = 0;

    uint8_t req[8] = {0};
    req[1] = static_cast<uint8_t>(index & 0xFF);
    req[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
    req[3] = subindex;

    if (n >= 1 && n <= 4) {
        req[0] = static_cast<uint8_t>(0x23 | ((4 - n) << 2));  // expedited, size set
        std::memcpy(&req[4], payload.data(), n);
        sock.send_message(tx_id, req, 8);
    } else {
        req[0] = 0x21;  // initiate segmented download, size in data bytes
        const uint32_t size = static_cast<uint32_t>(n);
        req[4] = static_cast<uint8_t>(size & 0xFF);
        req[5] = static_cast<uint8_t>((size >> 8) & 0xFF);
        req[6] = static_cast<uint8_t>((size >> 16) & 0xFF);
        req[7] = static_cast<uint8_t>((size >> 24) & 0xFF);
        sock.send_message(tx_id, req, 8);
    }

    // Initiate response: 0x60 ack or 0x80 abort, both echoing index/subindex.
    if (!recv_sdo_frame(
            sock, rx_id, timeout_ms,
            [&](const uint8_t* f, size_t l) {
                return l >= 4 && (f[0] == 0x60 || f[0] == 0x80) && index_matches(f, index, subindex);
            },
            d, len)) {
        return "no response to download initiate";
    }
    if (d[0] == 0x80) {
        return "download aborted, code " + hex32(abort_code_of(d));
    }
    if (n <= 4) {
        return "";
    }

    // Segments: 7 payload bytes each, toggle alternates, last segment sets bit 0.
    size_t off = 0;
    uint8_t toggle = 0;
    while (off < n) {
        const size_t chunk = (n - off < 7) ? (n - off) : 7;
        const bool last = off + chunk >= n;
        uint8_t seg[8] = {0};
        seg[0] = static_cast<uint8_t>((toggle << 4) | ((7 - chunk) << 1) | (last ? 1 : 0));
        std::memcpy(&seg[1], payload.data() + off, chunk);
        sock.send_message(tx_id, seg, 8);

        // Segment ack (scs=1, matching toggle) carries no index echo; aborts do.
        if (!recv_sdo_frame(
                sock, rx_id, timeout_ms,
                [&](const uint8_t* f, size_t l) {
                    if (f[0] == 0x80) {
                        return l >= 8 && index_matches(f, index, subindex);
                    }
                    return (f[0] & 0xE0) == 0x20 && ((f[0] >> 4) & 1) == toggle;
                },
                d, len)) {
            return "no response to download segment";
        }
        if (d[0] == 0x80) {
            return "download segment aborted, code " + hex32(abort_code_of(d));
        }
        off += chunk;
        toggle ^= 1;
    }
    return "";
}

// SDO upload (read) of an arbitrary-length byte string into `out`, expedited
// or segmented as chosen by the drive. Returns empty string on success.
std::string sdo_upload_bytes(CANSocket& sock, int node_id, uint16_t index, uint8_t subindex,
                             std::string& out, int timeout_ms) {
    const uint32_t rx_id = encode_canopen_sdo_rx_id(static_cast<uint32_t>(node_id));
    auto req = create_sdo_upload(static_cast<uint32_t>(node_id), index, subindex);
    sock.send_message(req.can_id, req.data, req.dlc);

    uint8_t d[8] = {0};
    size_t len = 0;
    if (!recv_sdo_frame(
            sock, rx_id, timeout_ms,
            [&](const uint8_t* f, size_t l) {
                return l >= 4 && (((f[0] & 0xE0) == 0x40) || f[0] == 0x80) &&
                       index_matches(f, index, subindex);
            },
            d, len)) {
        return "no response to upload request";
    }
    if (d[0] == 0x80) {
        return "upload aborted, code " + hex32(abort_code_of(d));
    }

    if (d[0] & 0x02) {  // expedited
        const size_t n = (d[0] & 0x01) ? 4 - ((d[0] >> 2) & 0x03) : 4;
        out.assign(reinterpret_cast<const char*>(&d[4]), n);
        return "";
    }

    // Segmented upload: request each segment with alternating toggle.
    out.clear();
    uint8_t toggle = 0;
    while (true) {
        uint8_t seg_req[8] = {0};
        seg_req[0] = static_cast<uint8_t>(0x60 | (toggle << 4));
        sock.send_message(encode_canopen_sdo_id(static_cast<uint32_t>(node_id)), seg_req, 8);

        // Segment data (scs=0, matching toggle) carries no index echo; aborts do.
        if (!recv_sdo_frame(
                sock, rx_id, timeout_ms,
                [&](const uint8_t* f, size_t l) {
                    if (f[0] == 0x80) {
                        return l >= 8 && index_matches(f, index, subindex);
                    }
                    return (f[0] & 0xE0) == 0x00 && ((f[0] >> 4) & 1) == toggle;
                },
                d, len)) {
            return "no response to upload segment";
        }
        if (d[0] == 0x80) {
            return "upload segment aborted, code " + hex32(abort_code_of(d));
        }
        const size_t n = 7 - ((d[0] >> 1) & 0x07);
        out.append(reinterpret_cast<const char*>(&d[1]), n);
        if (d[0] & 0x01) {
            return "";
        }
        toggle ^= 1;
    }
}

}  // namespace

ElmoOsCommandResult elmo_os_command(CANSocket& sock, int node_id, const std::string& command,
                                    int recv_timeout_ms) {
    using namespace std::chrono_literals;
    ElmoOsCommandResult res;

    std::string err =
        sdo_download_bytes(sock, node_id, CANOPEN_OS_COMMAND, 1, command, recv_timeout_ms);
    if (!err.empty()) {
        res.error = "command write: " + err;
        return res;
    }

    // Poll the status byte until the interpreter finishes (0xFF = executing).
    for (int i = 0; i < 30; ++i) {
        std::string status_bytes;
        err = sdo_upload_bytes(sock, node_id, CANOPEN_OS_COMMAND, 2, status_bytes,
                               recv_timeout_ms);
        if (!err.empty()) {
            res.error = "status read: " + err;
            return res;
        }
        res.status = status_bytes.empty() ? 0xFF : static_cast<uint8_t>(status_bytes[0]);
        if (res.status != 0xFF) {
            break;
        }
        std::this_thread::sleep_for(10ms);
    }
    if (res.status == 0xFF) {
        res.error = "command still executing after status polls";
        return res;
    }

    err = sdo_upload_bytes(sock, node_id, CANOPEN_OS_COMMAND, 3, res.reply, recv_timeout_ms);
    if (!err.empty()) {
        res.error = "reply read: " + err;
        return res;
    }
    while (!res.reply.empty() && (res.reply.back() == '\0' || res.reply.back() == ';')) {
        res.reply.pop_back();
    }

    if (res.status != 0x00 && res.status != 0x01) {
        res.error = "drive reported command error (status " + std::to_string(res.status) +
                    ", reply '" + res.reply + "')";
        return res;
    }
    res.ok = true;
    return res;
}

}  // namespace motorized_shoe

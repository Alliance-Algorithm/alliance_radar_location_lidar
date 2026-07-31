#include "radar_fast_livo2_rgb/rgb_pcd_io.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <unordered_set>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace radar::fast_livo2::rgb {
namespace {

    namespace fs = std::filesystem;

    constexpr std::size_t kChunkRecordSize = 28;
    constexpr std::size_t kFinalRecordSize = 16;

    enum class RecordKind {
        Chunk,
        Final,
    };

    struct SchemaSpec {
        std::span<const std::string_view> fields;
        std::span<const std::uint64_t> sizes;
        std::span<const std::string_view> types;
        std::size_t record_size;
    };

    constexpr std::array<std::string_view, 5> kChunkFields { "x", "y", "z", "rgb", "quality" };
    constexpr std::array<std::uint64_t, 5> kChunkSizes { 4, 4, 4, 4, 8 };
    constexpr std::array<std::string_view, 5> kChunkTypes { "F", "F", "F", "F", "F" };

    constexpr std::array<std::string_view, 4> kFinalFields { "x", "y", "z", "rgb" };
    constexpr std::array<std::uint64_t, 4> kFinalSizes { 4, 4, 4, 4 };
    constexpr std::array<std::string_view, 4> kFinalTypes { "F", "F", "F", "F" };

    const SchemaSpec& schema_for(RecordKind kind) {
        static const SchemaSpec chunk { kChunkFields, kChunkSizes, kChunkTypes, kChunkRecordSize };
        static const SchemaSpec final { kFinalFields, kFinalSizes, kFinalTypes, kFinalRecordSize };
        return kind == RecordKind::Chunk ? chunk : final;
    }

    void set_error(std::string& error, std::string message) { error = std::move(message); }

    std::string path_prefix(const fs::path& path, std::string_view phase) {
        return std::string(phase) + " for '" + path.string() + "': ";
    }

    std::string errno_message(int value) { return std::strerror(value); }

    bool parse_uint64(std::string_view token, std::uint64_t& value) {
        if (token.empty()) {
            return false;
        }
        const auto* first = token.data();
        const auto* last  = first + token.size();
        const auto result = std::from_chars(first, last, value, 10);
        return result.ec == std::errc { } && result.ptr == last;
    }

    bool parse_size(std::string_view token, std::size_t& value) {
        std::uint64_t parsed = 0;
        if (!parse_uint64(token, parsed)
            || parsed > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
            return false;
        }
        value = static_cast<std::size_t>(parsed);
        return true;
    }

    bool parse_finite_double(std::string_view token) {
        std::string owned(token);
        char* end          = nullptr;
        errno              = 0;
        const double value = std::strtod(owned.c_str(), &end);
        return errno != ERANGE && end == owned.c_str() + owned.size() && std::isfinite(value);
    }

    template <typename T>
    bool parse_uint64_list(const std::vector<std::string>& tokens, std::vector<T>& values) {
        values.clear();
        values.reserve(tokens.size() - 1U);
        for (std::size_t i = 1; i < tokens.size(); ++i) {
            std::uint64_t value = 0;
            if (!parse_uint64(tokens[i], value)
                || value > static_cast<std::uint64_t>(std::numeric_limits<T>::max())) {
                return false;
            }
            values.push_back(static_cast<T>(value));
        }
        return true;
    }

    bool matches_fields(
        const std::vector<std::string>& actual, std::span<const std::string_view> expected) {
        if (actual.size() != expected.size()) {
            return false;
        }
        for (std::size_t i = 0; i < actual.size(); ++i) {
            if (actual[i] != expected[i]) {
                return false;
            }
        }
        return true;
    }

    bool matches_types(
        const std::vector<std::string>& actual, std::span<const std::string_view> expected) {
        if (actual.size() != expected.size()) {
            return false;
        }
        for (std::size_t i = 0; i < actual.size(); ++i) {
            if (actual[i] != expected[i]) {
                return false;
            }
        }
        return true;
    }

    bool matches_sizes(
        const std::vector<std::uint64_t>& actual, std::span<const std::uint64_t> expected) {
        if (actual.size() != expected.size()) {
            return false;
        }
        for (std::size_t i = 0; i < actual.size(); ++i) {
            if (actual[i] != expected[i]) {
                return false;
            }
        }
        return true;
    }

    bool has_duplicate_fields(const std::vector<std::string>& fields) {
        std::unordered_set<std::string> unique;
        for (const auto& field : fields) {
            if (!unique.insert(field).second) {
                return true;
            }
        }
        return false;
    }

    struct ParsedHeader {
        std::size_t count          = 0;
        std::uintmax_t data_offset = 0;
    };

    bool parse_header(std::ifstream& input, const fs::path& path, const SchemaSpec& spec,
        ParsedHeader& header, std::string& error) {
        std::unordered_set<std::string> seen_keys;
        std::vector<std::string> fields;
        std::vector<std::uint64_t> sizes;
        std::vector<std::string> types;
        std::vector<std::uint64_t> counts;
        std::optional<std::size_t> width;
        std::optional<std::size_t> height;
        std::optional<std::size_t> points;
        bool data_seen = false;
        std::string line;

        while (std::getline(input, line)) {
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }

            const auto first_non_space = line.find_first_not_of(" \t");
            if (first_non_space == std::string::npos) {
                set_error(error, path_prefix(path, "header parse") + "blank line");
                return false;
            }
            if (line[first_non_space] == '#') {
                continue;
            }

            std::istringstream line_stream(line.substr(first_non_space));
            std::vector<std::string> tokens { std::istream_iterator<std::string> { line_stream },
                std::istream_iterator<std::string> { } };
            if (tokens.empty()) {
                set_error(error, path_prefix(path, "header parse") + "empty header line");
                return false;
            }

            const std::string& key = tokens.front();
            if (key == "DATA") {
                if (!seen_keys.insert(key).second) {
                    set_error(error, path_prefix(path, "header parse") + "duplicate DATA line");
                    return false;
                }
                if (tokens.size() != 2U || tokens[1] != "binary") {
                    set_error(error,
                        path_prefix(path, "header parse") + "DATA must be exactly 'DATA binary'");
                    return false;
                }
                data_seen           = true;
                const auto position = input.tellg();
                if (position == std::streampos(-1)) {
                    set_error(error,
                        path_prefix(path, "header parse") + "cannot determine payload offset");
                    return false;
                }
                const auto offset = static_cast<std::streamoff>(position);
                if (offset < 0) {
                    set_error(error, path_prefix(path, "header parse") + "invalid payload offset");
                    return false;
                }
                header.data_offset = static_cast<std::uintmax_t>(offset);
                break;
            }

            if (!seen_keys.insert(key).second) {
                set_error(
                    error, path_prefix(path, "header parse") + "duplicate '" + key + "' line");
                return false;
            }

            if (key == "VERSION") {
                if (tokens.size() != 2U || tokens[1] != "0.7") {
                    set_error(error, path_prefix(path, "header parse") + "unsupported VERSION");
                    return false;
                }
            } else if (key == "FIELDS") {
                if (tokens.size() < 2U) {
                    set_error(
                        error, path_prefix(path, "header parse") + "FIELDS is missing field names");
                    return false;
                }
                fields.assign(tokens.begin() + 1, tokens.end());
                if (has_duplicate_fields(fields)) {
                    set_error(error, path_prefix(path, "header parse") + "duplicate field name");
                    return false;
                }
            } else if (key == "SIZE") {
                if (tokens.size() < 2U || !parse_uint64_list(tokens, sizes)) {
                    set_error(error, path_prefix(path, "header parse") + "invalid SIZE values");
                    return false;
                }
            } else if (key == "TYPE") {
                if (tokens.size() < 2U) {
                    set_error(
                        error, path_prefix(path, "header parse") + "TYPE is missing field types");
                    return false;
                }
                types.assign(tokens.begin() + 1, tokens.end());
            } else if (key == "COUNT") {
                if (tokens.size() < 2U || !parse_uint64_list(tokens, counts)) {
                    set_error(error, path_prefix(path, "header parse") + "invalid COUNT values");
                    return false;
                }
            } else if (key == "WIDTH") {
                std::size_t value = 0;
                if (tokens.size() != 2U || !parse_size(tokens[1], value)) {
                    set_error(error, path_prefix(path, "header parse") + "invalid WIDTH");
                    return false;
                }
                width = value;
            } else if (key == "HEIGHT") {
                std::size_t value = 0;
                if (tokens.size() != 2U || !parse_size(tokens[1], value)) {
                    set_error(error, path_prefix(path, "header parse") + "invalid HEIGHT");
                    return false;
                }
                height = value;
            } else if (key == "VIEWPOINT") {
                if (tokens.size() != 8U) {
                    set_error(error,
                        path_prefix(path, "header parse") + "VIEWPOINT must contain seven values");
                    return false;
                }
                for (std::size_t i = 1; i < tokens.size(); ++i) {
                    if (!parse_finite_double(tokens[i])) {
                        set_error(
                            error, path_prefix(path, "header parse") + "invalid VIEWPOINT value");
                        return false;
                    }
                }
            } else if (key == "POINTS") {
                std::size_t value = 0;
                if (tokens.size() != 2U || !parse_size(tokens[1], value)) {
                    set_error(error, path_prefix(path, "header parse") + "invalid POINTS");
                    return false;
                }
                points = value;
            } else {
                set_error(error,
                    path_prefix(path, "header parse") + "unsupported header key '" + key + "'");
                return false;
            }
        }

        if (!data_seen) {
            set_error(error, path_prefix(path, "header parse") + "missing DATA binary line");
            return false;
        }
        if (fields.empty() || sizes.empty() || types.empty() || counts.empty() || !width.has_value()
            || !height.has_value() || !points.has_value()) {
            set_error(error, path_prefix(path, "header parse") + "missing required header entry");
            return false;
        }
        if (!matches_fields(fields, spec.fields)) {
            set_error(
                error, path_prefix(path, "header parse") + "unsupported or incomplete field list");
            return false;
        }
        if (!matches_sizes(sizes, spec.sizes)) {
            set_error(error, path_prefix(path, "header parse") + "unsupported field sizes");
            return false;
        }
        if (!matches_types(types, spec.types)) {
            set_error(error, path_prefix(path, "header parse") + "unsupported field types");
            return false;
        }
        if (counts.size() != spec.fields.size()
            || !std::all_of(
                counts.begin(), counts.end(), [](std::uint64_t count) { return count == 1U; })) {
            set_error(error, path_prefix(path, "header parse") + "unsupported field counts");
            return false;
        }
        if (*height != 1U) {
            set_error(error, path_prefix(path, "header parse") + "HEIGHT must be 1");
            return false;
        }
        if (*width != *points) {
            set_error(error, path_prefix(path, "header parse") + "WIDTH and POINTS do not match");
            return false;
        }
        header.count = *points;
        return true;
    }

    bool checked_payload_size(
        std::size_t count, std::size_t record_size, std::size_t& payload_size, std::string& error) {
        if (count > std::numeric_limits<std::size_t>::max() / record_size) {
            set_error(error, "payload size overflow");
            return false;
        }
        payload_size = count * record_size;
        return true;
    }

    bool check_payload_length(const fs::path& path, const ParsedHeader& header,
        const SchemaSpec& spec, std::string& error) {
        std::size_t payload_size = 0;
        if (!checked_payload_size(header.count, spec.record_size, payload_size, error)) {
            return false;
        }
        std::error_code file_error;
        const auto file_size = fs::file_size(path, file_error);
        if (file_error) {
            set_error(error, path_prefix(path, "payload validation") + file_error.message());
            return false;
        }
        const auto expected_file_size =
            header.data_offset + static_cast<std::uintmax_t>(payload_size);
        if (expected_file_size < header.data_offset || file_size != expected_file_size) {
            set_error(error,
                path_prefix(path, "payload validation")
                    + "payload length does not match POINTS and schema");
            return false;
        }
        return true;
    }

    bool decode_chunk(const std::byte* bytes, RgbChunkRecord& record, std::string& reason) {
        std::memcpy(&record.x, bytes, sizeof(record.x));
        std::memcpy(&record.y, bytes + 4, sizeof(record.y));
        std::memcpy(&record.z, bytes + 8, sizeof(record.z));
        std::memcpy(&record.rgb, bytes + 12, sizeof(record.rgb));
        std::memcpy(&record.quality, bytes + 16, sizeof(record.quality));
        if (!std::isfinite(record.x) || !std::isfinite(record.y) || !std::isfinite(record.z)
            || !std::isfinite(record.quality)) {
            reason = "record contains a non-finite coordinate or quality";
            return false;
        }
        return true;
    }

    bool decode_final(const std::byte* bytes, RgbFinalRecord& record, std::string& reason) {
        std::memcpy(&record.x, bytes, sizeof(record.x));
        std::memcpy(&record.y, bytes + 4, sizeof(record.y));
        std::memcpy(&record.z, bytes + 8, sizeof(record.z));
        std::memcpy(&record.rgb, bytes + 12, sizeof(record.rgb));
        if (!std::isfinite(record.x) || !std::isfinite(record.y) || !std::isfinite(record.z)) {
            reason = "record contains a non-finite coordinate";
            return false;
        }
        return true;
    }

    bool validate_chunk_record(const RgbChunkRecord& record, std::string& reason) {
        if (!std::isfinite(record.x) || !std::isfinite(record.y) || !std::isfinite(record.z)
            || !std::isfinite(record.quality)) {
            reason = "record contains a non-finite coordinate or quality";
            return false;
        }
        return true;
    }

    bool validate_final_record(const RgbFinalRecord& record, std::string& reason) {
        if (!std::isfinite(record.x) || !std::isfinite(record.y) || !std::isfinite(record.z)) {
            reason = "record contains a non-finite coordinate";
            return false;
        }
        return true;
    }

    template <typename Record, typename Decoder>
    bool read_records(const fs::path& path, const SchemaSpec& spec, std::vector<Record>& records,
        std::string& error, Decoder&& decoder) {
        records.clear();
        error.clear();

        std::ifstream input(path, std::ios::binary);
        if (!input.is_open()) {
            set_error(error, path_prefix(path, "open") + errno_message(errno));
            return false;
        }

        ParsedHeader header;
        if (!parse_header(input, path, spec, header, error)
            || !check_payload_length(path, header, spec, error)) {
            return false;
        }
        if (header.count > records.max_size()) {
            set_error(error, path_prefix(path, "payload read") + "POINTS exceeds vector capacity");
            return false;
        }

        std::size_t payload_size = 0;
        if (!checked_payload_size(header.count, spec.record_size, payload_size, error)) {
            return false;
        }
        (void)payload_size;

        std::vector<Record> decoded;
        try {
            decoded.reserve(header.count);
        } catch (const std::exception& exception) {
            set_error(error, path_prefix(path, "payload read") + exception.what());
            return false;
        }

        std::array<std::byte, kChunkRecordSize> record_bytes { };
        for (std::size_t i = 0; i < header.count; ++i) {
            input.read(reinterpret_cast<char*>(record_bytes.data()),
                static_cast<std::streamsize>(spec.record_size));
            if (input.gcount() != static_cast<std::streamsize>(spec.record_size)) {
                set_error(error, path_prefix(path, "payload read") + "short binary record");
                return false;
            }

            Record record { };
            std::string reason;
            if (!decoder(record_bytes.data(), record, reason)) {
                set_error(error, path_prefix(path, "payload read") + reason);
                return false;
            }
            decoded.push_back(record);
        }

        records.swap(decoded);
        return true;
    }

    bool records_equal(const RgbChunkRecord& expected, const RgbChunkRecord& actual) {
        return std::memcmp(&expected.x, &actual.x, sizeof(expected.x)) == 0
            && std::memcmp(&expected.y, &actual.y, sizeof(expected.y)) == 0
            && std::memcmp(&expected.z, &actual.z, sizeof(expected.z)) == 0
            && expected.rgb == actual.rgb
            && std::memcmp(&expected.quality, &actual.quality, sizeof(expected.quality)) == 0;
    }

    bool records_equal(const RgbFinalRecord& expected, const RgbFinalRecord& actual) {
        return std::memcmp(&expected.x, &actual.x, sizeof(expected.x)) == 0
            && std::memcmp(&expected.y, &actual.y, sizeof(expected.y)) == 0
            && std::memcmp(&expected.z, &actual.z, sizeof(expected.z)) == 0
            && expected.rgb == actual.rgb;
    }

    bool validate_file(
        const fs::path& path, std::span<const RgbChunkRecord> expected, std::string& error) {
        std::vector<RgbChunkRecord> actual;
        if (!read_records(path, schema_for(RecordKind::Chunk), actual, error, decode_chunk)) {
            return false;
        }
        if (actual.size() != expected.size()) {
            set_error(error, path_prefix(path, "validate") + "record count does not match input");
            return false;
        }
        for (std::size_t i = 0; i < expected.size(); ++i) {
            if (!records_equal(expected[i], actual[i])) {
                set_error(
                    error, path_prefix(path, "validate") + "read-back record differs from input");
                return false;
            }
        }
        return true;
    }

    bool validate_file(
        const fs::path& path, std::span<const RgbFinalRecord> expected, std::string& error) {
        std::vector<RgbFinalRecord> actual;
        if (!read_records(path, schema_for(RecordKind::Final), actual, error, decode_final)) {
            return false;
        }
        if (actual.size() != expected.size()) {
            set_error(error, path_prefix(path, "validate") + "record count does not match input");
            return false;
        }
        for (std::size_t i = 0; i < expected.size(); ++i) {
            if (!records_equal(expected[i], actual[i])) {
                set_error(
                    error, path_prefix(path, "validate") + "read-back record differs from input");
                return false;
            }
        }
        return true;
    }

    bool encode_chunk(const RgbChunkRecord& record, std::byte* bytes, std::string& reason) {
        (void)reason;
        std::memcpy(bytes, &record.x, sizeof(record.x));
        std::memcpy(bytes + 4, &record.y, sizeof(record.y));
        std::memcpy(bytes + 8, &record.z, sizeof(record.z));
        std::memcpy(bytes + 12, &record.rgb, sizeof(record.rgb));
        std::memcpy(bytes + 16, &record.quality, sizeof(record.quality));
        return true;
    }

    bool encode_final(const RgbFinalRecord& record, std::byte* bytes, std::string& reason) {
        (void)reason;
        std::memcpy(bytes, &record.x, sizeof(record.x));
        std::memcpy(bytes + 4, &record.y, sizeof(record.y));
        std::memcpy(bytes + 8, &record.z, sizeof(record.z));
        std::memcpy(bytes + 12, &record.rgb, sizeof(record.rgb));
        return true;
    }

    std::string make_header(const SchemaSpec& spec, std::size_t count) {
        std::string header = "# .PCD v0.7 - Point Cloud Data file format\n";
        header += "VERSION 0.7\nFIELDS";
        for (const auto field : spec.fields) {
            header += " ";
            header += field;
        }
        header += "\nSIZE";
        for (const auto size : spec.sizes) {
            header += " ";
            header += std::to_string(size);
        }
        header += "\nTYPE";
        for (const auto type : spec.types) {
            header += " ";
            header += type;
        }
        header += "\nCOUNT";
        for (std::size_t i = 0; i < spec.fields.size(); ++i) {
            header += " 1";
        }
        header += "\nWIDTH " + std::to_string(count);
        header += "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS ";
        header += std::to_string(count);
        header += "\nDATA binary\n";
        return header;
    }

    bool write_all(int fd, const void* data, std::size_t size, std::string& error) {
        const auto* bytes = static_cast<const std::byte*>(data);
        while (size != 0U) {
            const auto request_size = std::min<std::size_t>(
                size, static_cast<std::size_t>(std::numeric_limits<ssize_t>::max()));
            const auto written = ::write(fd, bytes, request_size);
            if (written < 0) {
                if (errno == EINTR) {
                    continue;
                }
                set_error(error, "write failed: " + errno_message(errno));
                return false;
            }
            if (written == 0) {
                set_error(error, "write returned zero bytes");
                return false;
            }
            bytes += written;
            size -= static_cast<std::size_t>(written);
        }
        return true;
    }

    struct TempIdentity {
        bool created    = false;
        bool identified = false;
        dev_t device    = 0;
        ino_t inode     = 0;
    };

    void remove_owned_temp(const fs::path& path, const TempIdentity& identity) {
        if (!identity.created || !identity.identified) {
            return;
        }
        struct stat current { };
        if (::stat(path.c_str(), &current) == 0 && current.st_dev == identity.device
            && current.st_ino == identity.inode) {
            (void)::unlink(path.c_str());
        }
    }

    bool make_parent_directory(const fs::path& formal, std::string& error) {
        fs::path parent = formal.parent_path();
        if (parent.empty()) {
            parent = ".";
        }

        std::error_code directory_error;
        fs::create_directories(parent, directory_error);
        if (directory_error) {
            std::error_code status_error;
            if (!fs::is_directory(parent, status_error) || status_error) {
                set_error(error,
                    path_prefix(formal, "create parent directory") + directory_error.message());
                return false;
            }
        }

        std::error_code status_error;
        if (!fs::is_directory(parent, status_error) || status_error) {
            set_error(error,
                path_prefix(formal, "create parent directory")
                    + (status_error ? status_error.message() : "parent is not a directory"));
            return false;
        }
        return true;
    }

    bool fsync_file(const fs::path& path, std::string& error) {
        int flags = O_WRONLY;
#ifdef O_CLOEXEC
        flags |= O_CLOEXEC;
#endif
        const int fd = ::open(path.c_str(), flags);
        if (fd < 0) {
            set_error(
                error, path_prefix(path, "open temporary file for fsync") + errno_message(errno));
            return false;
        }
        if (::fsync(fd) != 0) {
            const int saved_errno = errno;
            (void)::close(fd);
            set_error(
                error, path_prefix(path, "fsync temporary file") + errno_message(saved_errno));
            return false;
        }
        if (::close(fd) != 0) {
            set_error(error, path_prefix(path, "close temporary file") + errno_message(errno));
            return false;
        }
        return true;
    }

    bool fsync_parent_directory(const fs::path& formal, std::string& error) {
        fs::path parent = formal.parent_path();
        if (parent.empty()) {
            parent = ".";
        }
        int flags = O_RDONLY;
#ifdef O_DIRECTORY
        flags |= O_DIRECTORY;
#endif
#ifdef O_CLOEXEC
        flags |= O_CLOEXEC;
#endif
        const int fd = ::open(parent.c_str(), flags);
        if (fd < 0) {
            set_error(error,
                path_prefix(formal, "open parent directory for fsync") + errno_message(errno));
            return false;
        }
        if (::fsync(fd) != 0) {
            const int saved_errno = errno;
            (void)::close(fd);
            if (saved_errno == EINVAL || saved_errno == ENOTSUP || saved_errno == EOPNOTSUPP) {
                return true;
            }
            set_error(
                error, path_prefix(formal, "fsync parent directory") + errno_message(saved_errno));
            return false;
        }
        if (::close(fd) != 0) {
            set_error(error, path_prefix(formal, "close parent directory") + errno_message(errno));
            return false;
        }
        return true;
    }

    template <typename Record, typename Encoder, typename Validator>
    bool write_transactional(const fs::path& formal, std::span<const Record> records,
        const SchemaSpec& spec, Encoder&& encoder, Validator&& validator, std::string& error) {
        error.clear();
        if (formal.empty()) {
            set_error(error, "transactional write: formal path is empty");
            return false;
        }

        std::size_t payload_size = 0;
        if (!checked_payload_size(records.size(), spec.record_size, payload_size, error)) {
            return false;
        }

        std::vector<std::byte> payload;
        try {
            payload.resize(payload_size);
        } catch (const std::exception& exception) {
            set_error(error, path_prefix(formal, "allocate payload") + exception.what());
            return false;
        }
        for (std::size_t i = 0; i < records.size(); ++i) {
            std::string reason;
            if (!validator(records[i], reason)) {
                set_error(error, path_prefix(formal, "validate input") + reason);
                return false;
            }
            encoder(records[i], payload.data() + i * spec.record_size);
        }

        if (!make_parent_directory(formal, error)) {
            return false;
        }

        fs::path temporary = formal;
        temporary += ".tmp";

        int fd_flags = O_WRONLY | O_CREAT | O_EXCL;
#ifdef O_CLOEXEC
        fd_flags |= O_CLOEXEC;
#endif
        const int fd = ::open(temporary.c_str(), fd_flags, 0666);
        if (fd < 0) {
            set_error(
                error, path_prefix(temporary, "create temporary file") + errno_message(errno));
            return false;
        }

        TempIdentity identity;
        identity.created = true;
        struct stat opened_stat { };
        if (::fstat(fd, &opened_stat) == 0) {
            identity.identified = true;
            identity.device     = opened_stat.st_dev;
            identity.inode      = opened_stat.st_ino;
        }

        auto fail = [&](std::string message) {
            if (::close(fd) != 0 && message.empty()) {
                message = path_prefix(temporary, "close temporary file") + errno_message(errno);
            }
            remove_owned_temp(temporary, identity);
            set_error(error, std::move(message));
            return false;
        };

        const std::string header = make_header(spec, records.size());
        if (!write_all(fd, header.data(), header.size(), error)) {
            const std::string message = path_prefix(temporary, "write header") + error;
            return fail(message);
        }
        if (!write_all(fd, payload.data(), payload.size(), error)) {
            const std::string message = path_prefix(temporary, "write payload") + error;
            return fail(message);
        }
        if (::close(fd) != 0) {
            set_error(error, path_prefix(temporary, "close temporary file") + errno_message(errno));
            remove_owned_temp(temporary, identity);
            return false;
        }

        if (!validate_file(temporary, records, error)) {
            remove_owned_temp(temporary, identity);
            return false;
        }
        if (!fsync_file(temporary, error)) {
            remove_owned_temp(temporary, identity);
            return false;
        }
        if (::rename(temporary.c_str(), formal.c_str()) != 0) {
            const int saved_errno = errno;
            remove_owned_temp(temporary, identity);
            set_error(
                error, path_prefix(formal, "rename temporary file") + errno_message(saved_errno));
            return false;
        }
        identity.created = false;

        if (!fsync_parent_directory(formal, error)) {
            return false;
        }
        return true;
    }

} // namespace

bool write_rgb_chunk_transactional(
    const fs::path& path, std::span<const RgbChunkRecord> records, std::string& error) {
    return write_transactional(
        path, records, schema_for(RecordKind::Chunk),
        [](const RgbChunkRecord& record, std::byte* bytes) {
            std::string ignored;
            (void)encode_chunk(record, bytes, ignored);
        },
        [](const RgbChunkRecord& record, std::string& reason) {
            return validate_chunk_record(record, reason);
        },
        error);
}

bool write_rgb_final_transactional(
    const fs::path& path, std::span<const RgbFinalRecord> records, std::string& error) {
    return write_transactional(
        path, records, schema_for(RecordKind::Final),
        [](const RgbFinalRecord& record, std::byte* bytes) {
            std::string ignored;
            (void)encode_final(record, bytes, ignored);
        },
        [](const RgbFinalRecord& record, std::string& reason) {
            return validate_final_record(record, reason);
        },
        error);
}

bool read_rgb_chunk(
    const fs::path& path, std::vector<RgbChunkRecord>& records, std::string& error) {
    return read_records(path, schema_for(RecordKind::Chunk), records, error, decode_chunk);
}

bool read_rgb_final(
    const fs::path& path, std::vector<RgbFinalRecord>& records, std::string& error) {
    return read_records(path, schema_for(RecordKind::Final), records, error, decode_final);
}

bool validate_rgb_chunk_file(const fs::path& path, std::size_t expected_count, std::string& error) {
    std::vector<RgbChunkRecord> records;
    if (!read_rgb_chunk(path, records, error)) {
        return false;
    }
    if (records.size() != expected_count) {
        set_error(
            error, path_prefix(path, "validate") + "record count does not match expected count");
        return false;
    }
    return true;
}

bool validate_rgb_final_file(const fs::path& path, std::size_t expected_count, std::string& error) {
    std::vector<RgbFinalRecord> records;
    if (!read_rgb_final(path, records, error)) {
        return false;
    }
    if (records.size() != expected_count) {
        set_error(
            error, path_prefix(path, "validate") + "record count does not match expected count");
        return false;
    }
    return true;
}

} // namespace radar::fast_livo2::rgb

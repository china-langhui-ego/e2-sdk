#pragma once
#include <cstdint>
#include <cstddef>
#include <string>

namespace SXR {

// Minimal fragmented-MP4 (ISO BMFF) writer. Buffers samples and
// writes one fragment per GOP (auto-flush on I-frame boundary).
// Portable: POSIX file I/O only.
class FMP4Writer {
public:
    FMP4Writer();
    ~FMP4Writer();

    bool open(const std::string& path);

    // Codec config (csd) embedded in moov stsd. Call before start().
    // Video: csd0 = HEVC VPS+SPS+PPS (length-prefixed NALU), stored as hvcC.
    void setVideoTrack(int width, int height, int32_t timescale,
                       const uint8_t* csd0, size_t csd0Len,
                       int32_t sampleIntervalUs = 0);
    // Audio: raw PCM S16_LE. The SDK has no audio encoder (MI_AI is PCM-only),
    // so audio is stored raw — QuickTime "sowt" sample entry, no esds.
    // timescale = sampleRate; one writeSample() = one PCM chunk.
    void setAudioTrack(int sampleRate, int channels, int bitsPerSample);

    // Writes ftyp + moov (up front). Exactly one track must be set.
    // preallocBytes: if > 0, pre-allocate this many bytes to reduce
    // FAT cluster-chain growth overhead during recording.
    bool start(size_t preallocBytes = 0);

    // Buffer one encoded sample. Auto-flushes on GOP boundary (isSync).
    // ptsUs: presentation time in the track timescale (video: microseconds).
    bool writeSample(const uint8_t* data, size_t size,
                     int64_t ptsUs, bool isSync);

    // Force flush buffered samples as one fragment.
    bool flush();

    // Flush remaining + back-patch moov + close fd.
    bool close();

private:
    struct Impl;
    Impl* m;
};

} // namespace SXR

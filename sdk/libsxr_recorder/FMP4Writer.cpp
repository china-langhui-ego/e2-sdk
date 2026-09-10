#include "FMP4Writer.h"
#include "sxr_recorder.h"   /* sxr_fmp4_t (void*) for the C wrappers below */
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <cerrno>
#include <cstring>
#include <vector>

#include <cstdio>
#define FMP4_LOGE(...) do{fprintf(stderr,"E/FMP4Writer: " __VA_ARGS__); fputc('\n',stderr);}while(0)
#define FMP4_LOGW(...) do{fprintf(stderr,"W/FMP4Writer: " __VA_ARGS__); fputc('\n',stderr);}while(0)
#define FMP4_LOGI(...) do{fprintf(stdout,"I/FMP4Writer: " __VA_ARGS__); fputc('\n',stdout);}while(0)

namespace SXR {

// ---------------------------------------------------------------------------
// Big-endian writers into a byte buffer.
// ---------------------------------------------------------------------------
static void be32(std::vector<uint8_t>& b, uint32_t v) {
    b.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
    b.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
    b.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    b.push_back(static_cast<uint8_t>(v & 0xFF));
}
static void be16(std::vector<uint8_t>& b, uint16_t v) {
    b.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    b.push_back(static_cast<uint8_t>(v & 0xFF));
}
static void be64(std::vector<uint8_t>& b, uint64_t v) {
    be32(b, static_cast<uint32_t>(v >> 32));
    be32(b, static_cast<uint32_t>(v & 0xFFFFFFFFu));
}
static void bytes(std::vector<uint8_t>& b, const void* p, size_t n) {
    const uint8_t* q = static_cast<const uint8_t*>(p);
    b.insert(b.end(), q, q + n);
}
static void fourcc(std::vector<uint8_t>& b, const char tag[4]) {
    b.insert(b.end(), tag, tag + 4);
}

static std::vector<uint8_t> box(const char type[4], std::vector<uint8_t> payload) {
    std::vector<uint8_t> h;
    be32(h, static_cast<uint32_t>(8 + payload.size()));
    fourcc(h, type);
    h.insert(h.end(), payload.begin(), payload.end());
    return h;
}
static std::vector<uint8_t> fullbox(const char type[4], uint8_t version,
                                    uint32_t flags, std::vector<uint8_t> payload) {
    std::vector<uint8_t> body;
    body.push_back(version);
    body.push_back(static_cast<uint8_t>((flags >> 16) & 0xFF));
    body.push_back(static_cast<uint8_t>((flags >> 8) & 0xFF));
    body.push_back(static_cast<uint8_t>(flags & 0xFF));
    body.insert(body.end(), payload.begin(), payload.end());
    return box(type, std::move(body));
}

// ---------------------------------------------------------------------------
// HEVC hvcC construction (unchanged from original)
// ---------------------------------------------------------------------------
struct HvcCInfo {
    uint8_t configVersion = 1;
    uint8_t generalProfileSpace = 0;
    uint8_t generalTierFlag = 0;
    uint8_t generalProfileIdc = 0;
    uint32_t generalProfileCompat = 0;
    uint8_t generalConstraintFlags[6] = {0,0,0,0,0,0};
    uint8_t generalLevelIdc = 0;
    uint8_t minSpatialSegmentation = 0;
    uint8_t parallelismType = 0;
    uint8_t chromaFormat = 1;
    uint8_t bitDepthLuma = 0;
    uint8_t bitDepthChroma = 0;
    uint16_t avgFrameRate = 0;
    uint8_t constantFrameRate = 0;
    uint8_t numTemporalLayers = 1;
    uint8_t temporalIdNested = 1;
    uint8_t lengthSizeMinusOne = 3;
};

static bool looksLikeStartCode(const uint8_t* p) {
    return p[0] == 0x00 && p[1] == 0x00 && p[2] == 0x00 && p[3] == 0x01;
}
static void scanAnnexB(const uint8_t* data, size_t len,
                       std::vector<std::pair<uint8_t, std::vector<uint8_t>>>& out) {
    size_t i = 0;
    while (i + 4 <= len) {
        if (!looksLikeStartCode(data + i)) { ++i; continue; }
        size_t start = i + 4;
        size_t j = start;
        while (j + 4 <= len) {
            if (looksLikeStartCode(data + j)) break;
            ++j;
        }
        if (j + 4 > len) j = len;
        if (j > start) {
            const uint8_t* nalu = data + start;
            uint8_t nalType = (nalu[0] >> 1) & 0x3F;
            out.emplace_back(nalType, std::vector<uint8_t>(nalu, nalu + (j - start)));
        }
        i = j;
    }
}
static void extractNalus(const uint8_t* data, size_t len,
                         std::vector<std::pair<uint8_t, std::vector<uint8_t>>>& out) {
    if (len >= 4 && looksLikeStartCode(data)) {
        scanAnnexB(data, len, out);
        return;
    }
    size_t i = 0;
    std::vector<std::pair<uint8_t, std::vector<uint8_t>>> tmp;
    bool ok = true;
    while (i + 4 <= len) {
        uint32_t n = (static_cast<uint32_t>(data[i]) << 24) |
                     (static_cast<uint32_t>(data[i + 1]) << 16) |
                     (static_cast<uint32_t>(data[i + 2]) << 8) |
                     (static_cast<uint32_t>(data[i + 3]));
        if (n == 0 || i + 4 + n > len) { ok = false; break; }
        const uint8_t* nalu = data + i + 4;
        uint8_t nalType = (nalu[0] >> 1) & 0x3F;
        tmp.emplace_back(nalType, std::vector<uint8_t>(nalu, nalu + n));
        i += 4 + n;
    }
    if (ok && !tmp.empty()) { out = std::move(tmp); return; }
    scanAnnexB(data, len, out);
}

static std::vector<uint8_t> buildHvcC(const uint8_t* csd, size_t csdLen,
                                      int width, int height, int32_t timescale) {
    std::vector<std::pair<uint8_t, std::vector<uint8_t>>> nalus;
    extractNalus(csd, csdLen, nalus);
    HvcCInfo info;
    bool haveProfile = false;
    for (auto& nv : nalus) {
        uint8_t t = nv.first;
        const auto& nalu = nv.second;
        if ((t == 32 || t == 33 || t == 34) && !haveProfile && nalu.size() >= 13) {
            const uint8_t* p = nalu.data();
            info.generalProfileSpace  = (p[2] >> 6) & 0x3;
            info.generalTierFlag      = (p[2] >> 5) & 0x1;
            info.generalProfileIdc    = p[2] & 0x1F;
            info.generalProfileCompat = (static_cast<uint32_t>(p[3]) << 24) |
                                        (static_cast<uint32_t>(p[4]) << 16) |
                                        (static_cast<uint32_t>(p[5]) << 8) |
                                        (static_cast<uint32_t>(p[6]));
            for (int k = 0; k < 6; ++k) info.generalConstraintFlags[k] = p[7 + k];
            info.generalLevelIdc = p[13];
            haveProfile = true;
        }
    }
    std::vector<uint8_t> b;
    b.push_back(info.configVersion);
    b.push_back(static_cast<uint8_t>(
        ((info.generalProfileSpace & 0x3) << 6) |
        ((info.generalTierFlag & 0x1) << 5) |
        (info.generalProfileIdc & 0x1F)));
    be32(b, info.generalProfileCompat);
    for (int k = 0; k < 6; ++k) b.push_back(info.generalConstraintFlags[k]);
    b.push_back(info.generalLevelIdc);
    be16(b, 0xF000 | (info.minSpatialSegmentation & 0x0F));
    b.push_back(0xFC | (info.parallelismType & 0x03));
    b.push_back(0xFC | (info.chromaFormat & 0x03));
    b.push_back(0xF8 | (info.bitDepthLuma & 0x07));
    b.push_back(0xF8 | (info.bitDepthChroma & 0x07));
    be16(b, info.avgFrameRate);
    b.push_back(static_cast<uint8_t>(
        ((info.constantFrameRate & 0x3) << 6) |
        ((info.numTemporalLayers & 0x7) << 3) |
        ((info.temporalIdNested & 0x1) << 2) |
        (info.lengthSizeMinusOne & 0x3)));
    auto emitArray = [&](uint8_t nalType,
                         const std::vector<const std::vector<uint8_t>*>& items) {
        b.push_back(static_cast<uint8_t>(0x80 | (nalType & 0x3F)));
        be16(b, static_cast<uint16_t>(items.size()));
        for (auto* nalu : items) {
            be16(b, static_cast<uint16_t>(nalu->size()));
            bytes(b, nalu->data(), nalu->size());
        }
    };
    std::vector<const std::vector<uint8_t>*> vps, sps, pps, other;
    for (auto& nv : nalus) {
        if (nv.first == 32) vps.push_back(&nv.second);
        else if (nv.first == 33) sps.push_back(&nv.second);
        else if (nv.first == 34) pps.push_back(&nv.second);
        else other.push_back(&nv.second);
    }
    if (vps.empty() && sps.empty() && pps.empty()) {
        std::vector<const std::vector<uint8_t>*> all;
        for (auto& nv : nalus) all.push_back(&nv.second);
        b.push_back(1);
        emitArray(0, all);
    } else {
        uint8_t numArrays = 0;
        if (!vps.empty()) ++numArrays;
        if (!sps.empty()) ++numArrays;
        if (!pps.empty()) ++numArrays;
        if (!other.empty()) ++numArrays;
        b.push_back(numArrays);
        if (!vps.empty())   emitArray(32, vps);
        if (!sps.empty())   emitArray(33, sps);
        if (!pps.empty())   emitArray(34, pps);
        if (!other.empty()) emitArray(0, other);
    }
    (void)width; (void)height; (void)timescale;
    return box("hvcC", std::move(b));
}

// ---------------------------------------------------------------------------
// Annex-B -> length-prefixed conversion (unchanged)
// ---------------------------------------------------------------------------
static bool is3ByteStartCode(const uint8_t* p) {
    return p[0] == 0x00 && p[1] == 0x00 && p[2] == 0x01;
}
static bool annexBToLengthPrefixed(const uint8_t* data, size_t len,
                                   std::vector<uint8_t>& out) {
    if (len < 3) return false;
    const bool startsAnnexB =
        (len >= 4 && looksLikeStartCode(data)) || is3ByteStartCode(data);
    if (!startsAnnexB) return false;
    out.clear();
    out.reserve(len + 16);
    size_t i = 0;
    while (i < len) {
        bool sc4 = (i + 4 <= len) && looksLikeStartCode(data + i);
        bool sc3 = !sc4 && (i + 3 <= len) && is3ByteStartCode(data + i);
        if (!sc4 && !sc3) { ++i; continue; }
        size_t hdrLen = sc4 ? 4 : 3;
        size_t naluStart = i + hdrLen;
        size_t j = naluStart;
        while (j < len) {
            bool j4 = (j + 4 <= len) && looksLikeStartCode(data + j);
            bool j3 = !j4 && (j + 3 <= len) && is3ByteStartCode(data + j);
            if (j4 || j3) break;
            ++j;
        }
        size_t naluLen = j - naluStart;
        if (naluLen > 0) {
            be32(out, static_cast<uint32_t>(naluLen));
            bytes(out, data + naluStart, naluLen);
        }
        i = j;
    }
    if (out.empty()) return false;
    return true;
}

// ---------------------------------------------------------------------------
// Impl — with sample buffering for batched fragments
// ---------------------------------------------------------------------------
struct FMP4Writer::Impl {
    int fd = -1;
    bool started = false;
    bool closed = false;
    size_t preallocSize = 0;   // preallocation size, 0 = disabled

    bool hasVideo = false;
    bool hasAudio = false;
    bool isAudio = false;
    int width = 0, height = 0;
    int32_t timescale = 1000000;
    int32_t sampleIntervalUs = 0;
    int64_t firstPtsUs = -1;
    int64_t lastPtsUs = 0;
    int32_t totalSamples = 0;
    off_t moovFilePos = 0;
    size_t moovSize = 0;
    std::vector<uint8_t> csd0;
    int sampleRate = 0, channels = 0;
    int bytesPerSample = 2;   // audio bytes/sample (S16_LE => 2), for sample-duration calc

    uint32_t seqNumber = 0;
    int64_t prevPtsUs = -1;

    // ---- sample buffering for batched GOP fragments ----
    struct BufferedSample {
        std::vector<uint8_t> data;
        uint32_t         duration;   // in track timescale
        uint32_t         sampleFlags;
        bool             isSync;
    };
    std::vector<BufferedSample> pending;
    int64_t bufferedFirstPts = -1;  // first-sample PTS in current batch

    bool writeAll(const void* p, size_t n) {
        const uint8_t* q = static_cast<const uint8_t*>(p);
        size_t off = 0;
        while (off < n) {
            ssize_t w = ::write(fd, q + off, n - off);
            if (w < 0) { if (errno == EINTR) continue; FMP4_LOGE("write failed. ret: %d errno: %d", w, errno); return false; }
            off += static_cast<size_t>(w);
        }
        return true;
    }
    bool writeBuf(const std::vector<uint8_t>& b) { return writeAll(b.data(), b.size()); }
};

FMP4Writer::FMP4Writer() : m(new Impl()) {}
FMP4Writer::~FMP4Writer() { close(); delete m; }

bool FMP4Writer::open(const std::string& path) {
    if (m->fd >= 0) { FMP4_LOGE("already open"); return false; }
    m->fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_LARGEFILE, 0644);
    FMP4_LOGI("FMP4Writer open %s with O_WRONLY | O_CREAT | O_TRUNC | O_LARGEFILE", path.c_str());
    if (m->fd < 0) { FMP4_LOGE("open failed: %s", path.c_str()); return false; }
    m->started = false; m->closed = false;
    return true;
}

void FMP4Writer::setVideoTrack(int width, int height, int32_t timescale,
                               const uint8_t* csd0, size_t csd0Len,
                               int32_t sampleIntervalUs) {
    m->hasVideo = true;
    m->width = width; m->height = height; m->timescale = timescale;
    m->sampleIntervalUs = sampleIntervalUs;
    m->csd0.assign(csd0, csd0 + csd0Len);
}

void FMP4Writer::setAudioTrack(int sampleRate, int channels, int bitsPerSample) {
    m->hasAudio = true;
    m->isAudio = true;
    m->sampleRate = sampleRate;
    m->channels = channels;
    m->timescale = sampleRate;            // audio timescale = sample rate
    m->bytesPerSample = (bitsPerSample > 0) ? bitsPerSample / 8 : 2;
    // raw PCM: no codec config (csd0 left empty); sample entry built as "sowt".
}

// ---------------------------------------------------------------------------
// ftyp
// ---------------------------------------------------------------------------
static std::vector<uint8_t> buildFtyp() {
    std::vector<uint8_t> b;
    fourcc(b, "iso5");
    be32(b, 0);
    fourcc(b, "iso5");
    fourcc(b, "iso6");
    fourcc(b, "mp41");
    fourcc(b, "dash");
    return box("ftyp", std::move(b));
}

// ---------------------------------------------------------------------------
// moov and children (unchanged)
// ---------------------------------------------------------------------------
static std::vector<uint8_t> buildMvhd(int64_t durationMs) {
    std::vector<uint8_t> b;
    be32(b, 0); be32(b, 0);
    be32(b, 1000);
    be32(b, static_cast<uint32_t>(durationMs));
    be32(b, 0x00010000);
    be16(b, 0x0100);
    be16(b, 0);
    be32(b, 0); be32(b, 0);
    be32(b, 0x00010000); be32(b, 0); be32(b, 0);
    be32(b, 0); be32(b, 0x00010000); be32(b, 0);
    be32(b, 0); be32(b, 0); be32(b, 0x40000000);
    for (int i = 0; i < 6; ++i) be32(b, 0);
    be32(b, 2);
    return fullbox("mvhd", 0, 0, std::move(b));
}

static std::vector<uint8_t> buildTkhd(int width, int height, int64_t durationUs) {
    std::vector<uint8_t> b;
    be32(b, 0); be32(b, 0);
    be32(b, 1);
    be32(b, 0);
    be32(b, static_cast<uint32_t>(durationUs));
    be32(b, 0); be32(b, 0);
    be16(b, 0); be16(b, 0);
    be16(b, 0); be16(b, 0);
    be32(b, 0x00010000); be32(b, 0); be32(b, 0);
    be32(b, 0); be32(b, 0x00010000); be32(b, 0);
    be32(b, 0); be32(b, 0); be32(b, 0x40000000);
    be32(b, (static_cast<uint32_t>(width)  << 16));
    be32(b, (static_cast<uint32_t>(height) << 16));
    return fullbox("tkhd", 0, 0x0007, std::move(b));
}

static std::vector<uint8_t> buildMdhd(int32_t timescale, int64_t duration) {
    std::vector<uint8_t> b;
    be32(b, 0); be32(b, 0);
    be32(b, static_cast<uint32_t>(timescale));
    be32(b, static_cast<uint32_t>(duration));
    be16(b, 0x55C4);
    be16(b, 0);
    return fullbox("mdhd", 0, 0, std::move(b));
}

static std::vector<uint8_t> buildHdlr(const char* handlerType) {
    std::vector<uint8_t> b;
    be32(b, 0);
    bytes(b, handlerType, 4);
    be32(b, 0); be32(b, 0); be32(b, 0);
    const char name[] = "";
    bytes(b, name, sizeof(name));
    return fullbox("hdlr", 0, 0, std::move(b));
}

static std::vector<uint8_t> buildVmhd() {
    std::vector<uint8_t> b;
    be16(b, 0);
    be16(b, 0); be16(b, 0); be16(b, 0);
    return fullbox("vmhd", 0, 1, std::move(b));
}

static std::vector<uint8_t> buildDref() {
    std::vector<uint8_t> urlBody;
    std::vector<uint8_t> urlBox = fullbox("url ", 0, 1, std::move(urlBody));
    std::vector<uint8_t> drefBody;
    be32(drefBody, 1);
    drefBody.insert(drefBody.end(), urlBox.begin(), urlBox.end());
    return fullbox("dref", 0, 0, std::move(drefBody));
}

static std::vector<uint8_t> buildDinf() {
    std::vector<uint8_t> dref = buildDref();
    std::vector<uint8_t> dinf;
    dinf.insert(dinf.end(), dref.begin(), dref.end());
    return box("dinf", std::move(dinf));
}

static std::vector<uint8_t> buildStsd(const uint8_t* csd, size_t csdLen,
                                      int width, int height, int32_t timescale) {
    std::vector<uint8_t> hvc1;
    for (int i = 0; i < 6; ++i) hvc1.push_back(0);
    be16(hvc1, 1);
    be16(hvc1, 0); be16(hvc1, 0);
    for (int i = 0; i < 12; ++i) hvc1.push_back(0);
    be16(hvc1, static_cast<uint16_t>(width));
    be16(hvc1, static_cast<uint16_t>(height));
    be32(hvc1, 0x00480000);
    be32(hvc1, 0x00480000);
    be32(hvc1, 0);
    be16(hvc1, 1);
    hvc1.push_back(0);
    for (int i = 0; i < 31; ++i) hvc1.push_back(0);
    be16(hvc1, 0x0018);
    be16(hvc1, 0xFFFF);
    std::vector<uint8_t> hvcc = buildHvcC(csd, csdLen, width, height, timescale);
    hvc1.insert(hvc1.end(), hvcc.begin(), hvcc.end());
    std::vector<uint8_t> hvc1Box = box("hvc1", std::move(hvc1));
    std::vector<uint8_t> stsdBody;
    be32(stsdBody, 1);
    stsdBody.insert(stsdBody.end(), hvc1Box.begin(), hvc1Box.end());
    return fullbox("stsd", 0, 0, std::move(stsdBody));
}

// AAC esds/mp4a/audio stsd (unchanged)
static void putVarSize(std::vector<uint8_t>& b, uint32_t size) {
    if (size < 0x80) { b.push_back(static_cast<uint8_t>(size & 0x7F)); return; }
    uint8_t tmp[5];
    int n = 0;
    tmp[n++] = static_cast<uint8_t>(size & 0x7F);
    size >>= 7;
    while (size > 0) { tmp[n++] = static_cast<uint8_t>(0x80 | (size & 0x7F)); size >>= 7; }
    for (int i = n - 1; i >= 0; --i) b.push_back(tmp[i]);
}
static std::vector<uint8_t> buildEsds(const uint8_t* csd, size_t csdLen, int, int) {
    std::vector<uint8_t> dsi;
    dsi.push_back(0x05);
    putVarSize(dsi, static_cast<uint32_t>(csdLen));
    bytes(dsi, csd, csdLen);
    std::vector<uint8_t> dcdContent;
    dcdContent.push_back(0x40);
    dcdContent.push_back(0x15);
    dcdContent.push_back(0x00); dcdContent.push_back(0x00); dcdContent.push_back(0x00);
    be32(dcdContent, 0); be32(dcdContent, 0);
    dcdContent.insert(dcdContent.end(), dsi.begin(), dsi.end());
    std::vector<uint8_t> dcd;
    dcd.push_back(0x04);
    putVarSize(dcd, static_cast<uint32_t>(dcdContent.size()));
    dcd.insert(dcd.end(), dcdContent.begin(), dcdContent.end());
    std::vector<uint8_t> slc;
    slc.push_back(0x06); putVarSize(slc, 1); slc.push_back(0x02);
    std::vector<uint8_t> esdContent;
    be16(esdContent, 1); esdContent.push_back(0x00);
    esdContent.insert(esdContent.end(), dcd.begin(), dcd.end());
    esdContent.insert(esdContent.end(), slc.begin(), slc.end());
    std::vector<uint8_t> esd;
    esd.push_back(0x03);
    putVarSize(esd, static_cast<uint32_t>(esdContent.size()));
    esd.insert(esd.end(), esdContent.begin(), esdContent.end());
    return fullbox("esds", 0, 0, std::move(esd));
}
static std::vector<uint8_t> buildMp4a(const uint8_t* csd, size_t csdLen,
                                      int sampleRate, int channels) {
    std::vector<uint8_t> mp4a;
    for (int i = 0; i < 6; ++i) mp4a.push_back(0);
    be16(mp4a, 1);
    be32(mp4a, 0); be32(mp4a, 0);
    be16(mp4a, static_cast<uint16_t>(channels));
    be16(mp4a, 16);
    be16(mp4a, 0); be16(mp4a, 0);
    be32(mp4a, static_cast<uint32_t>(sampleRate) << 16);
    std::vector<uint8_t> esds = buildEsds(csd, csdLen, sampleRate, channels);
    mp4a.insert(mp4a.end(), esds.begin(), esds.end());
    return box("mp4a", std::move(mp4a));
}
// Raw 16-bit signed LE PCM audio sample entry (QuickTime/ISO "sowt"). No esds:
// the SDK has no audio encoder, so audio is raw PCM only.
static std::vector<uint8_t> buildRawAudioEntry(int sampleRate, int channels) {
    std::vector<uint8_t> e;
    for (int i = 0; i < 6; ++i) e.push_back(0);       // reserved
    be16(e, 1);                                        // data_reference_index
    be32(e, 0); be32(e, 0);                            // reserved
    be16(e, static_cast<uint16_t>(channels));          // channelcount
    be16(e, 16);                                       // samplesize (S16_LE fixed)
    be16(e, 0); be16(e, 0);                            // pre_defined + reserved
    be32(e, static_cast<uint32_t>(sampleRate) << 16);  // samplerate (hi16.16)
    return box("sowt", std::move(e));
}
static std::vector<uint8_t> buildAudioStsd(int sampleRate, int channels) {
    std::vector<uint8_t> entry = buildRawAudioEntry(sampleRate, channels);
    std::vector<uint8_t> stsdBody;
    be32(stsdBody, 1);
    stsdBody.insert(stsdBody.end(), entry.begin(), entry.end());
    return fullbox("stsd", 0, 0, std::move(stsdBody));
}
static std::vector<uint8_t> buildSmhd() {
    std::vector<uint8_t> b;
    be16(b, 0); be16(b, 0);
    return fullbox("smhd", 0, 0, std::move(b));
}
static std::vector<uint8_t> buildStbl(const uint8_t* csd, size_t csdLen,
                                      int width, int height, int32_t timescale,
                                      bool isAudio, int sampleRate, int channels,
                                      int32_t sampleIntervalUs) {
    std::vector<uint8_t> stbl;
    auto append = [&](const std::vector<uint8_t>& v) { stbl.insert(stbl.end(), v.begin(), v.end()); };
    if (isAudio)
        append(buildAudioStsd(sampleRate, channels));
    else
        append(buildStsd(csd, csdLen, width, height, timescale));
    std::vector<uint8_t> sttsP;
    if (!isAudio && sampleIntervalUs > 0) {
        be32(sttsP, 1);
        be32(sttsP, 1);
        be32(sttsP, static_cast<uint32_t>(sampleIntervalUs));
    } else {
        be32(sttsP, 0);
    }
    append(fullbox("stts", 0, 0, std::move(sttsP)));
    std::vector<uint8_t> stscP; be32(stscP, 0);
    append(fullbox("stsc", 0, 0, std::move(stscP)));
    std::vector<uint8_t> stszP; be32(stszP, 0); be32(stszP, 0);
    append(fullbox("stsz", 0, 0, std::move(stszP)));
    std::vector<uint8_t> stcoP; be32(stcoP, 0);
    append(fullbox("stco", 0, 0, std::move(stcoP)));
    return box("stbl", std::move(stbl));
}
static std::vector<uint8_t> buildMinf(const uint8_t* csd, size_t csdLen,
                                      int width, int height, int32_t timescale,
                                      bool isAudio, int sampleRate, int channels,
                                      int32_t sampleIntervalUs) {
    std::vector<uint8_t> minf;
    auto append = [&](const std::vector<uint8_t>& v) { minf.insert(minf.end(), v.begin(), v.end()); };
    append(isAudio ? buildSmhd() : buildVmhd());
    append(buildDinf());
    append(buildStbl(csd, csdLen, width, height, timescale, isAudio, sampleRate, channels, sampleIntervalUs));
    return box("minf", std::move(minf));
}
static std::vector<uint8_t> buildMdia(const uint8_t* csd, size_t csdLen,
                                      int width, int height, int32_t timescale,
                                      bool isAudio, int sampleRate, int channels,
                                      int32_t sampleIntervalUs, int64_t durationUs) {
    std::vector<uint8_t> mdia;
    auto append = [&](const std::vector<uint8_t>& v) { mdia.insert(mdia.end(), v.begin(), v.end()); };
    append(buildMdhd(timescale, durationUs));
    append(buildHdlr(isAudio ? "soun" : "vide"));
    append(buildMinf(csd, csdLen, width, height, timescale, isAudio, sampleRate, channels, sampleIntervalUs));
    return box("mdia", std::move(mdia));
}
static std::vector<uint8_t> buildTrak(const uint8_t* csd, size_t csdLen,
                                      int width, int height, int32_t timescale,
                                      bool isAudio, int sampleRate, int channels,
                                      int32_t sampleIntervalUs, int64_t durationUs) {
    std::vector<uint8_t> trak;
    auto append = [&](const std::vector<uint8_t>& v) { trak.insert(trak.end(), v.begin(), v.end()); };
    append(buildTkhd(isAudio ? 0 : width, isAudio ? 0 : height, durationUs));
    append(buildMdia(csd, csdLen, width, height, timescale, isAudio, sampleRate, channels, sampleIntervalUs, durationUs));
    return box("trak", std::move(trak));
}
static std::vector<uint8_t> buildMvex() {
    std::vector<uint8_t> trexBody;
    be32(trexBody, 1);
    be32(trexBody, 1);
    be32(trexBody, 0);
    be32(trexBody, 0);
    be32(trexBody, 0x02000000);
    std::vector<uint8_t> trex = fullbox("trex", 0, 0, std::move(trexBody));
    std::vector<uint8_t> mvex;
    mvex.insert(mvex.end(), trex.begin(), trex.end());
    return box("mvex", std::move(mvex));
}
static std::vector<uint8_t> buildMoov(const uint8_t* csd, size_t csdLen,
                                      int width, int height, int32_t timescale,
                                      bool isAudio, int sampleRate, int channels,
                                      int32_t sampleIntervalUs, int64_t durationUs) {
    std::vector<uint8_t> moov;
    auto append = [&](const std::vector<uint8_t>& v) { moov.insert(moov.end(), v.begin(), v.end()); };
    append(buildMvhd(durationUs / 1000));
    append(buildTrak(csd, csdLen, width, height, timescale, isAudio, sampleRate, channels, sampleIntervalUs, durationUs));
    append(buildMvex());
    return box("moov", std::move(moov));
}

bool FMP4Writer::start(size_t preallocBytes) {
    if (m->fd < 0) { FMP4_LOGE("start: not open"); return false; }
    if (m->started) { FMP4_LOGE("start: already started"); return false; }
    if (!m->hasVideo && !m->hasAudio) { FMP4_LOGE("start: no track set"); return false; }

    // Pre-allocate file to avoid FAT cluster-chain growth overhead
    if (preallocBytes > 0) {
        off_t curPos = ::lseek(m->fd, 0, SEEK_CUR);
        // Try posix_fallocate first, fall back to lseek+write trick
        int fallocRet = ::posix_fallocate(m->fd, 0, (off_t)preallocBytes);
        if (fallocRet == 0) {
            m->preallocSize = preallocBytes;
            ::lseek(m->fd, curPos, SEEK_SET);  // restore write position
            FMP4_LOGI("preallocated %zu bytes (posix_fallocate)", preallocBytes);
        } else {
            // Fallback: seek to end-1, write a byte, seek back
            if (::lseek(m->fd, (off_t)(preallocBytes - 1), SEEK_SET) != (off_t)-1) {
                if (::write(m->fd, "\0", 1) == 1) {
                    ::lseek(m->fd, curPos, SEEK_SET);
                    m->preallocSize = preallocBytes;
                    FMP4_LOGI("preallocated %zu bytes (lseek+write)", preallocBytes);
                } else {
                    ::lseek(m->fd, curPos, SEEK_SET);
                    FMP4_LOGW("prealloc write failed: %s", strerror(errno));
                }
            }
        }
    }

    if (!m->writeBuf(buildFtyp())) return false;
    m->moovFilePos = static_cast<off_t>(::lseek(m->fd, 0, SEEK_CUR));
    auto moov = buildMoov(m->csd0.data(), m->csd0.size(),
                           m->width, m->height, m->timescale,
                           m->isAudio, m->sampleRate, m->channels,
                           m->sampleIntervalUs, 0);
    m->moovSize = moov.size();
    if (!m->writeBuf(moov)) return false;
    m->started = true;
    return true;
}

// ---------------------------------------------------------------------------
// Fragment building — multi-sample trun
// ---------------------------------------------------------------------------
static std::vector<uint8_t> buildStyp() {
    std::vector<uint8_t> b;
    fourcc(b, "iso5");
    be32(b, 0);
    fourcc(b, "iso5");
    fourcc(b, "iso6");
    fourcc(b, "msdh");
    return box("styp", std::move(b));
}
static std::vector<uint8_t> buildMfhd(uint32_t sequenceNumber) {
    std::vector<uint8_t> b;
    be32(b, sequenceNumber);
    return fullbox("mfhd", 0, 0, std::move(b));
}
static std::vector<uint8_t> buildTfhd() {
    std::vector<uint8_t> b;
    be32(b, 1);
    return fullbox("tfhd", 0, 0x020000, std::move(b));
}
static std::vector<uint8_t> buildTfdt(uint64_t baseMediaDecodeTime) {
    std::vector<uint8_t> b;
    be64(b, baseMediaDecodeTime);
    return fullbox("tfdt", 1, 0, std::move(b));
}

// Multi-sample trun: flags = data-offset | sample-duration | sample-size | sample-flags = 0x000701
// Returns (trunBox, dataOffsetPos) — caller patches data_offset after moof sizing.
static std::pair<std::vector<uint8_t>, size_t> buildTrunMulti(
    const std::vector<uint32_t>& durations,
    const std::vector<uint32_t>& sizes,
    const std::vector<uint32_t>& flags) {
    const uint32_t count = static_cast<uint32_t>(durations.size());
    std::vector<uint8_t> body;
    body.push_back(1);  // version 1
    body.push_back(0x00); body.push_back(0x07); body.push_back(0x01);  // flags 0x000701
    be32(body, count);  // sample_count
    size_t dataOffsetPos = body.size();
    be32(body, 0);  // data_offset placeholder
    for (uint32_t i = 0; i < count; i++) {
        be32(body, durations[i]);
        be32(body, sizes[i]);
        be32(body, flags[i]);
    }
    dataOffsetPos += 8;  // adjust for box() 8-byte header
    return {box("trun", std::move(body)), dataOffsetPos};
}

// ---------------------------------------------------------------------------
// flush — emit one fragment for all buffered samples
// ---------------------------------------------------------------------------
bool FMP4Writer::flush() {
    if (m->pending.empty()) return true;

    const size_t ns = m->pending.size();
    std::vector<uint32_t> durations(ns), sizes(ns), sampleFlags(ns);

    size_t totalMdat = 0;
    for (size_t i = 0; i < ns; i++) {
        durations[i] = m->pending[i].duration;
        sizes[i] = static_cast<uint32_t>(m->pending[i].data.size());
        sampleFlags[i] = m->pending[i].sampleFlags;
        totalMdat += sizes[i];
    }

    auto [trun, dataOffsetPos] = buildTrunMulti(durations, sizes, sampleFlags);

    std::vector<uint8_t> traf;
    auto appendT = [&](const std::vector<uint8_t>& v) {
        traf.insert(traf.end(), v.begin(), v.end());
    };
    appendT(buildTfhd());
    appendT(buildTfdt(static_cast<uint64_t>(
        m->bufferedFirstPts < 0 ? 0 : m->bufferedFirstPts)));
    appendT(trun);
    traf = box("traf", std::move(traf));

    std::vector<uint8_t> moofPayload;
    auto appendM = [&](const std::vector<uint8_t>& v) {
        moofPayload.insert(moofPayload.end(), v.begin(), v.end());
    };
    appendM(buildMfhd(++m->seqNumber));
    appendM(traf);

    const uint32_t moofTotalSize = static_cast<uint32_t>(8 + moofPayload.size());
    const int32_t dataOffset = static_cast<int32_t>(moofTotalSize) + 8;

    // Patch data_offset in trun
    const size_t trunStart = moofPayload.size() - trun.size();
    const size_t patchPos = trunStart + dataOffsetPos;
    if (patchPos + 4 <= moofPayload.size()) {
        uint32_t v = static_cast<uint32_t>(dataOffset);
        moofPayload[patchPos + 0] = static_cast<uint8_t>((v >> 24) & 0xFF);
        moofPayload[patchPos + 1] = static_cast<uint8_t>((v >> 16) & 0xFF);
        moofPayload[patchPos + 2] = static_cast<uint8_t>((v >> 8) & 0xFF);
        moofPayload[patchPos + 3] = static_cast<uint8_t>(v & 0xFF);
    }

    // Write: styp + moof + mdat
    if (!m->writeBuf(buildStyp())) return false;
    std::vector<uint8_t> moofHdr;
    be32(moofHdr, moofTotalSize);
    fourcc(moofHdr, "moof");
    if (!m->writeBuf(moofHdr)) return false;
    if (!m->writeBuf(moofPayload)) return false;

    std::vector<uint8_t> mdatHdr;
    be32(mdatHdr, static_cast<uint32_t>(8 + totalMdat));
    fourcc(mdatHdr, "mdat");
    if (!m->writeBuf(mdatHdr)) return false;
    for (size_t i = 0; i < ns; i++) {
        if (!m->writeAll(m->pending[i].data.data(), m->pending[i].data.size()))
            return false;
    }

    m->pending.clear();
    m->bufferedFirstPts = -1;
    return true;
}

// ---------------------------------------------------------------------------
// writeSample — buffers, auto-flushes on GOP boundary
// ---------------------------------------------------------------------------
bool FMP4Writer::writeSample(const uint8_t* data, size_t size,
                              int64_t ptsUs, bool isSync) {
    if (!m->started) { FMP4_LOGE("writeSample: not started"); return false; }
    if (m->fd < 0) { FMP4_LOGE("writeSample: no file"); return false; }
    if (size == 0) { FMP4_LOGE("writeSample: empty sample"); return false; }

    // Annex-B -> length-prefixed conversion for video
    std::vector<uint8_t> converted;
    const uint8_t* sampleData = data;
    size_t sampleSize = size;
    if (!m->isAudio) {
        if (annexBToLengthPrefixed(data, size, converted)) {
            sampleData = converted.data();
            sampleSize = converted.size();
        }
    }

    int64_t pts = ptsUs;

    if (m->prevPtsUs >= 0 && pts < m->prevPtsUs) {
        FMP4_LOGW("non-monotonic PTS: prev=%lld us < cur=%lld us; check B-frames",
                  static_cast<long long>(m->prevPtsUs),
                  static_cast<long long>(pts));
    }

    /* Sample duration from actual VENC PTS delta.  VENC PTS is clean
     * (stddev ~8us); large deltas are REAL sensor frame drops (verified
     * via drv_frame_seq gaps in metainfo.csv) and must be preserved for
     * timestamp accuracy.  Forcing fixed 33333us would hide frame drops. */
    uint32_t duration;
    if (m->isAudio) {
        // PCM frames in this chunk = bytes / (channels * bytesPerSample)
        duration = (m->channels > 0 && m->bytesPerSample > 0)
                   ? static_cast<uint32_t>(size / ((uint32_t)m->channels * m->bytesPerSample))
                   : 0u;
    } else {
        duration = (m->prevPtsUs < 0 || pts <= m->prevPtsUs)
                       ? 33333u
                       : static_cast<uint32_t>(pts - m->prevPtsUs);
    }

    /* Sample flags (ISO 14496-12 trun):
     *   I-frame: sample_depends_on=2, sample_is_non_sync=0 -> 0x90000000
     *   P-frame: sample_depends_on=1, sample_is_non_sync=1 -> 0x50040000 */
    uint32_t sampleFlags = isSync ? 0x90000000u : 0x50040000u;

    // Auto-flush on GOP boundary (new sync frame and buffer not empty)
    if (isSync && !m->pending.empty()) {
        if (!flush()) return false;
    }

    // Audio has no sync frames; flush ~every 16 chunks (~1s) to bound memory.
    if (m->isAudio && m->pending.size() >= 16) {
        if (!flush()) return false;
    }

    // Buffer this sample
    Impl::BufferedSample s;
    s.data.assign(sampleData, sampleData + sampleSize);
    s.duration = duration;
    s.sampleFlags = sampleFlags;
    s.isSync = isSync;
    if (m->pending.empty()) m->bufferedFirstPts = pts;
    m->pending.push_back(std::move(s));

    if (!m->isAudio) {
        if (m->totalSamples == 0) m->firstPtsUs = pts;
        m->lastPtsUs = pts;
        m->totalSamples++;
    }
    m->prevPtsUs = pts;
    return true;
}

// ---------------------------------------------------------------------------
// close — flush remaining + back-patch moov
// ---------------------------------------------------------------------------
bool FMP4Writer::close() {
    if (m->closed) return true;

    // Flush any buffered samples and record end-of-data position
    if (!flush()) return false;
    off_t endOfData = ::lseek(m->fd, 0, SEEK_CUR);  // save before moov back-patch

    // Back-patch moov
    if (!m->isAudio && m->totalSamples >= 1 && m->moovFilePos > 0 && m->moovSize > 0) {
        int32_t realIntervalUs = m->sampleIntervalUs;
        if (m->totalSamples >= 2) {
            int64_t spanUs = m->lastPtsUs - m->firstPtsUs;
            if (spanUs > 0)
                realIntervalUs = static_cast<int32_t>(spanUs / (m->totalSamples - 1));
        }
        int64_t durationUs = m->lastPtsUs - m->firstPtsUs;
        auto rebuilt = buildMoov(m->csd0.data(), m->csd0.size(),
                                  m->width, m->height, m->timescale,
                                  m->isAudio, m->sampleRate, m->channels,
                                  realIntervalUs, durationUs);
        if (rebuilt.size() == m->moovSize) {
            ::lseek(m->fd, m->moovFilePos, SEEK_SET);
            m->writeBuf(rebuilt);
            FMP4_LOGI("moov back-patched: avgInterval=%d us, duration=%lld us, "
                      "samples=%d",
                      realIntervalUs, static_cast<long long>(durationUs),
                      m->totalSamples);
        } else {
            FMP4_LOGW("moov size mismatch on close (old=%zu new=%zu); skipping",
                      m->moovSize, rebuilt.size());
        }
    }
    if (m->fd >= 0) {
        // Truncate to actual size if we pre-allocated
        if (m->preallocSize > 0 && endOfData > 0) {
            ::ftruncate(m->fd, endOfData);
            FMP4_LOGI("truncated to %lld bytes (prealloc was %zu)",
                      (long long)endOfData, m->preallocSize);
        }
        ::close(m->fd); m->fd = -1;
    }
    return true;
}

} // namespace SXR

/* ===================================================================
 * Public C wrappers (sxr_fmp4_*_impl) — bridge between the C API in
 * sxr_recorder.c and the SXR::FMP4Writer C++ class. Relocated from
 * mi_pipeline.cpp (Plan 2b Task 5) so that deleting mi_pipeline.cpp
 * leaves the fMP4 path intact and recorder-side (no MI dependency).
 *
 * sxr_recorder.c forwards the public sxr_fmp4_* symbols to these _impl
 * functions; they are the only definitions (no duplicates elsewhere).
 * =================================================================== */
extern "C" {

sxr_fmp4_t sxr_fmp4_open_impl(const char *path)
{
    SXR::FMP4Writer *w = new SXR::FMP4Writer();
    if (!w->open(std::string(path))) {
        delete w;
        return NULL;
    }
    return (sxr_fmp4_t)w;
}

int sxr_fmp4_set_video_track_impl(sxr_fmp4_t f, int width, int height, int timescale,
                                  const uint8_t *csd, size_t csd_len, int sample_interval_us)
{
    if (!f) return -1;
    SXR::FMP4Writer *w = (SXR::FMP4Writer *)f;
    w->setVideoTrack(width, height, timescale, csd, csd_len, sample_interval_us);
    return 0;
}

int sxr_fmp4_set_audio_track_impl(sxr_fmp4_t f, int sample_rate, int channels, int bits_per_sample)
{
    if (!f) return -1;
    SXR::FMP4Writer *w = (SXR::FMP4Writer *)f;
    w->setAudioTrack(sample_rate, channels, bits_per_sample);
    return 0;
}

int sxr_fmp4_start_impl(sxr_fmp4_t f, size_t prealloc_bytes)
{
    if (!f) return -1;
    SXR::FMP4Writer *w = (SXR::FMP4Writer *)f;
    return w->start(prealloc_bytes) ? 0 : -1;
}

int sxr_fmp4_write_sample_impl(sxr_fmp4_t f, const uint8_t *data, size_t size,
                               int64_t pts_us, int is_sync)
{
    if (!f) return -1;
    SXR::FMP4Writer *w = (SXR::FMP4Writer *)f;
    return w->writeSample(data, size, pts_us, is_sync != 0) ? 0 : -1;
}

int sxr_fmp4_close_impl(sxr_fmp4_t f)
{
    if (!f) return -1;
    SXR::FMP4Writer *w = (SXR::FMP4Writer *)f;
    bool ok = w->close();
    delete w;
    return ok ? 0 : -1;
}

} /* extern "C" */

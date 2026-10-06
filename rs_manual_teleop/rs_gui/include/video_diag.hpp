#pragma once
// Diagnóstico do pipeline de vídeo no RS. Mede, por frame, quanto tempo passa
// em cada etapa entre o h264parse e o appsink, e lê as estatísticas do
// rtpjitterbuffer. summary() devolve uma linha com as medianas desde a última
// chamada. Sem dependências de Qt, para poder ser testado à parte.
//
// Etapas (todas com o relógio do RS):
//   queue   : saída do parser   -> entrada do avdec_h264
//   decode  : entrada do avdec  -> saída do avdec
//   convert : saída do avdec    -> saída do videoconvert
//
// Os frames são associados entre probes pelo PTS do buffer, que o decoder e
// o videoconvert preservam (o x264 está em zerolatency, sem B-frames).

#include <gst/gst.h>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <map>
#include <mutex>
#include <string>
#include <vector>

class VideoDiag {
public:
    // Elementos com estes nomes têm de existir no pipeline:
    //   jb (rtpjitterbuffer), parser (h264parse), dec (avdec_h264), conv (videoconvert)
    void attach(GstElement *pipeline) {
        pipeline_ = pipeline;
        add_probe("parser", "src",  &VideoDiag::on_parser_out);
        add_probe("dec",    "sink", &VideoDiag::on_dec_in);
        add_probe("dec",    "src",  &VideoDiag::on_dec_out);
        add_probe("conv",   "src",  &VideoDiag::on_conv_out);
    }

    std::string summary() {
        std::vector<double> q, d, c;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            q.swap(q_ms_); d.swap(dec_ms_); c.swap(conv_ms_);
        }
        guint64 pushed = 0, lost = 0, late = 0, dup = 0;
        if (pipeline_) {
            if (GstElement *jb = gst_bin_get_by_name(GST_BIN(pipeline_), "jb")) {
                GstStructure *st = nullptr;
                g_object_get(jb, "stats", &st, nullptr);
                gst_object_unref(jb);
                if (st) {
                    gst_structure_get_uint64(st, "num-pushed", &pushed);
                    gst_structure_get_uint64(st, "num-lost", &lost);
                    gst_structure_get_uint64(st, "num-late", &late);
                    gst_structure_get_uint64(st, "num-duplicates", &dup);
                    gst_structure_free(st);
                }
            }
        }
        char buf[320];
        std::snprintf(buf, sizeof buf,
            "frames=%zu | queue med %.2f p95 %.2f | decode med %.2f p95 %.2f | "
            "convert med %.2f p95 %.2f ms | RTP total pushed=%llu lost=%llu late=%llu dup=%llu",
            d.size(), pct(q, .5), pct(q, .95), pct(d, .5), pct(d, .95), pct(c, .5), pct(c, .95),
            (unsigned long long)pushed, (unsigned long long)lost,
            (unsigned long long)late, (unsigned long long)dup);
        return buf;
    }

private:
    using Cb = void (VideoDiag::*)(GstClockTime pts, uint64_t now);
    struct ProbeCtx { VideoDiag *self; Cb cb; };

    GstElement *pipeline_ = nullptr;
    std::mutex mtx_;
    std::map<GstClockTime, uint64_t> t_parse_, t_dec_in_, t_dec_out_;
    std::vector<double> q_ms_, dec_ms_, conv_ms_;

    static uint64_t now_ns() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    static double pct(std::vector<double> &v, double p) {
        if (v.empty()) return 0.0;
        std::sort(v.begin(), v.end());
        return v[std::min(v.size() - 1, static_cast<size_t>(p * (v.size() - 1) + 0.5))];
    }

    void add_probe(const char *elem, const char *pad, Cb cb) {
        GstElement *e = gst_bin_get_by_name(GST_BIN(pipeline_), elem);
        if (!e) { std::fprintf(stderr, "[video_diag] elemento '%s' não encontrado\n", elem); return; }
        GstPad *p = gst_element_get_static_pad(e, pad);
        gst_pad_add_probe(p, GST_PAD_PROBE_TYPE_BUFFER, &VideoDiag::probe,
                          new ProbeCtx{this, cb},
                          [](gpointer d) { delete static_cast<ProbeCtx *>(d); });
        gst_object_unref(p);
        gst_object_unref(e);
    }

    static GstPadProbeReturn probe(GstPad *, GstPadProbeInfo *info, gpointer user_data) {
        auto *ctx = static_cast<ProbeCtx *>(user_data);
        GstBuffer *b = GST_PAD_PROBE_INFO_BUFFER(info);
        if (b && GST_BUFFER_PTS_IS_VALID(b))
            (ctx->self->*(ctx->cb))(GST_BUFFER_PTS(b), now_ns());
        return GST_PAD_PROBE_OK;
    }

    // Guarda t em m[pts], limitando o tamanho para não crescer com frames descartados.
    static void put(std::map<GstClockTime, uint64_t> &m, GstClockTime pts, uint64_t t) {
        m[pts] = t;
        while (m.size() > 32) m.erase(m.begin());
    }
    static bool take(std::map<GstClockTime, uint64_t> &m, GstClockTime pts, uint64_t &t) {
        auto it = m.find(pts);
        if (it == m.end()) return false;
        t = it->second;
        m.erase(it);
        return true;
    }

    void on_parser_out(GstClockTime pts, uint64_t now) {
        std::lock_guard<std::mutex> lk(mtx_);
        put(t_parse_, pts, now);
    }
    void on_dec_in(GstClockTime pts, uint64_t now) {
        std::lock_guard<std::mutex> lk(mtx_);
        uint64_t t;
        if (take(t_parse_, pts, t)) q_ms_.push_back((now - t) / 1e6);
        put(t_dec_in_, pts, now);
    }
    void on_dec_out(GstClockTime pts, uint64_t now) {
        std::lock_guard<std::mutex> lk(mtx_);
        uint64_t t;
        if (take(t_dec_in_, pts, t)) dec_ms_.push_back((now - t) / 1e6);
        put(t_dec_out_, pts, now);
    }
    void on_conv_out(GstClockTime pts, uint64_t now) {
        std::lock_guard<std::mutex> lk(mtx_);
        uint64_t t;
        if (take(t_dec_out_, pts, t)) conv_ms_.push_back((now - t) / 1e6);
    }
};
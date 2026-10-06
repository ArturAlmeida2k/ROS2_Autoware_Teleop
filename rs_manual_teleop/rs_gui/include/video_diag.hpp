#pragma once
// Diagnóstico do pipeline de vídeo no RS. Mede, por frame, quanto tempo passa
// em cada etapa entre o h264parse e o appsink, e lê as estatísticas do
// rtpjitterbuffer. summary() devolve uma linha com as medianas desde a última
// chamada. Sem dependências de Qt, para poder ser testado à parte.
//
// Etapas (todas com o relógio do RS):
//   queue   : saída do parser       -> entrada do avdec_h264
//   decode  : entrada do avdec      -> saída do avdec
//   convert : saída do avdec        -> saída do videoconvert
//   sink    : saída do videoconvert -> início da callback do appsink
//   total   : saída do parser       -> início da callback do appsink
//             (é o que a métrica do CSV mede como decode isolado)
//
// Os frames são associados por ordem (FIFO), não pelo PTS: cada etapa recebe
// e entrega os frames 1:1 e pela mesma ordem (x264 em zerolatency, sem
// B-frames). Na versão anterior, por PTS, a maioria dos frames não emparelhava.

#include <gst/gst.h>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <deque>
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
        {
            std::lock_guard<std::mutex> lk(mtx_);
            for (auto &f : fifo_) f.clear();
        }
        add_probe("parser", "src",  0);
        add_probe("dec",    "sink", 1);
        add_probe("dec",    "src",  2);
        add_probe("conv",   "src",  3);
    }

    // Chamar no início da callback new-sample do appsink.
    void mark_appsink() { stage(4, now_ns()); }

    std::string summary() {
        std::vector<double> v[5];
        size_t resync;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            for (int i = 0; i < 5; ++i) v[i].swap(ms_[i]);
            resync = resync_; resync_ = 0;
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
        // v[0]=queue v[1]=decode v[2]=convert v[3]=sink v[4]=total
        char buf[400];
        std::snprintf(buf, sizeof buf,
            "frames=%zu | queue %.2f | decode %.2f/%.2f | convert %.2f | sink %.2f/%.2f | "
            "TOTAL %.2f/%.2f ms (med/p95) | resync=%zu | RTP pushed=%llu lost=%llu late=%llu dup=%llu",
            v[4].size(), pct(v[0], .5), pct(v[1], .5), pct(v[1], .95), pct(v[2], .5),
            pct(v[3], .5), pct(v[3], .95), pct(v[4], .5), pct(v[4], .95), resync,
            (unsigned long long)pushed, (unsigned long long)lost,
            (unsigned long long)late, (unsigned long long)dup);
        return buf;
    }

private:
    struct ProbeCtx { VideoDiag *self; int stage; };
    struct Entry { uint64_t t0, prev; };   // t0 = saída do parser, prev = etapa anterior

    GstElement *pipeline_ = nullptr;
    std::mutex mtx_;
    std::deque<Entry> fifo_[4];   // frames à espera da etapa seguinte
    std::vector<double> ms_[5];
    size_t resync_ = 0;

    static uint64_t now_ns() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    static double pct(std::vector<double> &v, double p) {
        if (v.empty()) return 0.0;
        std::sort(v.begin(), v.end());
        return v[std::min(v.size() - 1, static_cast<size_t>(p * (v.size() - 1) + 0.5))];
    }

    void add_probe(const char *elem, const char *pad, int st) {
        GstElement *e = gst_bin_get_by_name(GST_BIN(pipeline_), elem);
        if (!e) { std::fprintf(stderr, "[video_diag] elemento '%s' não encontrado\n", elem); return; }
        GstPad *p = gst_element_get_static_pad(e, pad);
        gst_pad_add_probe(p, GST_PAD_PROBE_TYPE_BUFFER, &VideoDiag::probe,
                          new ProbeCtx{this, st},
                          [](gpointer d) { delete static_cast<ProbeCtx *>(d); });
        gst_object_unref(p);
        gst_object_unref(e);
    }

    static GstPadProbeReturn probe(GstPad *, GstPadProbeInfo *, gpointer user_data) {
        auto *ctx = static_cast<ProbeCtx *>(user_data);
        ctx->self->stage(ctx->stage, now_ns());
        return GST_PAD_PROBE_OK;
    }

    // Etapa 0 abre o frame; etapas 1..4 tiram-no da fila anterior, registam
    // o tempo desde a etapa anterior e (exceto a última) passam-no à seguinte.
    // Se uma fila crescer demais (frame descartado algures), esvazia tudo.
    void stage(int st, uint64_t now) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (st == 0) {
            fifo_[0].push_back({now, now});
        } else {
            auto &in = fifo_[st - 1];
            if (in.empty()) return;
            Entry e = in.front(); in.pop_front();
            ms_[st - 1].push_back((now - e.prev) / 1e6);
            if (st == 4) ms_[4].push_back((now - e.t0) / 1e6);
            else         fifo_[st].push_back({e.t0, now});
        }
        for (auto &f : fifo_) {
            if (f.size() > 8) {
                for (auto &g : fifo_) g.clear();
                ++resync_;
                break;
            }
        }
    }
};
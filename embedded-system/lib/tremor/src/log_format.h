// =============================================================================
//  log_format.h — formato de las líneas del log serie.
//
//  Lo usan el firmware (output_task.cpp) y el simulador de PC
//  (tools/host_sim/sim_log.cpp), así ambos escriben exactamente lo mismo y los
//  scripts de Python (tools/tremorlog.py) leen las dos cosas.
//  Cada función devuelve la longitud escrita (como snprintf), sin contar '\0'.
// =============================================================================
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "tremor_dsp.h"
#include "tremor_pipeline.h"

namespace tremor {

// S,seq,t_us,ax,ay,az,gx,gy,gz
int formatSample(char* buf, size_t size, const RawSample& s);

// W,seq_end,t_us,valid,peak_hz,tremor_dps,conc,move_dps,tilt_deg,c1,c2,c3,c4,pos_streak,episode,proc_us
int formatWindow(char* buf, size_t size, uint32_t seq_end, int64_t t_us, bool valid, const WindowMetrics& m,
                 unsigned pos_streak, bool episode, uint32_t proc_us);

// E,START|END,id,first_pos_seq,seq,t_us,mean_hz,mean_dps,max_dps,pos_windows,total_windows,reason
int formatEvent(char* buf, size_t size, const EpisodeEvent& ev, int64_t t_us, const char* reason);

// G,seq,t_us,missing
int formatGap(char* buf, size_t size, uint32_t seq, int64_t t_us, uint32_t missing);

// Cabecera: comentario de versión + una línea C,clave,valor por constante de config.h.
// Llama a emit(texto, longitud, ctx) por cada línea.
typedef void (*LineSink)(const char* line, int len, void* ctx);
void formatHeader(LineSink emit, void* ctx, const char* fft_backend);

}  // namespace tremor

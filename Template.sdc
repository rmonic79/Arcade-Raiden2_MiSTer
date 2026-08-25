derive_pll_clocks
derive_clock_uncertainty

# core specific constraints

# ============================================================
# Audio subsystem runs at ce_4m (96MHz/24 = 4MHz)
# All internal paths are CE-gated with 24 cycles between active edges.
# Multicycle = 24 for setup, 23 for hold.
# Target everything under Raiden2_audio_z80 module (jt51, T80, mixer, jt6295...).
# FIX residuo Darius: il target era *darius_audio_z80* (nome pre-fork) che NON
# matchava il modulo reale Raiden2_audio_z80 → multicycle audio MAI applicato →
# path audio valutati single-cycle → timing negativo. Trovato anche in altri core.
# ============================================================
set_multicycle_path -setup -from [get_registers {*Raiden2_audio_z80*}] -to [get_registers {*Raiden2_audio_z80*}] 24
set_multicycle_path -hold  -from [get_registers {*Raiden2_audio_z80*}] -to [get_registers {*Raiden2_audio_z80*}] 23

# ============================================================
# Raiden2 V30 CPUs (core nuovo cycle-accurate v30_core: biu+eu).
# I registri interni del V30 avanzano su CE (raiden2_ce_gen: 10 MHz = 1 ogni 8
# clk_sys), quindi ogni path reg->reg CE->CE ha ~8 clk fisici per assestarsi.
# Worst reale: v30_eu|opc -> v30_eu|psw (~18 ns), valutato single-cycle -> -6.5.
# Multicycle 2 (25 ns > 18) chiude ed e' CONSERVATIVO (2 << 8) -> NON masking.
# Target: registri DENTRO v30_core.
# ============================================================
set_multicycle_path -setup -from [get_registers {*v30_core:u_core*}] -to [get_registers {*v30_core:u_core*}] 2
set_multicycle_path -hold  -from [get_registers {*v30_core:u_core*}] -to [get_registers {*v30_core:u_core*}] 1

# ── Catture di fine-ciclo del bus V30: ad_q / bs_q / ube_n_q (v30_bus.sv:176-190)
# 292 dei ~300 path violati del design stanno QUI, e NON erano coperti: la
# maschera sopra usa il pattern *v30_core:u_core* che matcha solo i registri
# DENTRO v30_core, mentre ad_q vive in v30_bus (un livello sopra) -> valutata
# single-cycle a 10.4 ns quando la finestra vera e' 2 cicli.
# Cadenza VERA (misurata sull'RTL, non stimata):
#   - sorgenti = registri di v30_eu/v30_biu: avanzano SOLO su `ce`
#     (raiden2_ce_gen: 1 pulse ogni 6 clk @96 MHz = CPU 16 MHz);
#   - AD/BS/UBE_N sono pilotati dal core e cambiano solo DOPO un edge CE;
#   - la cattura e' gated da ce_half_d = ce+2 clk (v30_bus.sv, fix 2026-08-13).
#   => lancio al ciclo N, cattura a N+2: finestra REALE 2 cicli = 20.83 ns,
#      contro un worst data delay misurato di 11.75 ns nel fit 13. Chiude con
#      ~9 ns di margine. Multicycle 2 = la finestra vera, non una coperta:
#      il valore intermedio non viene MAI consumato (tutti i lettori di
#      ad_q/bs_q stanno dentro `if (ce)`, verificato riga per riga).
# Pattern STRETTO (solo i 3 registri di cattura), come prescrive il piano.
set_multicycle_path -setup 2 -to [get_registers {*v30_bus:u_core|ad_q* *v30_bus:u_core|bs_q* *v30_bus:u_core|ube_n_q*}]
set_multicycle_path -hold  1 -to [get_registers {*v30_bus:u_core|ad_q* *v30_bus:u_core|bs_q* *v30_bus:u_core|ube_n_q*}]

# ── addr_lat -> registri di v30_core (2026-08-16) ─────────────────────────
# STA della build 21 (output_files/critical_paths_v21.txt, 4000 path estratti):
#   3862 dei path violati hanno TUTTI la stessa forma
#     from v30_bus:u_core|addr_lat[*]  ->  to v30_core:u_core|v30_eu:u_eu|psw[*]
#   (worst -1.694 ns; il resto del design: 123 sprite renderer + 15 COP).
# Non erano coperti da NESSUNA maschera: quella di riga 26 richiede che ANCHE
# la sorgente stia dentro *v30_core:u_core*, mentre addr_lat vive in v30_bus,
# un livello sopra (stessa svista gia' corretta per ad_q/bs_q qui accanto).
# Cadenza VERA, letta sull'RTL riga per riga (NON stimata):
#   - raiden2_ce_gen.sv:40 -> ce_div = 5, cioe' ce_cnt conta 0..5: `ce` e' alto
#     1 clk ogni 6 (CPU 16 MHz da clk_sys 96 MHz, spec MAME V30 = 32/2).
#     `stall` e' hardcodato a 1'b0 (ce_gen:56) => la periodicita' non si
#     accorcia mai; `pause` puo' solo ALLUNGARE l'intervallo (piu' margine).
#   - LANCIO: addr_lat si aggiorna SOLO in `if (ce_half && t_state == ST_T1)`
#     (v30_bus.sv:283-284) e ce_half = ce ritardato 1 clk (cpu_v30_bridge:69)
#     => addr_lat cambia al ciclo ce+1.
#   - CATTURA: psw sta dentro il ramo `end else if (ce)` di v30_eu.sv:2078
#     (il commento :2080-2083 lo dichiara: "run on every enabled clock (CE),
#     NOT on CE-low fabric clocks") => psw cambia solo sui cicli ce.
#   => distanza GARANTITA fra lancio (ce+1) e cattura (ce successivo, ce+6):
#      5 cicli = 52.1 ns, contro i 10.4 ns con cui la STA li valuta oggi.
# Uso 4 e non 5: resta sotto la finestra reale, quindi e' conservativo e non
# una coperta (4 << 5). Il valore intermedio non e' mai consumato perche' i
# consumatori di psw stanno tutti sotto lo stesso `ce`.
# Elenco ESPLICITO (mai un pattern largo su tutto v30_bus: ad_q/bs_q/ube_n_q
# stanno su ce_half_d e hanno gia' la loro maschera a 2 cicli qui sopra, e
# ce_half e' un ENABLE che NON va mai mascherato).
# Tutti i registri elencati sotto sono aggiornati dentro `if (ce)` di
# v30_bus.sv:253 (t_state/lat_type/is_read_cycle/is_write_cycle) oppure dentro
# `if (ce_half && t_state == ST_T1)` di v30_bus.sv:283 (addr_lat) => cadenza
# 1 su 6, contro destinazioni (v30_core) anch'esse gated da `ce`.
# ⚠ RICALCOLATO 2026-08-18 (porting UCORE): 4 -> 3.
# Il valore 4 era derivato da `ce` PERIODICO ogni 6 clk (ce_div=5), con lancio a
# ce+1 e cattura al ce successivo = 5 cicli reali. Con il CE-STALL + catch-up a
# crediti (raiden2_ce_gen.sv) `ce` NON e' piu' periodico: dopo uno stall i cicli
# differiti rientrano a raffica con distanza minima CE_GAP_MIN = 5 clk. E con
# l'ucore ce_half = ce+2 (non ce+1), quindi l'arco ce_half -> ce successivo vale
# (CE_GAP_MIN - 2) = 3 periodi nel caso peggiore, non 5.
# Dichiarare 4 sarebbe una maschera FALSA: STA verde e silicio rotto.
# 3 e' il caso peggiore REALE; nel caso nominale (nessuno stall) ce ne sono 4.
# SOSTITUITO dal blocco V30 UCORE in fondo (2026-08-18): quella collezione
# include GIA' questi registri (addr_lat/lat_type/t_state/is_*_cycle) con il
# multicycle 4/3 derivato, e in piu' le porte M10K della ROM di microcodice
# che get_registers NON restituisce. Tenere anche questo qui creerebbe due
# vincoli diversi (3 e 4) sullo stesso arco.
# set_multicycle_path -setup 3 -from [get_registers {*v30_bus:u_core|addr_lat* *v30_bus:u_core|lat_type* *v30_bus:u_core|t_state* *v30_bus:u_core|is_read_cycle* *v30_bus:u_core|is_write_cycle*}] -to [get_registers {*v30_core:u_core*}]
# set_multicycle_path -hold  2 -from [get_registers {*v30_bus:u_core|addr_lat* *v30_bus:u_core|lat_type* *v30_bus:u_core|t_state* *v30_bus:u_core|is_read_cycle* *v30_bus:u_core|is_write_cycle*}] -to [get_registers {*v30_core:u_core*}]

# cpu_idle_r (cpu_v30_bridge.sv:94) <= ss_quiet: sorgente = stato BIU/EU
# (CE-paced, 6 clk). Consumatori: main_cpu_pause (= pause_iso & cpu_idle, con
# pause_iso statico per frame) e cpus_ss_ready (savestate, a CPU ferma).
# Nessun consumatore campiona entro il ciclo: finestra reale >= 2 cicli.
set_multicycle_path -setup 2 -to [get_registers {*cpu_v30_bridge:u_cpu|cpu_idle_r*}]
set_multicycle_path -hold  1 -to [get_registers {*cpu_v30_bridge:u_cpu|cpu_idle_r*}]

# addr_neg/ube_neg (adapter v30_bus): indirizzo catturato sul NEGEDGE gated da
# ce_half, e ce_half = ce+1 clk (cpu_v30_bridge). Finestra setup reale
# launch(ce, posedge N) -> capture(ce_half negedge, N+1.5) = 1.5 clk = 18.75 ns,
# non 0.5 (6.25 ns) come STA assume ignorando il phasing ce/ce_half. Multicycle
# -setup 2 = finestra VERA -> REALE, non masking (verificato 0 hold viol).
set_multicycle_path -setup 2 -to [get_registers {*v30_bus:u_core|addr_neg* *v30_bus:u_core|ube_neg*}]
set_multicycle_path -hold  1 -to [get_registers {*v30_bus:u_core|addr_neg* *v30_bus:u_core|ube_neg*}]

# READY path: main/sub_rq_active -> cpu_ready -> READY del core. READY e' un input
# di WAIT-STATE: se arriva in ritardo il core inserisce solo un Tw in piu' (lo
# tollera by-design, nessun errore ne' freeze). Quindi READY->core e' latency-
# tolerant -> multicycle 2 SICURO (NON una maschera dannosa: un READY tardivo non
# causa dato/istruzione sbagliata, solo un ciclo di attesa). Nessuna ce-sync RTL.
set_multicycle_path -setup 2 -from [get_registers {*rq_active*}] -to [get_registers {*v30_core:u_core*}]
set_multicycle_path -hold  1 -from [get_registers {*rq_active*}] -to [get_registers {*v30_core:u_core*}]

# Video (pre-esistente, non V30): vpos (Raiden2_video_timing) avanza a rate di RIGA
# (una volta per scanline = stabile per centinaia di clk_sys) -> vpos -> tile_col
# (Raiden2_tile_layer) e' multicycle di fatto (come i path hpos sotto). Multicycle 4
# conservativo -> REALE (vpos stabile ben oltre 4 clk).
set_multicycle_path -setup 4 -from [get_registers {*Raiden2_video_timing*vpos*}] -to [get_registers {*Raiden2_tile_layer*tile_col_pf*}]
set_multicycle_path -hold  3 -from [get_registers {*Raiden2_video_timing*vpos*}] -to [get_registers {*Raiden2_tile_layer*tile_col_pf*}]

# ============================================================
# Video timing → palette RAM address.
# hpos avanza a ce_pix (clk_sys/16), quindi resta STABILE per 16 clk_sys.
# Il path hpos → composite_pen → pal_b_addr → palette RAM portb ha 16 clk
# reali per stabilizzarsi (la sorgente hpos non cambia tra due ce_pix).
# Quartus lo valuta single-cycle (worst -1.3 ns) ma e' un multicycle di
# fatto. Multicycle 4 (conservativo, 4 << 16) chiude il timing senza
# nascondere path realmente lenti.
# ============================================================
set_multicycle_path -setup -from [get_registers {*Raiden2_video_timing*hpos*}] -to [get_registers {*raiden2_video_subbus*pal_*}] 4
set_multicycle_path -hold  -from [get_registers {*Raiden2_video_timing*hpos*}] -to [get_registers {*raiden2_video_subbus*pal_*}] 3
# stesso path verso il registro pal_b_addr (indirizzo palette lato emu) e
# ctrl_flipscreen (stabile per frame): anch'essi hpos/ce_pix-paced.
set_multicycle_path -setup -from [get_registers {*Raiden2_video_timing*hpos*}] -to [get_registers {*pal_b_addr*}] 4
set_multicycle_path -hold  -from [get_registers {*Raiden2_video_timing*hpos*}] -to [get_registers {*pal_b_addr*}] 3
# ctrl_reg[6] = flip_screen (DIP): STABILE per l'intero frame (cambia solo su
# scrittura CPU rara). Il path flip_screen → hpos_for_read (255-hpos) →
# pal_b_addr era il worst (-0.238 ns) valutato single-cycle. Multicycle 4.
set_multicycle_path -setup -from [get_registers {*Raiden2_main_top*ctrl_reg[6]*}] -to [get_registers {*pal_b_addr*}] 4
set_multicycle_path -hold  -from [get_registers {*Raiden2_main_top*ctrl_reg[6]*}] -to [get_registers {*pal_b_addr*}] 3

# ============================================================
# COP3 su ce_cop (/2, Raiden2_cop3.sv): il blocco FSM avanza SOLO sugli edge
# abilitati → ogni path verso i suoi registri ha 2 periodi REALI (3 dalle BRAM,
# che hanno uno stato di wait interposto). Multicycle VERO post-CE, non una
# maschera: stessa classe dei ×24 audio e ×2 V30 qui sopra. Coperti SOLO i
# registri del blocco gated (sort/tmp/cordic/dma/div/fsm = ~99% del TNS);
# i blocchi full-clock del COP (trig_blk/cmd_search, register file, rdata,
# latch impulsi) restano single-cycle come devono.
set_multicycle_path -setup -to [get_registers {*Raiden2_cop3:u_cop3|sort_*}] 2
set_multicycle_path -hold  -to [get_registers {*Raiden2_cop3:u_cop3|sort_*}] 1
set_multicycle_path -setup -to [get_registers {*Raiden2_cop3:u_cop3|tmp*}] 2
set_multicycle_path -hold  -to [get_registers {*Raiden2_cop3:u_cop3|tmp*}] 1
set_multicycle_path -setup -to [get_registers {*Raiden2_cop3:u_cop3|cordic_*}] 2
set_multicycle_path -hold  -to [get_registers {*Raiden2_cop3:u_cop3|cordic_*}] 1
set_multicycle_path -setup -to [get_registers {*Raiden2_cop3:u_cop3|dma_*}] 2
set_multicycle_path -hold  -to [get_registers {*Raiden2_cop3:u_cop3|dma_*}] 1
set_multicycle_path -setup -to [get_registers {*Raiden2_cop3:u_cop3|div_*}] 2
set_multicycle_path -hold  -to [get_registers {*Raiden2_cop3:u_cop3|div_*}] 1
# dde5_* (radar): stessi registri del blocco FSM ce_cop-gated, ma il pattern
# div_* NON li matcha (nome dde5_div) -> restavano single-cycle, 36 path
# violati BRAM->COP nel fit. Stessa classe, stessa cadenza: 2 periodi reali.
set_multicycle_path -setup -to [get_registers {*Raiden2_cop3:u_cop3|dde5_*}] 2
set_multicycle_path -hold  -to [get_registers {*Raiden2_cop3:u_cop3|dde5_*}] 1
set_multicycle_path -setup -to [get_registers {*Raiden2_cop3:u_cop3|fsm*}] 2
set_multicycle_path -hold  -to [get_registers {*Raiden2_cop3:u_cop3|fsm*}] 1

# ===========================================================================
# PORTATO da Arcade-Raiden_MiSTer_private/Template.sdc il 2026-08-18 insieme
# all'UCORE. Adattato alle NOSTRE fasi: ce 1 ogni 6 clk (ce_div=5 @96 MHz,
# non /8 come Raiden 1), ce_half = ce+2, CE_GAP_MIN = 5 anche in catch-up.
# Il contratto C-a/C-b/C-c dell'ucore e' quindi soddisfatto: ce a 0,
# ce_half a +2 (1 clk vuoto in mezzo), ce successivo a >= +5.
# SENZA questo blocco i registri interni di v30u_eu restano coperti solo dal
# multicycle 2 ereditato dal core FSM: MISURATO worst -24.560 ns su 4000
# path, TUTTI dentro v30u_eu (build 25).
# ===========================================================================
# ============================================================================
# V30 UCORE — multicycle CE-paced (da nec_test/hdl/nec_test.sdc, derivazione
# completa nel file upstream; contratto portabile C-a/C-b/C-c: ce e ce_half
# mai nello stesso clk, assert >=2 clk di distanza, >=1 ce_half tra due ce.
# Il nostro train (raiden2_ce_gen /8, ce_half=ce+1clk) lo soddisfa.
#   ce -> ce           : setup 4 / hold 3
#   ce -> ce_half      : setup 2 / hold 1   (t1_half2 = UNICO flop negedge)
#   ce_half -> ce      : setup 3 / hold 2
# I path di confine (bridge/ssbus dentro, uscite core verso fabric) restano
# single-cycle: i loro launch reg non sono CE-gated. NON estendere a coperta.
# ============================================================================
# ─────────────────────────────────────────────────────────────────────────
# CONTRATTO ce/ce_half (nec_test, dichiarato "operating envelope" 2026-08-13).
# Clausole verificate dal gate upstream hdl/tb/ce_contract_check.sv:
#   C-a: ce e ce_half MAI sullo stesso fabric clock
#   C-b: MAI su clock adiacenti (almeno 1 clock vuoto in mezzo)
#   C-c: MAI due ce senza un ce_half in mezzo
# NOSTRE FASI (cpu_v30_bridge + raiden2_ce_gen):
#   ce a 0, ce_half a +2, ce successivo a >= +5 (CE_GAP_MIN, anche in catch-up)
#   -> C-a ok, C-b ok (1 clock vuoto tra ce e ce_half, 2 tra ce_half e ce),
#      C-c ok. Prima ce_half era a +1: violava C-b E rendeva FALSO il
#      multicycle 2 qui sotto (1 solo periodo reale) -> STA verde +1.18 con il
#      silicio che violava di -11.3 ns = bug punteggio.
# ─────────────────────────────────────────────────────────────────────────
# La collezione include ANCHE le porte interne della ROM di microcodice.
# ucdecode/ucrom sono letture combinatorie in RTL; Quartus le mappa in M10K
# (OPERATION_MODE=ROM) e vi ASSORBE il registro di indirizzo a monte, che e' un
# registro dell'EU -> gated da CE come tutto il core, quindi la multicycle e'
# onesta anche li'. Ma get_registers NON restituisce le porte delle memorie:
# serve get_keepers. Senza, quei path restano single-cycle -> setup -5.886 ns
# (build 2026-08-16 00:25, TUTTI i peggiori dentro ucdecode del sub).
# Upstream non lo vede perche' chiude a 50 MHz: a 20 ns quel path passa lo
# stesso, a 12.5 ns no.
# get_keepers e NON get_registers su TUTTI i termini: add_to_collection rifiuta
# di mescolare tipi ("requires type ( reg ), but found type kpr") e i nodi che
# ci servono -- le porte interne dell'M10K della ROM di microcodice -- sono
# keeper, non register. Tipo uniforme = collezione valida.
# Il pattern half usa t1_half2* per prendere anche il t1_half2~DUPLICATE che il
# fitter crea sul sub: upstream lo tiene stretto per non ri-basare le proprie
# misure, ma se il duplicato finisce fra i vincolati si prende una multicycle
# su un arco cross-phase da 1.0 periodo = PASS falso.
set v30u_regs [add_to_collection \
                   [get_keepers -nowarn {*|v30u_eu:*|*}] \
                   [get_keepers -nowarn {*|v30u_biu:*|*}]]
# Registri dell'ADATTATORE che avanzano sullo STESSO ce del core (tutti dentro
# `if (ce)` in v30_bus.sv, verificato riga per riga). Il lancio e' CE-gated e la
# cattura pure: la finestra reale e' 8 clk, dichiararne 4 e' conservativo --
# stessa identica derivazione gia' applicata ai registri interni del core.
# Elencati UNO PER UNO: non e' una coperta su *v30_bus*, che prenderebbe anche
# i registri free-running (rdata_q, bs_q, ube_n_q) dove sarebbe una bugia.
set v30bus_ce [get_keepers -nowarn {*|v30_bus:*|addr_lat[*] *|v30_bus:*|be_lat[*]                                     *|v30_bus:*|dout_lat[*] *|v30_bus:*|t_state[*]                                     *|v30_bus:*|lat_type[*] *|v30_bus:*|is_read_cycle                                     *|v30_bus:*|is_write_cycle *|v30_bus:*|addr_valid *|v30_bus:*|a0_lat                                     *|v30_bus:*|inta_prev *|v30_bus:*|inta_second}]
if {[get_collection_size $v30bus_ce] > 0} {
    set v30u_regs [add_to_collection $v30u_regs $v30bus_ce]
}
set v30u_half [get_keepers -nowarn {*|v30u_biu:*|t1_half2*}]
# Nel bus de-muxato t1_half2 NON ESISTE: la collezione half e' vuota e
# remove_from_collection con una collezione vuota da errore. Guardia esplicita.
if {[get_collection_size $v30u_half] > 0} {
    set v30u_ce [remove_from_collection $v30u_regs $v30u_half]
} else {
    set v30u_ce $v30u_regs
}
if {[get_collection_size $v30u_ce] > 0} {
    # 4/3 e non il 2/1 di upstream: loro derivano 2 dal treno MINIMO legale
    # (div 2). Il NOSTRO treno e' 1 ce ogni 6 clk e non scende sotto
    # CE_GAP_MIN=5 nemmeno in catch-up -> 4 periodi (50 ns) restano dentro la
    # finestra fisica reale (>=52 ns: CE_GAP_MIN=5 @96 MHz). Vero, e piu' rilassato per l'Fmax.
    # 5/4 e non 4/3 (2026-08-18): 4 era la scelta di Raiden 1, il cui treno e'
    # /8 (8 clk fra due ce, quindi 4 periodi = meta' della finestra = molto
    # margine). Il NOSTRO treno e' /6 con CE_GAP_MIN=5, e quel 5 e' un
    # INVARIANTE del generatore, non una media: raiden2_ce_gen.sv emette `ce`
    # solo con `ce_gap >= CE_GAP_MIN`, quindi due ce non distano MAI meno di 5
    # clk, nemmeno durante il catch-up dei crediti. La finestra fisica garantita
    # e' percio' 5 periodi = 52.1 ns.
    # Misura build 26 con 4 dichiarati: worst -3.425, cioe' il cono peggiore
    # chiede 41.6 + 3.425 = ~45 ns. Contro i 52 ns reali resta ~7 ns di margine.
    # Dichiarare 5 NON e' una maschera: e' la finestra vera, derivata dal codice.
    set_multicycle_path -setup 5 -from $v30u_ce -to $v30u_ce
    set_multicycle_path -hold  4 -from $v30u_ce -to $v30u_ce
    post_message -type info \
        "Template.sdc: CE multicycle 4/3 su [get_collection_size $v30u_ce] registri v30u ce-gated"
}
# ⚠ LE DUE ECCEZIONI CROSS-PHASE SONO CANCELLATE, NON MANCANTI.
# Erano `ce -> ce_half` e `ce_half -> ce` a 2/1. Upstream le ha cancellate con
# la correzione del contratto (nec_test 5f63289d, user ruling 2026-08-13) e lo
# scrive esplicitamente: "DO NOT RESTORE THEM". Motivo: sul contratto corretto
# gli enable ADIACENTI sono legali, quindi quegli archi valgono 1.0 periodo --
# che E' il controllo di default -- e un -setup 2 li' e' un PASS falso di un
# fattore due, nella direzione ottimista, proprio sull'arco che lo split della
# collezione esiste per proteggere.
# NON riderivarle "perche' il nostro treno e' piu' largo": si segue upstream.
if {[get_collection_size $v30u_half] > 0} {
    post_message -type info \
        "Template.sdc: [get_collection_size $v30u_half] flop ce_half TENUTI FUORI dalla\
         multicycle CE; archi cross-phase single-cycle, come upstream"
}

# status[] HPS -> Raiden2_audio_z80: SOLO selettori volume OSD (quasi-statici:
# cambiano su azione utente nel menu; il pause arriva da paused_safe, registro
# derivato, NON da status raw -> questi archi non portano controllo critico).
# La catena gain_resolve*mix*softclip (3 moltiplicatori) e' ~21.7ns: single-cycle
# non chiude ne' deve — al cambio slider al peggio 1 sample transitorio.
set_false_path -from [get_registers {*|hps_io:*|status[*]}] -to [get_registers {*|Raiden2_audio_z80:*|*}]

# v30u (CE-gated) -> capture free-running del v30_bus (ad_q/bs_q/ube_n_q):
# ricatturano OGNI clk ma il valore e' consumato solo a istanti ce-paced
# (dout_lat in T2/T3, commit su transizioni T-state = >=8 clk dal lancio CE).
# La cattura a L+1 non e' mai letta -> setup 2 / hold 1 derivato dal NOSTRO
# train fisso /8 (non dal contratto di portabilita' upstream, che qui non serve).
# rdata_q AGGIUNTO 2026-08-18: stessa identica classe di ad_q/bs_q/ube_n_q.
# v30_bus_demux.sv:385-386 `rdata_q <= ... cpu_din` cattura a OGNI clock, ma
# il core legge DATA_I solo a T2 (v30u_biu F57 'cur_data = ad_i at T2'), cioe'
# almeno un ce DOPO il T1 in cui addr_lat e' cambiato => >= CE_GAP_MIN = 5 clk.
# Le catture intermedie non vengono MAI lette. Senza questo, i 156 path
# addr_lat -> rdata_q restano single-cycle su un cono che attraversa il mux
# di cpu_din (tutte le regioni).
set v30bus_cap [get_registers -nowarn {*|v30_bus:*|ad_q[*] *|v30_bus:*|bs_q[*] *|v30_bus:*|ube_n_q *|v30_bus:*|rdata_q[*]}]
if {[get_collection_size $v30u_ce] > 0 && [get_collection_size $v30bus_cap] > 0} {
    # 2026-08-17: la cattura ora avviene a +4 clk dal CE (v30_bus, ce_pipe[3]),
    # quindi l'arco ha 4 periodi REALI e il vincolo li dichiara. Prima erano 2
    # dichiarati su 1 disponibile: MISURATO -8.993 ns una volta rimossa la
    # copertura (ucrom -> bs_q). Ora non maschera niente.
    set_multicycle_path -setup 4 -from $v30u_ce -to $v30bus_cap
    set_multicycle_path -hold  3 -from $v30u_ce -to $v30bus_cap
    post_message -type info \
        "Template.sdc: boundary 2/1 su [get_collection_size $v30bus_cap] capture reg v30_bus"
}

# RIMOSSO 2026-08-16: qui c'era un set_false_path EU -> addr_neg/ube_neg.
# Quei registri NON ESISTONO PIU' (cattura negedge eliminata col bus
# de-muxato): l'eccezione non agganciava piu' nulla e restava a dire il falso
# a chi legge. Un false_path verso registri inesistenti e' rumore, e il rumore
# nell'SDC e' esattamente come nascono le bugie che costano mesi.

# ssbus -> registri core (write restore): v30_bus ora tiene addr/dato stabili
# 2 clk PRIMA di SS_WE (ss_wr_delay) -> il cono di decode (~30 ns) ha 3 periodi
# VERI. La destinazione cattura solo al clk di SS_WE (3o) -> setup 3 / hold 2
# derivati, non maschera.
set ss_addr_regs [get_registers -nowarn {*|v30_core:*|ss_addr_q[*]}]
if {[get_collection_size $ss_addr_regs] > 0 && [get_collection_size $v30u_regs] > 0} {
    # 4/3 (era 3/2) dopo l'allungamento della staging in v30_bus.sv: addr/dato
    # restano stabili 3 periodi PRIMA del capture buono, che ora e' il 4o fronte
    # di SS_WE. Derivato dalla staging, non assunto.
    set_multicycle_path -setup 4 -from $ss_addr_regs -to $v30u_regs
    set_multicycle_path -hold  3 -from $ss_addr_regs -to $v30u_regs
}

# ss_we_q (interno v30_core) -> regfile v30u: fanout WE ~18ns, single-cycle non
# chiude a 80MHz (upstream 45MHz non lo vede). SS_WE ora largo 2 clk in v30_bus
# (write tutte idempotenti, verificato) -> il capture buono e' il 2o, con WE e
# decode stabili da 1 clk -> setup 2 / hold 1 DERIVATO.
set ss_we_regs [get_registers -nowarn {*|v30_core:*|ss_we_q}]
if {[get_collection_size $ss_we_regs] > 0 && [get_collection_size $v30u_regs] > 0} {
    set_multicycle_path -setup 2 -from $ss_we_regs -to $v30u_regs
    set_multicycle_path -hold  1 -from $ss_we_regs -to $v30u_regs
}

# ss_we_q e ss_wdata_q -> regfile v30u: cono ~39 ns (WE verso regfile + blocchi
# M10K della ROM di microcodice). SS_WE ora largo 4 clk (v30_bus, write tutte
# idempotenti): il capture valido e' il 4o fronte, con WE, decode e dato stabili
# da 3 periodi -> setup 4 / hold 3 DERIVATI dalla staging in v30_bus.sv, non
# assunti. Con 3 periodi (37.5 ns) mancavano 1.737 ns: misurato, non stimato.
set ss_wd_regs [get_registers -nowarn {*|v30_core:*|ss_wdata_q[*]}]
if {[get_collection_size $ss_we_regs] > 0 && [get_collection_size $v30u_regs] > 0} {
    set_multicycle_path -setup 4 -from $ss_we_regs -to $v30u_regs
    set_multicycle_path -hold  3 -from $ss_we_regs -to $v30u_regs
}
if {[get_collection_size $ss_wd_regs] > 0 && [get_collection_size $v30u_regs] > 0} {
    set_multicycle_path -setup 3 -from $ss_wd_regs -to $v30u_regs
    set_multicycle_path -hold  2 -from $ss_wd_regs -to $v30u_regs
}
# (fine del blocco portato: il resto del file di Raiden 1 riguarda irq_pending
# e path specifici di quel core, non trasferiti qui senza verifica.)

# ── Latch degli INGRESSI (quasi-statici) -> registri del core ───────────────
# STA build 25: ~2000 dei path negativi partono da p1p2_data_lat e arrivano a
# v30u_biu|r_cur_data / r_cur_pn / r_cmt_data / r_ts e v30u_eu|row_posted.
# Il cono e' lungo (~49 ns) perche' quel dato attraversa il MUX di cpu_din, che
# seleziona fra TUTTE le regioni (RAM, tilemap, palette, CRTC, COP, IO...).
# Perche' il multicycle e' ONESTO qui:
#   - Raiden2_main_top.sv:889 `p1p2_data_lat <= {p2_input, p1_input}` ricopia gli
#     ingressi a ogni clock, ma il VALORE cambia a ritmo UMANO (millisecondi):
#     il registro commuta di continuo con lo stesso contenuto.
#   - la cattura nel BIU e' CE-paced (dato campionato a T2 del ciclo di bus).
#   - conseguenza di un ritardo: la CPU legge lo stato del pad un frame dopo.
#     Indistinguibile e innocuo; NON puo' produrre un dato o un'istruzione
#     sbagliata, che e' il criterio per distinguere un vincolo vero da una bugia.
# Stessa classe gia' trattata cosi' in questo file (vpos -> tile_col, status OSD).
# Elenco ESPLICITO dei soli latch di INPUT: NON si estende ai latch di dato vero
# (ram/text/bg/fg/mg/pal/crtc/cop), dove un ritardo darebbe una lettura sbagliata.
set in_lat [get_registers -nowarn {*|Raiden2_main_top:*|p1p2_data_lat[*]                                   *|Raiden2_main_top:*|p3p4_data_lat[*]                                   *|Raiden2_main_top:*|sys_data_lat[*]                                   *|Raiden2_main_top:*|dsw_data_lat[*]                                   *|Raiden2_main_top:*|dsw2_data_lat[*]}]
if {[get_collection_size $in_lat] > 0 && [get_collection_size $v30u_regs] > 0} {
    set_multicycle_path -setup 4 -from $in_lat -to $v30u_regs
    set_multicycle_path -hold  3 -from $in_lat -to $v30u_regs
    post_message -type info         "Template.sdc: input-latch multicycle 4/3 su [get_collection_size $in_lat] registri"
}

# ── Bus CE-gated -> registri del COP (scritture della CPU) ─────────────────
# STA build 25: ~2300 path da v30_bus|addr_lat verso Raiden2_cop3 (bcd_val,
# cop_dma_src/dst/size...), worst -5.871.
# CORREZIONE di una mia classificazione sbagliata del 2026-08-16, quando li
# avevo dati per "reali a UN ciclo, non mascherabili". Il conteggio vero:
#   - addr_lat si aggiorna all'INGRESSO IN T1 (v30_bus_demux.sv, `if (ce &&
#     next_t == ST_T1)`);
#   - i registri del COP catturano su cpu_wr_pulse (Raiden2_cop3.sv:259
#     `cs_wr_now & ~cs_wr_prev`), e cs_wr_now richiede `wr`, cioe' mem_wr, che
#     e' alto SOLO in T3 (v30_bus_demux.sv:403).
#   => fra lancio (T1) e cattura (T3) ci sono DUE T-state, cioe' 2 impulsi ce,
#      cioe' >= 10 clk (CE_GAP_MIN=5) = >= 104 ns. Dichiararne 4 (41.6 ns) e'
#      largamente conservativo, non una coperta.
# Sorgente = la stessa collezione $v30bus_ce gia' costruita sopra (elenco
# esplicito dei registri del bus che avanzano su ce), NON un pattern largo.
if {[get_collection_size $v30bus_ce] > 0} {
    set cop_regs [get_registers -nowarn {*|Raiden2_cop3:*|*}]
    if {[get_collection_size $cop_regs] > 0} {
        set_multicycle_path -setup 4 -from $v30bus_ce -to $cop_regs
        set_multicycle_path -hold  3 -from $v30bus_ce -to $cop_regs
        post_message -type info             "Template.sdc: bus->COP multicycle 4/3 su [get_collection_size $cop_regs] registri cop3"
    }
}

# ── irq_pending -> core: INT e' ASINCRONO by-design ────────────────────────
# Il V30 reale campiona INT per-istruzione: arrivare un CE piu' tardi significa
# un interrupt esterno leggermente posticipato, indistinguibile e corretto.
# Non puo' produrre un dato sbagliato. Stessa derivazione di Raiden 1.
set irq_regs [get_registers -nowarn {*|Raiden2_main_top:*|irq_pending*}]
if {[get_collection_size $irq_regs] > 0 && [get_collection_size $v30u_regs] > 0} {
    set_multicycle_path -setup 4 -from $irq_regs -to $v30u_regs
    set_multicycle_path -hold  3 -from $irq_regs -to $v30u_regs
}

# ── COP interno: FSM pacata da ce_cop (/2) ─────────────────────────────────
# Raiden2_cop3.sv:318-321 `ce_cop <= ~ce_cop`: la FSM del COP avanza 1 clk su 2,
# quindi ogni arco reg->reg INTERNO al COP ha 2 periodi fisici (20.8 ns) e non 1.
# Multicycle 2 = la finestra VERA, non una copertura.
# ⚠ SOLO cop3 -> cop3: gli archi che ESCONO dal COP verso le BRAM video o la
# main RAM restano single-cycle (li' il consumatore non e' ce_cop-paced).
set cop_all [get_registers -nowarn {*|Raiden2_cop3:*|*}]
if {[get_collection_size $cop_all] > 0} {
    set_multicycle_path -setup 2 -from $cop_all -to $cop_all
    set_multicycle_path -hold  1 -from $cop_all -to $cop_all
    post_message -type info         "Template.sdc: COP interno multicycle 2/1 (ce_cop /2)"
}

# addr_lat & C. (CE-gated, collezione $v30bus_ce) -> capture free-running del bus
# (rdata_q/bs_q/ube_n_q): stessa derivazione del blocco qui sopra, ma con la
# sorgente giusta. Il lancio e' in T1 su ce, la lettura e' a T2 = 1 ce dopo.
if {[get_collection_size $v30bus_ce] > 0 && [get_collection_size $v30bus_cap] > 0} {
    set_multicycle_path -setup 4 -from $v30bus_ce -to $v30bus_cap
    set_multicycle_path -hold  3 -from $v30bus_ce -to $v30bus_cap
}

# ── addr_lat -> memorie/periferiche del top (2026-08-20) ───────────────────
# Ultimi 33 path negativi che toccano la CPU (build 29): la sorgente e'
# `addr_lat`, la destinazione sono le BRAM (altsyncram) e il CRTC del top.
# Perche' il multicycle e' ONESTO:
#   - LANCIO: addr_lat si aggiorna solo su `ce_half && t_state == ST_T1`,
#     cioe' 1 volta ogni >= CE_GAP_MIN = 5 clk;
#   - CATTURA: le BRAM registrano l'indirizzo a ogni clock, ma il dato che ne
#     esce viene letto dalla CPU solo a T2 del ciclo di bus, cioe' >= 1 `ce`
#     dopo il T1 in cui l'indirizzo e' cambiato;
#   - le letture intermedie (indirizzo non ancora assestato) NON vengono MAI
#     consumate: quando la CPU campiona, l'indirizzo e' fermo da >= 5 clk e la
#     memoria ha gia' riletto.
# Stessa identica derivazione gia' applicata a ad_q/bs_q/ube_n_q/rdata_q.
# Dichiaro 4 (41.6 ns) contro una finestra reale di 5 (52.1 ns): conservativo.
set addr_dst [get_keepers -nowarn {*|altsyncram:*|* *|Raiden2_seibu_crtc:*|*}]
set addr_src [get_registers -nowarn {*|v30_bus:*|addr_lat[*]}]
if {[get_collection_size $addr_src] > 0 && [get_collection_size $addr_dst] > 0} {
    set_multicycle_path -setup 4 -from $addr_src -to $addr_dst
    set_multicycle_path -hold  3 -from $addr_src -to $addr_dst
    post_message -type info         "Template.sdc: addr_lat -> memorie/CRTC multicycle 4/3"
}

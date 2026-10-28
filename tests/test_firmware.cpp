#include <cassert>
#include <iostream>
#include <vector>
#include <fstream>
#include "accelerometer.h"
#include "sd_card.h"
#include "diagnostics.h"
#include "acquisition_scheduler.h"

uint32_t fake_mask = 0, fake_time = 0;
volatile uint32_t SCB_AIRCR = 0;
FakeSerial Serial;
SPIClass SPI, SPI1;
extern FsFile file;
extern uint64_t total_bytes_written;
extern LSM6DSO32Sensor lsm_main, lsm_sat;
extern ADXL371class adxl_main;
extern bool has_adxl_main, has_adxl_sat, has_lsm_main, has_lsm_sat;

void reset_ring() {
    rb_head = rb_tail = 0;
    acquisition_diag = AcquisitionDiagnostics{};
    bench_diag = BenchDiagnostics{};
    file.output.clear(); file.max_write = SIZE_MAX; file.busy = false;
    total_bytes_written = 0;
}

void push_axis(SPIClass &spi, int16_t v, bool marker) {
    uint16_t word = ((uint16_t)v & 0xfff) << 4;
    word |= marker;
    spi.fifo.push_back(word >> 8); spi.fifo.push_back(word & 255);
}

int main(int argc, char **argv) {
    std::ofstream fixture;
    if (argc > 1) fixture.open(argv[1], std::ios::binary);
    auto export_packet = [&]() {
        if (fixture.is_open()) fixture.write((const char*)ring_buffer, rb_head);
    };
    static_assert(sizeof(CHUNK_HEADER) == 10, "Header ABI changed");
    static_assert(sizeof(AdxlAcquisitionPacket) == 1058, "ADXL packet layout changed");
    static_assert(sizeof(LsmAcquisitionPacket) == 2138, "LSM packet layout changed");
    static_assert(sizeof(Mic_PACKET) == 522, "Audio ABI changed");
    static_assert(sizeof(BME280_PACKET) == 22, "BME ABI changed");
    SPIClass bus; ADXL371class a(10,bus); a.begin(); a.setFifoSamples(300);
    assert(bus.regs[FIFO_SAMPLES] == 0x2c && (bus.regs[FIFO_CTL] & 1) == 1);
    uint8_t raw[600]; uint16_t entries;
    for (int i=0;i<299;++i) push_axis(bus,0,i%3==0);
    assert(a.readFifoRawData(raw,sizeof(raw),entries) == AdxlFifoResult::TooFewEntries);
    assert(entries == 299 && bus.fifo.size() == 598);
    uint32_t discarded = 0; assert(a.drainFifo(&discarded)); assert(discarded == 299);
    for (int i=0;i<100;++i) { push_axis(bus,-2048,true); push_axis(bus,2047,false); push_axis(bus,-1,false); }
    assert(a.readFifoRawData(raw,599,entries) == AdxlFifoResult::InvalidRequest);
    assert(bus.fifo.size()==600);
    assert(a.readFifoRawData(raw,sizeof(raw),entries)==AdxlFifoResult::Ok && entries==300);
    TRIPLET decoded[100];
    assert(a.decodeFifoData(raw,decoded,99)==AdxlFifoResult::InvalidRequest);
    assert(a.decodeFifoData(raw,decoded,100)==AdxlFifoResult::Ok);
    for (const auto &v: decoded) assert(v.x==-2048 && v.y==2047 && v.z==-1);
    raw[3] |= 1;
    assert(a.decodeFifoData(raw,decoded,100)==AdxlFifoResult::MarkerMismatch);

    reset_ring();
    std::vector<uint8_t> bytes(610); bytes[0]=0xaa; bytes[1]=PACKET_ADXL_MAIN;
    for (size_t i=2;i<bytes.size();++i) bytes[i]=i%251;
    rb_head=rb_tail=RING_BUFFER_SIZE-15;
    fake_mask=1;
    assert(ring_buffer_push(bytes.data(),bytes.size())); assert(fake_mask==1);
    fake_mask=0;
    file.max_write=7;
    write_buffer_to_sd(true);
    assert(file.output.size()==7 && total_bytes_written==7 && rb_tail==RING_BUFFER_SIZE-8);
    assert(acquisition_diag.sd_short_writes==1);
    file.max_write=SIZE_MAX; while (rb_tail != rb_head) write_buffer_to_sd(true);
    assert(file.output==bytes && rb_tail==rb_head && total_bytes_written==610);
    assert(acquisition_diag.sensor[1].queued_packets==1);
    rb_tail=0; rb_head=RING_BUFFER_SIZE-1;
    const uint32_t old=rb_head;
    assert(!ring_buffer_push(bytes.data(),bytes.size()) && rb_head==old);
    assert(acquisition_diag.sensor[1].ring_drops==1);

    reset_ring(); ring_buffer_push(bytes.data(),bytes.size()); file.max_write=0;
    write_buffer_to_sd(true); assert(rb_tail==0 && total_bytes_written==0);
    assert(!close_sd(JUMPER_END)); assert(file.output.empty());
    assert(acquisition_diag.sd_close_failures==1);

    // Coalesce across ring wrap, retain short-write suffix, and align the next write.
    reset_ring();
    std::vector<uint8_t> blocks(2*SD_BLOCK_BYTES+123);
    for (size_t i=0;i<blocks.size();++i) blocks[i]=i%251;
    rb_head=rb_tail=RING_BUFFER_SIZE-13;
    ring_buffer_push(blocks.data(),100);
    assert(!sd_write_due()); write_buffer_to_sd(); assert(file.output.empty());
    ring_buffer_push(blocks.data()+100,blocks.size()-100);
    assert(sd_write_due()); file.max_write=7; write_buffer_to_sd();
    assert(file.output.size()==7 && acquisition_diag.sd_requested_bytes==SD_BLOCK_BYTES);
    file.max_write=SIZE_MAX; write_buffer_to_sd();
    assert(file.output.size()==SD_BLOCK_BYTES && acquisition_diag.sd_requested_bytes==2*SD_BLOCK_BYTES-7);
    assert(std::equal(file.output.begin(),file.output.end(),blocks.begin()));
    assert(close_sd(JUMPER_END));
    assert(file.output.size()==blocks.size()+sizeof(CHUNK_HEADER));
    assert(std::equal(blocks.begin(),blocks.end(),file.output.begin()));
    assert(file.output[blocks.size()]==0xaa && file.output[blocks.size()+1]==EOF_);
    assert(rb_head==rb_tail && total_bytes_written==blocks.size());

    // Pending sensors never lose priority at timeout or ring pressure.
    reset_ring(); ring_buffer_push(blocks.data(),blocks.size());
    fake_time=0xfffffff0U;
    write_buffer_to_sd(false,8); assert(file.output.empty());
    fake_time+=SD_MAX_DEFER_US+1;
    write_buffer_to_sd(false,8); assert(file.output.empty());
    assert(acquisition_diag.sd_max_defer_us>=SD_MAX_DEFER_US);
    write_buffer_to_sd(false,0); assert(file.output.size()==SD_BLOCK_BYTES);
    reset_ring(); rb_head=SD_PRESSURE_BYTES;
    write_buffer_to_sd(false,4); assert(file.output.empty());
    assert(acquisition_diag.sd_forced_pending_writes==0);
    // Card busy is never overridden by ring pressure, but shutdown may block.
    file.busy=true; write_buffer_to_sd(false,0); assert(file.output.empty());
    write_buffer_to_sd(true); assert(file.output.size()==SD_BLOCK_BYTES);
    reset_ring(); rb_head=RING_BUFFER_SIZE-1;
    assert(close_sd(JUMPER_END));
    assert(total_bytes_written==RING_BUFFER_SIZE-1 && rb_head==rb_tail);

    volatile bool pending=true; volatile uint32_t timestamp=123; uint32_t out=0;
    fake_mask=1;
    assert(take_sensor_interrupt(pending,timestamp,out) && out==123 && !pending && fake_mask==1);
    assert(!take_sensor_interrupt(pending,timestamp,out)); fake_mask=0;

    auto metadata = []() {
        AcquisitionMetadata m;
        memcpy(&m,ring_buffer+sizeof(CHUNK_HEADER),sizeof(m)); return m;
    };
    reset_ring();
    for(int i=0;i<110;++i) { push_axis(bus,i,true); push_axis(bus,-i,false); push_axis(bus,10,false); }
    log_adxl371_fifo(&a,0,ID_ADXL371_MAIN);
    assert(ring_buffer[1]==PACKET_ADXL_MAIN);
    assert(metadata().record_count==100 && rb_head==10+28+100*6);
    assert(acquisition_diag.sensor[1].records_queued==100 && acquisition_diag.sensor[1].discarded_words==30);
    export_packet();
    reset_ring();
    for(int i=0;i<100;++i) { push_axis(bus,1,true); push_axis(bus,2,false); push_axis(bus,3,false); }
    push_axis(bus,1,true);
    log_adxl371_fifo(&a,0,ID_ADXL371_MAIN);
    assert(metadata().record_count==100 && !(metadata().flags&ACQ_PARTIAL_TAIL));
    assert(acquisition_diag.sensor[1].discarded_words==1);
    // A malformed surplus tail is drained, never appended to the valid payload.
    reset_ring();
    for(int i=0;i<100;++i) { push_axis(bus,1,true); push_axis(bus,2,false); push_axis(bus,3,false); }
    for(int i=0;i<9;++i) push_axis(bus,7,false);
    log_adxl371_fifo(&a,0,ID_ADXL371_MAIN);
    assert(metadata().record_count==100 && acquisition_diag.sensor[1].marker_failures==0);
    assert(acquisition_diag.sensor[1].discarded_words==9 && bus.fifo.empty());

    // A malformed first block is rejected and its exact first bad word retained.
    reset_ring();
    for(int i=0;i<100;++i) { push_axis(bus,1,true); push_axis(bus,2,i==4); push_axis(bus,3,false); }
    log_adxl371_fifo(&a,0,ID_ADXL371_MAIN);
    assert(metadata().record_count==0 && (metadata().flags&ACQ_BAD_MARKER));
    assert(bench_diag.adxl_count[0]==1 && bench_diag.adxl[0][0].first_bad_word==13);
    assert(bench_diag.adxl[0][0].bad_word==0x21);
    assert(acquisition_diag.sensor[1].discarded_words==300);

    // Empty and below-watermark IRQs cannot write ADC/FIFO control registers.
    reset_ring();
    const uint32_t control_writes = bus.write_count;
    log_adxl371_fifo(&a,0,ID_ADXL371_MAIN);
    for(int i=0;i<299;++i) push_axis(bus,0,i%3==0);
    log_adxl371_fifo(&a,0,ID_ADXL371_MAIN);
    assert(bus.write_count==control_writes && rb_head==0 && bus.fifo.size()==598);
    assert(acquisition_diag.sensor[1].skipped_services==2 && acquisition_diag.sensor[1].empty_services==1);
    assert(acquisition_diag.sensor[1].pause_calls==0);
    bus.fifo.clear();
    reset_ring(); bus.regs[STATUS]=FIFO_OVR;
    for(int i=0;i<100;++i) { push_axis(bus,1,true); push_axis(bus,2,false); push_axis(bus,3,false); }
    log_adxl371_fifo(&a,0,ID_ADXL371_MAIN);
    assert(metadata().record_count==0 && (metadata().flags&ACQ_OVERRUN));
    bus.regs[STATUS]=0;

    // Physical FIFO headroom budget: low occupancy passes, higher occupancy
    // requires proactive drain before starting a card operation.
    LSM6DSO32Sensor admission_lsm(&SPI,10,10000000);
    assert(lsm_safe_for_sd(&admission_lsm, ID_LSM_MAIN));
    for (int i=0;i<360;++i)
        admission_lsm.fifo.push_back({{8,1,0,2,0,3,0}});
    assert(!lsm_safe_for_sd(&admission_lsm, ID_LSM_MAIN));
    while (!admission_lsm.fifo.empty())
        log_lsm_fifo(&admission_lsm, 0, ID_LSM_MAIN, false);
    assert(admission_lsm.fifo.empty());
    assert(lsm_safe_for_sd(&admission_lsm, ID_LSM_MAIN));
    // An expired prediction must be re-read before draining. The simulated
    // FIFO remains empty while host time advances; a redundant drain is wrong.
    fake_time += 40000;
    assert(!lsm_cached_headroom_valid(ID_LSM_MAIN));
    const auto before_refresh = admission_lsm.status_reads;
    assert(lsm_cached_safe_for_sd(&admission_lsm, ID_LSM_MAIN));
    assert(admission_lsm.status_reads > before_refresh);
    // Never lose a latched overrun flag through the admission-status read.
    admission_lsm.fifo.push_back({{8,1,0,2,0,3,0}});
    admission_lsm.status_flags=0x48;
    assert(!lsm_safe_for_sd(&admission_lsm, ID_LSM_MAIN));
    admission_lsm.status_flags=0; // Simulate status latch acknowledged by poll.
    while (!admission_lsm.fifo.empty())
        log_lsm_fifo(&admission_lsm, 0, ID_LSM_MAIN, false);
    assert(admission_lsm.fifo.empty());
    assert(admission_lsm.mode_writes==std::vector<uint8_t>({0,6}));

    reset_ring(); LSM6DSO32Sensor lsm(&SPI,10,10000000);
    lsm.regs[0x63]=0xd3;
    lsm.regs[0x10]=0xa4;
    lsm.fail_register=0x11;
    capture_lsm_registers(&lsm,ID_LSM_MAIN);
    assert(bench_diag.lsm[0].captured && bench_diag.lsm[0].values[13]==0xd3);
    assert((bench_diag.lsm[0].valid_mask & (1<<6))==0);
    assert(bench_diag.lsm[0].valid_mask & (1<<13));
    lsm.fail_register=-1;
    auto add_pairs = [&](int n) {
        for(int i=0;i<n;++i) { lsm.fifo.push_back({{8,1,0,2,0,3,0}}); lsm.fifo.push_back({{16,4,0,5,0,6,0}}); }
    };
    add_pairs(165);
    log_lsm_fifo(&lsm,0,ID_LSM_MAIN);
    assert(metadata().record_count==300 && lsm.fifo.size()==30);
    assert(metadata().first_acc_index==0 && metadata().first_gyro_index==0);
    assert(lsm.mode_writes.empty());
    export_packet();
    reset_ring();
    log_lsm_fifo(&lsm,0,ID_LSM_MAIN,false);
    assert(metadata().record_count==30 && lsm.fifo.empty());
    assert(metadata().first_acc_index==150 && metadata().first_gyro_index==150);
    assert(!(metadata().flags&ACQ_IRQ_VALID) && lsm.mode_writes.empty());
    export_packet();
    const uint32_t last_sequence = metadata().sequence;
    reset_ring();
    log_lsm_fifo(&lsm,0,ID_LSM_MAIN);
    assert(rb_head==0); // Empty stale IRQ must not manufacture a lost packet.

    reset_ring(); add_pairs(165); lsm.status_flags=0x48;
    log_lsm_fifo(&lsm,0,ID_LSM_MAIN);
    assert(metadata().sequence==last_sequence+1);
    assert(metadata().record_count==0 && (metadata().flags&ACQ_RESET) && (metadata().flags&ACQ_OVERRUN));
    assert(lsm.fifo.empty() && lsm.mode_writes==std::vector<uint8_t>({0,6}));
    export_packet();
    const uint32_t previous_epoch=metadata().epoch;
    reset_ring(); add_pairs(150);
    log_lsm_fifo(&lsm,0,ID_LSM_MAIN);
    assert(metadata().epoch==previous_epoch+1 && metadata().first_acc_index==0);
    export_packet();

    reset_ring(); add_pairs(150); lsm.fifo_reads=0; lsm.overrun_after_reads=100;
    log_lsm_fifo(&lsm,0,ID_LSM_MAIN);
    assert(metadata().record_count==0 && (metadata().flags&ACQ_OVERRUN));
    lsm.overrun_after_reads=-1;

    reset_ring(); add_pairs(150); rb_tail=0; rb_head=RING_BUFFER_SIZE-1;
    log_lsm_fifo(&lsm,0,ID_LSM_MAIN);
    assert(acquisition_diag.sensor[3].ring_drops==1 && acquisition_diag.sensor[3].records_read==300);
    reset_ring(); add_pairs(1);
    log_lsm_fifo(&lsm,0,ID_LSM_MAIN,true);
    assert(metadata().first_acc_index==150 && metadata().first_gyro_index==150);
    assert(!(metadata().flags&ACQ_IRQ_VALID)); // Below-watermark stale IRQ is not an anchor.

    reset_ring(); add_pairs(150); lsm.fifo_reads=0; lsm.fail_after_reads=5;
    log_lsm_fifo(&lsm,0,ID_LSM_MAIN);
    assert(metadata().record_count==0 && (metadata().flags&ACQ_IO_ERROR));
    reset_ring();
    trace_sd_write(0xfffffff0U,50000,512,512);
    assert(bench_diag.sd_count==1 && bench_diag.sd_worst.start_us==0xfffffff0U);
    for(int i=0;i<70;++i) trace_sd_write(i,10000,512,512);
    assert(bench_diag.sd_count==SD_TRACE_CAPACITY && bench_diag.sd_omitted==7);
    assert(bench_diag.sd_long_calls==71 && bench_diag.sd_worst.duration_us==50000);
    // Exercise the real SD writer hook with a delayed fake card.
    reset_ring(); ring_buffer_push(bytes.data(),bytes.size()); file.write_delay_us=49069;
    write_buffer_to_sd(true); file.write_delay_us=0;
    assert(bench_diag.sd_count==1 && acquisition_diag.sd_max_write_us>=49069);
    // V2.5: Busy SD cards never trigger optional FIFO SPI probes.
    reset_ring();
    has_lsm_main=has_lsm_sat=has_adxl_main=true;
    has_adxl_sat=false;
    adxl_main.setFifoSamples(300);
    adxl_main_int=adxl_sat_int=lsm_main_int=lsm_sat_int=false;
    log_lsm_fifo(&lsm_main,0,ID_LSM_MAIN,false);
    log_lsm_fifo(&lsm_sat,0,ID_LSM_SAT,false);
    reset_ring(); ring_buffer_push(blocks.data(),blocks.size());
    for(int i=0;i<360;++i) {
        lsm_main.fifo.push_back({{8,1,0,2,0,3,0}});
        lsm_sat.fifo.push_back({{8,1,0,2,0,3,0}});
    }
    // A physical 360-entry refill takes appreciable time; force elapsed
    // time to expire the post-service safe estimate in this fake sensor.
    fake_time += 30000;
    const auto probes_before=lsm_main.status_reads+lsm_sat.status_reads;
    file.busy=true;
    service_sd_if_ready();
    assert(lsm_main.status_reads+lsm_sat.status_reads==probes_before);
    assert(acquisition_diag.sd_headroom_drains==0 && file.output.empty());
    file.busy=false;
    // Pending ADXL IRQ generated DURING optional drain must take priority.
    for (int i=0;i<100;++i) {
        push_axis(SPI,1,true); push_axis(SPI,2,false); push_axis(SPI,3,false);
    }
    bool injected=false;
    lsm_main.on_read=[&](uint8_t reg) {
        if (reg==0x78 && !injected) { injected=true; adxl_main_ISR(); }
    };
    lsm_sat.on_read=[&](uint8_t reg) {
        if (reg==0x78) assert(acquisition_diag.sensor[1].records_queued==100);
    };
    service_sd_if_ready();
    assert(injected && acquisition_diag.sensor[1].records_queued==100);
    assert(acquisition_diag.sd_headroom_drains==1 && file.output.empty());
    assert(lsm_sat.fifo.size()==360);
    // The other unsafe LSM is served next. Both successful drains furnish
    // post-drain FIFO occupancy; SD must now be allowed to write INSIDE
    // their cooldown, with no redundant SPI status reads.
    service_sd_if_ready();
    assert(acquisition_diag.sd_headroom_drains==2);
    const auto probes_cool=lsm_main.status_reads+lsm_sat.status_reads;
    assert(lsm_cached_headroom_valid(ID_LSM_MAIN));
    assert(lsm_cached_headroom_valid(ID_LSM_SAT));
    service_sd_if_ready();
    assert(lsm_main.status_reads+lsm_sat.status_reads==probes_cool);
    assert(file.output.size()==SD_BLOCK_BYTES);
    lsm_main.on_read=nullptr; lsm_sat.on_read=nullptr;
    // When the conservative occupancy estimate really expires, refresh
    // it once from hardware; do not blindly request another FIFO drain.
    fake_time+=40000;
    const auto before_retry=acquisition_diag.sd_headroom_probes;
    service_sd_if_ready();
    assert(acquisition_diag.sd_headroom_probes>before_retry);
    // Urgent watermark servicing is independent of optional-drain cooldown.
    for(int i=0;i<300;++i) lsm_main.fifo.push_back({{8,1,0,2,0,3,0}});
    lsm_main_ISR();
    service_acquisition_pass();
    assert(!lsm_main_int);
    // Cached admission survives micros wrap and expires.
    log_lsm_fifo(&lsm_main,0,ID_LSM_MAIN,false);
    fake_time=0xffffff00U;
    assert(lsm_cached_safe_for_sd(&lsm_main,ID_LSM_MAIN));
    fake_time+=500;
    assert(lsm_cached_headroom_valid(ID_LSM_MAIN));
    fake_time+=30000;
    assert(!lsm_cached_headroom_valid(ID_LSM_MAIN));
    // Latched overrun observed by admission requires recovery.
    log_lsm_fifo(&lsm_main,0,ID_LSM_MAIN,false);
    lsm_main.status_flags=0x48;
    // A latched overrun emerging after an older safe cache requires the
    // forecast to age out before an admission-status read can detect it.
    fake_time += 40000;
    assert(!lsm_cached_safe_for_sd(&lsm_main,ID_LSM_MAIN));
    lsm_main.status_flags=0;
    log_lsm_fifo(&lsm_main,0,ID_LSM_MAIN,false);
    assert(acquisition_diag.sensor[3].fifo_overruns==1);

    // Separate accepted normal-write latency from shutdown latency.
    reset_ring(); ring_buffer_push(blocks.data(),blocks.size());
    file.write_delay_us=SD_ADMISSION_BUDGET_US+100;
    write_buffer_to_sd();
    assert(acquisition_diag.sd_normal_budget_exceeded==1);
    const auto normal_max=acquisition_diag.sd_normal_max_write_us;
    file.write_delay_us=50000; write_buffer_to_sd(true);
    assert(acquisition_diag.sd_normal_max_write_us==normal_max);
    assert(acquisition_diag.sd_max_write_us>=50000);
    file.write_delay_us=0;

    // V2.5 versioned health diagnostics persist in the same binary stream,
    // regardless of DEBUG_ = 0. Exercise periodic, detailed and final paths.
    reset_ring(); reset_diagnostic_logging_for_test();
    fake_time=6000000;
    acquisition_diag.sensor[1].discarded_words=42;
    acquisition_diag.sensor[1].interrupts=UINT32_MAX;
    diag_inc(acquisition_diag.sensor[1].interrupts);
    assert(acquisition_diag.sensor[1].interrupts==UINT32_MAX);
    acquisition_diag.sd_max_write_us=110;
    lsm.regs[0x63]=0xd3;
    capture_lsm_registers(&lsm,ID_LSM_MAIN);
    trace_adxl(ID_ADXL371_MAIN,4,5990000,300,0xffff,14,0x10,13,0x21,nullptr,false);
    trace_sd_write(5900000,12001,512,512,1100);
    service_diagnostic_logging();
    assert(ring_buffer[1]==DIAG_HEALTH_V1);
    CHUNK_HEADER hh; memcpy(&hh,ring_buffer,sizeof(hh));
    assert(hh.payload_len==1004);
    assert(ring_buffer[sizeof(CHUNK_HEADER)]==1);
    // Another health snapshot and detail packets must survive final shutdown.
    finalize_diagnostic_logging(1);
    export_packet();
    reset_ring(); reset_diagnostic_logging_for_test();

    if (fixture.is_open()) {
        CHUNK_HEADER end{0xaa, 0xff, micros(), 0};
        fixture.write((const char*)&end, sizeof(end));
        fixture.close();
    }
    std::cout << "PASS: persistent diagnostics, bounded priority, busy gating, cached headroom/cooldown, SD block batching, wrap, short-write alignment, complete close drain, pending/pressure deferral; original driver/ring checks; ADXL fixed100, tail isolation, marker trace, no-pause stale IRQ, overrun; LSM continuous reads, overrun recovery, reset epochs, indices across ring drops, I/O recovery; register snapshot; bounded SD stall traces\n";
}

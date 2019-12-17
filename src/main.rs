// #![deny(warnings)]

#![no_std]
#![no_main]
#![cfg_attr(feature = "nightly", feature(asm))]
// Enable returning `!`
#![cfg_attr(feature = "nightly", feature(never_type))]
#![cfg_attr(feature = "nightly", feature(core_intrinsics))]

#[inline(never)]
#[panic_handler]
#[cfg(all(feature = "nightly", not(feature = "semihosting")))]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    let gpiod = unsafe { &*pac::GPIOD::ptr() };
    gpiod.odr.modify(|_, w| w.odr6().high().odr12().high());  // FP_LED_1, FP_LED_3
    unsafe { core::intrinsics::abort(); }
}

#[inline(never)]
#[panic_handler]
#[cfg(feature = "seriallog")]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let gpiod = unsafe { &*pac::GPIOD::ptr() };
    gpiod.odr.modify(|_, w| w.odr6().high().odr12().high());  // FP_LED_1, FP_LED_3
    error!("{}", info);
    loop { }
}

#[cfg(feature = "semihosting")]
extern crate panic_semihosting;

#[cfg(not(any(feature = "nightly", feature = "semihosting", feature = "seriallog")))]
extern crate panic_halt;

#[macro_use]
extern crate log;

use core::ptr;
// use core::sync::atomic::{AtomicU32, AtomicBool, Ordering};
use core::fmt::Write;
use cortex_m_rt::exception;
use stm32h7::stm32h743 as pac;
use heapless::{String, Vec, consts::*};
use rtfm::cyccnt::{Instant, U32Ext as _};

use smoltcp as net;

use serde::{Serialize, Deserialize, de::DeserializeOwned};
use serde_json_core::{ser::to_string, de::from_slice};

mod eth;

mod cpu_dac;
use cpu_dac::*;

mod storage;

mod iir;
use iir::*;

mod i2c;
mod eeprom;
mod board;
mod feedforward;

#[cfg(feature = "seriallog")]
struct UartWriter;

#[cfg(feature = "seriallog")]
impl core::fmt::Write for UartWriter {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let data = s.as_bytes();
        for i in 0..data.len() {
            usart3_write(data[i]);
        }
        Ok(())
    }
}


#[cfg(not(any(feature = "semihosting", feature = "seriallog")))]
fn init_log() {}

#[cfg(feature = "semihosting")]
fn init_log() {
    use log::LevelFilter;
    use cortex_m_log::log::{Logger, init as init_log};
    use cortex_m_log::printer::semihosting::{InterruptOk, hio::HStdout};
    static mut LOGGER: Option<Logger<InterruptOk<HStdout>>> = None;
    let logger = Logger {
        inner: InterruptOk::<_>::stdout().unwrap(),
        level: LevelFilter::Info,
    };
    let logger = unsafe {
        LOGGER.get_or_insert(logger)
    };

    init_log(logger).unwrap();
}

#[cfg(feature = "seriallog")]
struct SerialLog;

#[cfg(feature = "seriallog")]
use log::{Record, Level, Metadata, LevelFilter};

#[cfg(feature = "seriallog")]
impl log::Log for SerialLog {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= Level::Info
    }

    fn log(&self, record: &Record) {
        let mut writer = UartWriter{};
        writer.write_fmt(format_args!("{} - {}\n", record.level(), record.args())).ok();
    }

    fn flush(&self) {}
}

const SCALE: f32 = ((1 << 15) - 1) as f32;
#[link_section = ".sram1.datspi"]
static mut DAT: u32 = 0x201;  // EN | CSTART

// static ETHERNET_PENDING: AtomicBool = AtomicBool::new(true);

const TCP_RX_BUFFER_SIZE: usize = 8192;
const TCP_TX_BUFFER_SIZE: usize = 8192;

const DHCP_RX_BUFFER_SIZE: usize = 900;
const DHCP_TX_BUFFER_SIZE: usize = 900;

macro_rules! create_socket {
    ($set:ident, $rx_storage:ident, $tx_storage:ident, $target:ident) => (
        let mut $rx_storage = [0; TCP_RX_BUFFER_SIZE];
        let mut $tx_storage = [0; TCP_TX_BUFFER_SIZE];
        let tcp_rx_buffer = net::socket::TcpSocketBuffer::new(&mut $rx_storage[..]);
        let tcp_tx_buffer = net::socket::TcpSocketBuffer::new(&mut $tx_storage[..]);
        let tcp_socket = net::socket::TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer);
        let $target = $set.add(tcp_socket);
    )
}

#[rtfm::app(device = stm32h7::stm32h743, peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        spi: (pac::SPI1, pac::SPI2, pac::SPI4, pac::SPI5),
        tim: pac::TIM1,
        i2c: pac::I2C2,
        ethernet_periph: (pac::ETHERNET_MAC, pac::ETHERNET_DMA, pac::ETHERNET_MTL),
        cpu_dac: pac::DAC,
        #[init([CPU_DAC {out: 0, en: false}; 2])]
        cpu_dac_ch: [CPU_DAC; 2],
        gpio_hdr_spi: pac::SPI3,  // different use to spi1/2/4/5
        // #[init(storage::RingBuffer {
        //     storage: [0 as u8; storage::STORAGE_SIZE],
        //     tail: AtomicUsize::new(0),
        //     head: AtomicUsize::new(0),
        //     write_lock: AtomicBool::new(false),
        //     read_lock: AtomicBool::new(false)
        // })]
        // adc_buf: RingBuffer<u8>,
        #[init(0 as u32)]
        adc_logging: u32,
        #[init([[0.; 5]; 2])]
        iir_state: [IIRState; 2],
        #[init([IIR { ba: [0., 0., 0., 0., 0.], y_offset: 0., y_min: -SCALE - 1., y_max: SCALE }; 2])]
        iir_ch: [IIR; 2],
        ff_waveform: feedforward::Waveform,
        #[init(feedforward::State{id:0, n_coarse:0, period_correction:0, phase:0})]
        ff_state: feedforward::State,
        #[link_section = ".sram3.eth"]
        #[init(eth::Device::new())]
        ethernet: eth::Device,
    }

    #[init(schedule = [tick])]
    fn init(c: init::Context) -> init::LateResources {
        board::init();
        let dp = unsafe { pac::Peripherals::steal() };
        let ff_waveform = feedforward::Waveform::new();

        init_log();
        // info!("Version {} {}", build_info::PKG_VERSION, build_info::GIT_VERSION.unwrap());
        // info!("Built on {}", build_info::BUILT_TIME_UTC);
        // info!("{} {}", build_info::RUSTC_VERSION, build_info::TARGET);

        init::LateResources {
            spi: (dp.SPI1, dp.SPI2, dp.SPI4, dp.SPI5),
            tim: dp.TIM1,
            i2c: dp.I2C2,
            ff_waveform: ff_waveform,
            ethernet_periph: (dp.ETHERNET_MAC, dp.ETHERNET_DMA, dp.ETHERNET_MTL),
            cpu_dac: dp.DAC,
            gpio_hdr_spi: dp.SPI3,
        }
    }


    // #[idle(resources = [ethernet, ethernet_periph, iir_state, iir_ch, i2c])]
    #[idle(resources = [ethernet, ethernet_periph, cpu_dac, cpu_dac_ch,
                        iir_state, iir_ch, i2c, gpio_hdr_spi, adc_logging, ff_state, ff_waveform])]
    // #[idle(resources = [ethernet, ethernet_periph,
    //                     iir_state, iir_ch, i2c, gpio_hdr_spi, adc_logging])]
    fn idle(c: idle::Context) -> ! {
        let (MAC, DMA, MTL) = c.resources.ethernet_periph;
        let use_dhcp = false;

        board::late_init();

        let hardware_addr = match eeprom::read_eui48(c.resources.i2c) {
            Err(_) => {
                info!("Could not read EEPROM, using default MAC address");
                net::wire::EthernetAddress([0xb0, 0xd5, 0xcc, 0xfc, 0xfb, 0xf6])

            },
            // Ok(raw_mac) => net::wire::EthernetAddress(raw_mac)
            Ok(raw_mac) => net::wire::EthernetAddress([0xb0, 0xd5, 0xcc, 0xfc, 0xfb, 0xf6])
        };
        info!("MAC: {}", hardware_addr);

        let local_addr = if use_dhcp {
            info!("Using DHCP");
            net::wire::Ipv4Address::UNSPECIFIED.into()
        } else {
            let static_ip = net::wire::IpAddress::v4(10, 255, 6, 56);
            info!("Using static IP: {}", static_ip);
            static_ip
        };

        unsafe { c.resources.ethernet.init(hardware_addr, MAC, DMA, MTL) };
        let mut neighbor_cache_storage = [None; 8];
        let neighbor_cache = net::iface::NeighborCache::new(&mut neighbor_cache_storage[..]);
        let mut ip_addrs = [net::wire::IpCidr::new(local_addr, 24)];
        let mut routes_storage = [None; 1];
        let routes = net::iface::Routes::new(&mut routes_storage[..]);
        let mut iface = net::iface::EthernetInterfaceBuilder::new(c.resources.ethernet)
                    .ethernet_addr(hardware_addr)
                    .neighbor_cache(neighbor_cache)
                    .ip_addrs(&mut ip_addrs[..])
                    .routes(routes)
                    .finalize();
        let mut socket_set_entries: [_; 8] = Default::default();
        let mut sockets = net::socket::SocketSet::new(&mut socket_set_entries[..]);
        create_socket!(sockets, tcp_rx_storage0, tcp_tx_storage0, tcp_handle0);
        create_socket!(sockets, tcp_rx_storage1, tcp_tx_storage1, tcp_handle1);
        create_socket!(sockets, tcp_rx_storage2, tcp_tx_storage2, tcp_handle2);
        create_socket!(sockets, tcp_rx_storage3, tcp_tx_storage3, tcp_handle3);
        create_socket!(sockets, tcp_rx_storage4, tcp_tx_storage4, tcp_handle4);

        let mut dhcp_rx_storage = [0u8; DHCP_RX_BUFFER_SIZE];
        let mut dhcp_tx_storage = [0u8; DHCP_TX_BUFFER_SIZE];
        let mut dhcp_rx_metadata = [net::socket::RawPacketMetadata::EMPTY; 1];
        let mut dhcp_tx_metadata = [net::socket::RawPacketMetadata::EMPTY; 1];

        let dhcp_rx_buffer = net::socket::RawSocketBuffer::new(
            &mut dhcp_rx_metadata[..],
            &mut dhcp_rx_storage[..]
        );
        let dhcp_tx_buffer = net::socket::RawSocketBuffer::new(
            &mut dhcp_tx_metadata[..],
            &mut dhcp_tx_storage[..]
        );
        let mut dhcp = net::dhcp::Dhcpv4Client::new(&mut sockets, dhcp_rx_buffer, dhcp_tx_buffer, net::time::Instant::from_millis(0));
        let mut prev_cidr = net::wire::Ipv4Cidr::new(net::wire::Ipv4Address::UNSPECIFIED, 0);

        // unsafe { eth::enable_interrupt(DMA); }
        let mut time = 0u32;
        let mut next_ms = Instant::now();
        next_ms += 400_000.cycles();
        let mut server = Server::new();
        let mut iir_state: resources::iir_state = c.resources.iir_state;
        let mut iir_ch: resources::iir_ch = c.resources.iir_ch;
        let mut ff_waveform: resources::ff_waveform = c.resources.ff_waveform;
        let mut ff_state: resources::ff_state = c.resources.ff_state;
        let mut last_id: u32 = 0;
        let mut cpu_dac_ch = c.resources.cpu_dac_ch;
        let mut gpio_hdr_spi = c.resources.gpio_hdr_spi;
        // let mut adc_buf = c.resources.adc_buf;
        let mut adc_logging = c.resources.adc_logging;

        loop {
            // if ETHERNET_PENDING.swap(false, Ordering::Relaxed) { }
            let tick = Instant::now() > next_ms;
            if tick {
                next_ms += 400_000.cycles();
                time += 1;
            }
            {
                let socket = &mut *sockets.get::<net::socket::TcpSocket>(tcp_handle0);
                if socket.state() == net::socket::TcpState::CloseWait {
                    socket.close();
                } else if !(socket.is_open() || socket.is_listening()) {
                    socket.listen(1234).unwrap_or_else(|e| warn!("TCP listen error: {:?}", e));
                } else if tick && socket.can_send() {
                    let mut buf = [0u8; 10];
                    unsafe {storage::ADC_BUF.dequeue_into(&mut buf)};
                    let s = iir_state.lock(|iir_state| (Status {
                        t: time,
                        x0: iir_state[0][0],
                        y0: iir_state[0][2],
                        x1: iir_state[1][0],
                        y1: iir_state[1][2]
                    }, buf));
                    json_reply(socket, &s);
                }
            }
            {
                let socket = &mut *sockets.get::<net::socket::TcpSocket>(tcp_handle1);
                if socket.state() == net::socket::TcpState::CloseWait {
                    socket.close();
                } else if !(socket.is_open() || socket.is_listening()) {
                    socket.listen(1235).unwrap_or_else(|e| warn!("TCP listen error: {:?}", e));
                } else {
                    server.poll(socket, |req: &Request| {
                        if req.channel < 2 {
                            cpu_dac_ch[req.channel as usize] = req.cpu_dac;
                            iir_ch.lock(|iir_ch| iir_ch[req.channel as usize] = req.iir);
                            let word: u16 = req.gpio_hdr_spi;
                            let txdr = &gpio_hdr_spi.txdr as *const _ as *mut u16;
                            unsafe { ptr::write_volatile(txdr, word) };

                            let gpiod = unsafe { &*pac::GPIOD::ptr() };
                            gpiod.odr.modify(|_, w| w.odr6().low().odr12().low().odr5().high());  // FP_LED_1, FP_LED_3
                        }
                    });
                }
            }
            {
                let socket = &mut *sockets.get::<net::socket::TcpSocket>(tcp_handle2);
                if socket.state() == net::socket::TcpState::CloseWait {
                    socket.close();
                } else if !(socket.is_open() || socket.is_listening()) {
                    socket.listen(1236).unwrap_or_else(|e| warn!("TCP listen error: {:?}", e));
                } else if tick && socket.can_send() {
                    let s = ff_state.lock(|ff_state| feedforward::State{..*ff_state});
                    if s.id != last_id {
                        json_reply(socket, &s);
                        last_id = s.id;
                    }
                }
            }
            {
                let socket = &mut *sockets.get::<net::socket::TcpSocket>(tcp_handle3);
                if socket.state() == net::socket::TcpState::CloseWait {
                    socket.close();
                } else if !(socket.is_open() || socket.is_listening()) {
                    socket.listen(1237).unwrap_or_else(|e| warn!("TCP listen error: {:?}", e));
                } else {
                    server.poll(socket, |req: &feedforward::Settings| {
                        ff_waveform.lock(|wav| wav.update(req));
                    });
                }
            }
            {
                let socket = &mut *sockets.get::<net::socket::TcpSocket>(tcp_handle4);
                if socket.state() == net::socket::TcpState::CloseWait {
                    socket.close();
                    info!("close");
                } else if !(socket.is_open() || socket.is_listening()) {
                    socket.listen(1238).unwrap_or_else(|e| warn!("TCP listen error: {:?}", e));
                    adc_logging.lock(|adc_logging| {*adc_logging = 0; unsafe { storage::ADC_BUF.clear() }});
                    info!("clear buf");
                } else if socket.can_send() {
                    socket.send(|buf| unsafe {
                        let sent = storage::ADC_BUF.dequeue_into(buf);
                        (sent, sent)
                    }).unwrap();

                    adc_logging.lock(|adc_logging|
                                     if *adc_logging == 0 {
                                        info!("start logging");
                                        *adc_logging = 1;
                                     })
                }
            }
            let timestamp = net::time::Instant::from_millis(time as i64);
            if !match iface.poll(&mut sockets, timestamp) {
                Ok(changed) => changed,
                Err(net::Error::Unrecognized) => true,
                Err(e) => { info!("iface poll error: {:?}", e); true }
            } {
                // cortex_m::asm::wfi();
            }

            c.resources.cpu_dac.cr.modify(|_, w| {
                let mut temp = match cpu_dac_ch[0].en{
                    true => w.en1().set_bit(),
                    false => w.en1().clear_bit(),
                };
                match cpu_dac_ch[1].en{
                    true => temp.en2().set_bit(),
                    false => temp.en2().clear_bit(),
                }
            });

            c.resources.cpu_dac.dhr12r1.write(|w| unsafe {
                w.dacc1dhr().bits(cpu_dac_ch[0].out)});
            c.resources.cpu_dac.dhr12r2.write(|w| unsafe {
                w.dacc2dhr().bits(cpu_dac_ch[1].out)});

            if use_dhcp {
                let config = dhcp.poll(&mut iface, &mut sockets, timestamp)
                .unwrap_or_else(|e| {
                    if e != net::Error::Unrecognized {
                        info!("DHCP poll error: {:?}", e);
                    }
                    None
                });

                config.map(|config| {
                    match config.address {
                        Some(cidr) => if cidr != prev_cidr {
                            iface.update_ip_addrs(|addrs| {
                                addrs.iter_mut().nth(0)
                                    .map(|addr| {
                                        *addr = net::wire::IpCidr::Ipv4(cidr);
                                    });
                            });
                            prev_cidr = cidr;
                            info!("Assigned a new IP address: {}", cidr);
                        }
                        _ => {}
                    }
                    config.router.map(|router| iface.routes_mut()
                                      .add_default_ipv4_route(router.into())
                                      .unwrap()
                    );
                });
            }
        }
    }

    #[task(priority = 1, schedule = [tick])]
    fn tick(c: tick::Context) {
        static mut TIME: u32 = 0;
        *TIME += 1;
        const PERIOD: u32 = 200_000_000;
        c.schedule.tick(c.scheduled + PERIOD.cycles()).unwrap();
    }

    #[task(binds = TIM1_UP, resources = [tim, ff_state, ff_waveform], priority = 2) ]
    fn feedforward_timer(c: feedforward_timer::Context) {
        static mut N: u32 = 0;
        static mut ID: u32 = 0;
        static mut PHASE_INT: i32 = 0;

        let tim1 = c.resources.tim;
        let ff_state = c.resources.ff_state;
        let waveform = c.resources.ff_waveform;
        let sr = tim1.sr.read();

        if sr.uif().bit_is_set() {
            tim1.sr.write(|w| w.uif().clear_bit() );
            *N += 1;
            if *N == feedforward::N_LOOKUP as u32 {
                *N = 0;
            }
            let y = waveform.amplitude[*N as usize];
            let dac_val = y as u16 ^ 0x8000;
            let spi = unsafe { &*pac::SPI4::ptr() };
            let txdr = &spi.txdr as *const _ as *mut u16;
            unsafe { ptr::write_volatile(txdr, dac_val) };
        }

        if sr.cc1if().bit_is_set() {
            tim1.sr.write(|w| w.cc1if().clear_bit() );
            *ID += 1;
            let n_fine = tim1.ccr1.read().bits();
            let phase = n_fine + (*N)*feedforward::TMR_ARR_NOMINAL;
            let phase_error = phase as i32 - (feedforward::N_LOOKUP as i32 + 1)*(feedforward::TMR_ARR_NOMINAL as i32)/2;
            ff_state.id = *ID;
            ff_state.n_coarse = *N;
            ff_state.phase = phase_error;

            *PHASE_INT += phase_error;
            let mut period_correction: i32 = *PHASE_INT/10000 + phase_error/500;
            if period_correction > 1000 {period_correction=1000;}
            if period_correction < -1000 {period_correction=-1000;}
            ff_state.period_correction = period_correction;
            let period = feedforward::TMR_ARR_NOMINAL as i32 + period_correction;
            tim1.arr.write(|w| unsafe { w.bits(period as u32) });
        }

    }

    // seems to slow it down
    // #[link_section = ".data.spi1"]
    #[task(binds = SPI1, resources = [spi, iir_state, iir_ch,adc_logging], priority = 3)]
    fn spi1(c: spi1::Context) {
        #[cfg(feature = "bkpt")]
        cortex_m::asm::bkpt();

        let gpiod = unsafe { &*pac::GPIOD::ptr() };
        gpiod.odr.modify(|r, w| w.odr5().high());
        let gpiog = unsafe { &*pac::GPIOG::ptr() };
        gpiog.odr.modify(|r, w| w.odr4().bit(!r.odr4().bit_is_set()));
        let (spi1, spi2, spi4, spi5) = c.resources.spi;
        let iir_ch = c.resources.iir_ch;
        let iir_state = c.resources.iir_state;
        let mut adc_logging = c.resources.adc_logging;
        let sr = spi1.sr.read();
        // *adc_logging = sr.bits();
        if sr.eot().bit_is_set() {
            spi1.ifcr.write(|w| w.eotc().set_bit());
        }
        if sr.rxp().bit_is_set() {
            let rxdr = &spi1.rxdr as *const _ as *const u16;
            let a = unsafe { ptr::read_volatile(rxdr) };
            let x0 = f32::from(a as i16);
            let y0 = iir_ch[0].update(&mut iir_state[0], x0);
            let d = y0 as i16 as u16 ^ 0x8000;
            let txdr = &spi2.txdr as *const _ as *mut u16;
            unsafe { ptr::write_volatile(txdr, d) };

            if *adc_logging == 1 {
                 unsafe {
                        let sample = [a as u8, (a >> 8) as u8];
                        storage::ADC_BUF.enqueue_slice(&sample).unwrap_or_else(|_| {
                            *adc_logging = 2;
                            error!("storage full");
                        });
                    }
            }
        }
        /*
        let sr = spi5.sr.read();
        if sr.eot().bit_is_set() {
            spi5.ifcr.write(|w| w.eotc().set_bit());
        }
        if sr.rxp().bit_is_set() {
            let rxdr = &spi5.rxdr as *const _ as *const u16;
            let a = unsafe { ptr::read_volatile(rxdr) };
            let x0 = f32::from(a as i16);
            // let y0 = iir_ch[1].update(&mut iir_state[1], x0);
            // let d = y0 as i16 as u16 ^ 0x8000;
        }
        // work around for stabilizer_current_sense issue #9
        let d = 0xff as u16;
        let txdr = &spi4.txdr as *const _ as *mut u16;
        unsafe { ptr::write_volatile(txdr, d) };
        #[cfg(feature = "bkpt")]
        cortex_m::asm::bkpt();
        */
    }

    /*
    #[task(binds = ETH, resources = [ethernet_periph], priority = 1)]
    fn eth(c: eth::Context) {
        let dma = &c.resources.ethernet_periph.1;
        ETHERNET_PENDING.store(true, Ordering::Relaxed);
        unsafe { eth::interrupt_handler(dma) }
    }
    */

    extern "C" {
        // hw interrupt handlers for RTFM to use for scheduling tasks
        // one per priority
        fn DCMI();
        fn JPEG();
        fn SDMMC();
    }
};

#[derive(Deserialize,Serialize)]
struct Request {
    channel: u8,
    iir: IIR,
    cpu_dac: CPU_DAC,
    gpio_hdr_spi: u16,
}

#[derive(Serialize)]
struct Response<'a> {
    code: i32,
    message: &'a str,
}

#[derive(Serialize)]
struct Status {
    t: u32,
    x0: f32,
    y0: f32,
    x1: f32,
    y1: f32
}

fn json_reply<T: Serialize>(socket: &mut net::socket::TcpSocket, msg: &T) {
    let mut u: String<U128> = to_string(msg).unwrap();
    u.push('\n').unwrap();
    socket.write_str(&u).unwrap();
}

struct Server {
    data: Vec<u8, U256>,
    discard: bool,
}

impl Server {
    fn new() -> Self {
        Self { data: Vec::new(), discard: false }
    }

    fn poll<T, F, R>(&mut self, socket: &mut net::socket::TcpSocket, f: F) -> Option<R>
        where
            T: DeserializeOwned,
            F: FnOnce(&T) -> R,
    {// attempts to run f on received data, returns option of f(response)
        while socket.can_recv() {
            let found = socket.recv(|buf| {
                let (len, found) = match buf.iter().position(|&c| c as char == '\n') {
                    Some(end) => (end + 1, true),
                    None => (buf.len(), false),
                };
                if self.data.len() + len >= self.data.capacity() {
                    self.discard = true;
                    self.data.clear();
                } else if !self.discard && len > 0 {
                    self.data.extend_from_slice(&buf[..len]).unwrap();
                }
                (len, found)
            }).unwrap();
            if found {
                if self.discard {
                    self.discard = false;
                    json_reply(socket, &Response { code: 520, message: "command buffer overflow" });
                    self.data.clear();
                } else {
                    let r = from_slice::<T>(&self.data[..self.data.len() - 1]);
                    self.data.clear();
                    match r {
                        Ok(res) => {
                            let r = f(&res);
                            json_reply(socket, &Response { code: 200, message: "ok" });
                            self.data.clear();
                            return Some(r);
                        },
                        Err(err) => {
                            warn!("parse error {:?}", err);
                            json_reply(socket, &Response { code: 550, message: "parse error" });
                        },
                    }
                    self.data.clear();
                }
            }
        }
        None
    }
}

#[exception]
fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("HardFault at {:#?}", ef);
}

#[exception]
fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}

//! Tock kernel for the Adafruit Feather nrf52 pro. </br>
//! It is based on nRF52838 SoC (Cortex M4 core with a BLE transceiver).
//!
//! Author
//! -------------------
//! * Niklas Adolfsson <niklasadolfsson1@gmail.com>
//! * July 16, 2017

#![no_std]
#![no_main]
#![deny(missing_docs)]

#[allow(unused_imports)]
use kernel::{create_capability, debug, debug_gpio, debug_verbose, static_init};

use capsules::virtual_alarm::VirtualMuxAlarm;
use capsules::virtual_spi::MuxSpiMaster;
use capsules::virtual_uart::MuxUart;
use kernel::capabilities;
use kernel::component::Component;
use kernel::hil;
use nrf52::uicr::Regulator0Output;
use nrf52832::gpio::Pin;
use nrf52dk_base::nrf52_components::ble::BLEComponent;
use nrf52dk_base::nrf52_components::ieee802154::Ieee802154Component;
use nrf52dk_base::SpiPins;

// Constants related to the configuration of the 15.4 network stack
const SRC_MAC: u16 = 0xf00f;
const PAN_ID: u16 = 0xABCD;

const LED1_PIN: Pin = Pin::P0_17;
const LED2_PIN: Pin = Pin::P0_19;
const DEBUG_PIN: Pin = Pin::P0_18;

const BUTTON1_PIN: Pin = Pin::P0_20;
const BUTTON_RST_PIN: Pin = Pin::P0_21;

const UART_CTS: Pin = Pin::P0_05;
const UART_TXD: Pin = Pin::P0_06;
const UART_RTS: Pin = Pin::P0_07;
const UART_RXD: Pin = Pin::P0_08;

const SPI_CLK: Pin = Pin::P0_12;
const SPI_MOSI: Pin = Pin::P0_13;
const SPI_MISO: Pin = Pin::P0_14;

/// UART Writer
pub mod io;

// State for loading and holding applications.
// How should the kernel respond when a process faults.
const FAULT_RESPONSE: kernel::procs::FaultResponse = kernel::procs::FaultResponse::Panic;

// Number of concurrent processes this platform supports.
const NUM_PROCS: usize = 4;

#[link_section = ".app_memory"]
static mut APP_MEMORY: [u8; 32768] = [0; 32768];

static mut PROCESSES: [Option<&'static dyn kernel::procs::ProcessType>; NUM_PROCS] =
    [None, None, None, None];

/// Dummy buffer that causes the linker to reserve enough space for the stack.
#[no_mangle]
#[link_section = ".stack_buffer"]
pub static mut STACK_MEMORY: [u8; 0x1000] = [0; 0x1000];

/// Entry point in the vector table called on hard reset.
#[no_mangle]
pub unsafe fn reset_handler() {
    // Loads relocations and clears BSS
    nrf52832::init();

    let gpio_pins = static_init!(
        [&'static dyn kernel::hil::gpio::InterruptValuePin; 11],
        [
            static_init!(
                kernel::hil::gpio::InterruptValueWrapper,
                kernel::hil::gpio::InterruptValueWrapper::new(&nrf52832::gpio::PORT[Pin::P0_02])
            )
            .finalize(),
            static_init!(
                kernel::hil::gpio::InterruptValueWrapper,
                kernel::hil::gpio::InterruptValueWrapper::new(&nrf52832::gpio::PORT[Pin::P0_03])
            )
            .finalize(),
            static_init!(
                kernel::hil::gpio::InterruptValueWrapper,
                kernel::hil::gpio::InterruptValueWrapper::new(&nrf52832::gpio::PORT[Pin::P0_04])
            )
            .finalize(),
            static_init!(
                kernel::hil::gpio::InterruptValueWrapper,
                kernel::hil::gpio::InterruptValueWrapper::new(&nrf52832::gpio::PORT[Pin::P0_11])
            )
            .finalize(),
            static_init!(
                kernel::hil::gpio::InterruptValueWrapper,
                kernel::hil::gpio::InterruptValueWrapper::new(&nrf52832::gpio::PORT[Pin::P0_15])
            )
            .finalize(),
            static_init!(
                kernel::hil::gpio::InterruptValueWrapper,
                kernel::hil::gpio::InterruptValueWrapper::new(&nrf52832::gpio::PORT[Pin::P0_16])
            )
            .finalize(),
            static_init!(
                kernel::hil::gpio::InterruptValueWrapper,
                kernel::hil::gpio::InterruptValueWrapper::new(&nrf52832::gpio::PORT[Pin::P0_27])
            )
            .finalize(),
            static_init!(
                kernel::hil::gpio::InterruptValueWrapper,
                kernel::hil::gpio::InterruptValueWrapper::new(&nrf52832::gpio::PORT[Pin::P0_28])
            )
            .finalize(),
            static_init!(
                kernel::hil::gpio::InterruptValueWrapper,
                kernel::hil::gpio::InterruptValueWrapper::new(&nrf52832::gpio::PORT[Pin::P0_29])
            )
            .finalize(),
            static_init!(
                kernel::hil::gpio::InterruptValueWrapper,
                kernel::hil::gpio::InterruptValueWrapper::new(&nrf52832::gpio::PORT[Pin::P0_30])
            )
            .finalize(),
            static_init!(
                kernel::hil::gpio::InterruptValueWrapper,
                kernel::hil::gpio::InterruptValueWrapper::new(&nrf52832::gpio::PORT[Pin::P0_31])
            )
            .finalize(),
        ]
    );

    // LEDs
    let led_pins = static_init!(
        [(
            &'static dyn kernel::hil::gpio::Pin,
            capsules::led::ActivationMode
        ); 2],
        [
            (
                &nrf52832::gpio::PORT[LED1_PIN],
                capsules::led::ActivationMode::ActiveHigh
            ),
            (
                &nrf52832::gpio::PORT[LED2_PIN],
                capsules::led::ActivationMode::ActiveHigh
            ),
        ]
    );

    let button_pins = static_init!(
        [(
            &'static dyn kernel::hil::gpio::InterruptValuePin,
            capsules::button::GpioMode
        ); 1],
        [
            (
                static_init!(
                    kernel::hil::gpio::InterruptValueWrapper,
                    kernel::hil::gpio::InterruptValueWrapper::new(
                        &nrf52832::gpio::PORT[BUTTON1_PIN]
                    )
                )
                .finalize(),
                capsules::button::GpioMode::LowWhenPressed
            ), // 20
        ]
    );

    for &(btn, _) in button_pins.iter() {
        btn.set_floating_state(kernel::hil::gpio::FloatingState::PullUp);
    }

    let board_kernel = static_init!(kernel::Kernel, kernel::Kernel::new(&PROCESSES));

    setup_board(
        board_kernel,
        BUTTON_RST_PIN,
        &nrf52832::gpio::PORT,
        gpio_pins,
        LED1_PIN,
        LED2_PIN,
        led_pins,
        (UART_TXD, UART_RXD),
        &SpiPins::new(SPI_MOSI, SPI_MISO, SPI_CLK),
        button_pins,
        false,
        &mut APP_MEMORY,
        &mut PROCESSES,
        FAULT_RESPONSE,
        nrf52832::uicr::Regulator0Output::DEFAULT,
        false,
    );
}

/// Generic function for starting an nrf52dk board.
#[inline]
unsafe fn setup_board(
    board_kernel: &'static kernel::Kernel,
    button_rst_pin: Pin,
    gpio_port: &'static nrf52::gpio::Port,
    gpio_pins: &'static mut [&'static dyn kernel::hil::gpio::InterruptValuePin],
    debug_pin1_index: Pin,
    debug_pin2_index: Pin,
    led_pins: &'static mut [(
        &'static dyn kernel::hil::gpio::Pin,
        capsules::led::ActivationMode,
    )],
    uart_pins: (Pin, Pin),
    spi_pins: &SpiPins,
    button_pins: &'static mut [(
        &'static dyn kernel::hil::gpio::InterruptValuePin,
        capsules::button::GpioMode,
    )],
    ieee802154: bool,
    app_memory: &mut [u8],
    process_pointers: &'static mut [Option<&'static dyn kernel::procs::ProcessType>],
    app_fault_response: kernel::procs::FaultResponse,
    reg_vout: Regulator0Output,
    nfc_as_gpios: bool,
) {
    // Make non-volatile memory writable and activate the reset button
    let uicr = nrf52::uicr::Uicr::new();

    // Check if we need to erase UICR memory to re-program it
    // This only needs to be done when a bit needs to be flipped from 0 to 1.
    let psel0_reset: u32 = uicr.get_psel0_reset_pin().map_or(0, |pin| pin as u32);
    let psel1_reset: u32 = uicr.get_psel1_reset_pin().map_or(0, |pin| pin as u32);
    let mut erase_uicr = ((!psel0_reset & (button_rst_pin as u32))
        | (!psel1_reset & (button_rst_pin as u32))
        | (!(uicr.get_vout() as u32) & (reg_vout as u32)))
        != 0;

    // Only enabling the NFC pin protection requires an erase.
    if nfc_as_gpios {
        erase_uicr |= !uicr.is_nfc_pins_protection_enabled();
    }

    if erase_uicr {
        nrf52::nvmc::NVMC.erase_uicr();
    }

    nrf52::nvmc::NVMC.configure_writeable();
    while !nrf52::nvmc::NVMC.is_ready() {}

    let mut needs_soft_reset: bool = false;

    // Configure reset pins
    if uicr
        .get_psel0_reset_pin()
        .map_or(true, |pin| pin != button_rst_pin)
    {
        uicr.set_psel0_reset_pin(button_rst_pin);
        while !nrf52::nvmc::NVMC.is_ready() {}
        needs_soft_reset = true;
    }
    if uicr
        .get_psel1_reset_pin()
        .map_or(true, |pin| pin != button_rst_pin)
    {
        uicr.set_psel1_reset_pin(button_rst_pin);
        while !nrf52::nvmc::NVMC.is_ready() {}
        needs_soft_reset = true;
    }

    // Configure voltage regulator output
    if uicr.get_vout() != reg_vout {
        uicr.set_vout(reg_vout);
        while !nrf52::nvmc::NVMC.is_ready() {}
        needs_soft_reset = true;
    }

    // Check if we need to free the NFC pins for GPIO
    if nfc_as_gpios {
        uicr.set_nfc_pins_protection(true);
        while !nrf52::nvmc::NVMC.is_ready() {}
        needs_soft_reset = true;
    }

    // Any modification of UICR needs a soft reset for the changes to be taken into account.
    if needs_soft_reset {
        cortexm4::scb::reset();
    }

    // Create capabilities that the board needs to call certain protected kernel
    // functions.
    let process_management_capability =
        create_capability!(capabilities::ProcessManagementCapability);
    let main_loop_capability = create_capability!(capabilities::MainLoopCapability);
    let memory_allocation_capability = create_capability!(capabilities::MemoryAllocationCapability);

    // Configure kernel debug gpios as early as possible
    kernel::debug::assign_gpios(
        Some(&gpio_port[debug_pin1_index]),
        Some(&gpio_port[debug_pin2_index]),
        None,
    );

    let gpio = static_init!(
        capsules::gpio::GPIO<'static>,
        capsules::gpio::GPIO::new(
            gpio_pins,
            board_kernel.create_grant(&memory_allocation_capability)
        )
    );

    for pin in gpio_pins.iter() {
        pin.set_client(gpio);
    }

    // LEDs
    let led = static_init!(
        capsules::led::LED<'static>,
        capsules::led::LED::new(led_pins)
    );

    // Buttons
    let button = static_init!(
        capsules::button::Button<'static>,
        capsules::button::Button::new(
            button_pins,
            board_kernel.create_grant(&memory_allocation_capability)
        )
    );

    for (pin, _) in button_pins.iter() {
        pin.set_client(button);
    }

    let rtc = &nrf52::rtc::RTC;
    rtc.start();
    let mux_alarm = static_init!(
        capsules::virtual_alarm::MuxAlarm<'static, nrf52::rtc::Rtc>,
        capsules::virtual_alarm::MuxAlarm::new(&nrf52::rtc::RTC)
    );
    hil::time::Alarm::set_client(rtc, mux_alarm);

    let alarm = components::alarm::AlarmDriverComponent::new(board_kernel, mux_alarm)
        .finalize(components::alarm_component_helper!(nrf52::rtc::Rtc));

    // Create a shared UART channel for the console and for kernel debug.
    let uart_mux = static_init!(
        MuxUart<'static>,
        MuxUart::new(
            &nrf52::uart::UARTE0,
            &mut capsules::virtual_uart::RX_BUF,
            115200
        )
    );
    uart_mux.initialize();
    hil::uart::Transmit::set_transmit_client(&nrf52::uart::UARTE0, uart_mux);
    hil::uart::Receive::set_receive_client(&nrf52::uart::UARTE0, uart_mux);

    nrf52::uart::UARTE0.initialize(
        nrf52::pinmux::Pinmux::new(uart_pins.0 as u32),
        nrf52::pinmux::Pinmux::new(uart_pins.1 as u32),
        None,
        None,
    );

    // Setup the console.
    let console = components::console::ConsoleComponent::new(board_kernel, uart_mux).finalize(());
    // Create the debugger object that handles calls to `debug!()`.
    components::debug_writer::DebugWriterComponent::new(uart_mux).finalize(());

    let ble_radio =
        BLEComponent::new(board_kernel, &nrf52::ble_radio::RADIO, mux_alarm).finalize(());

    let ieee802154_radio = if ieee802154 {
        let (radio, _) = Ieee802154Component::new(
            board_kernel,
            &nrf52::ieee802154_radio::RADIO,
            PAN_ID,
            SRC_MAC,
        )
        .finalize(());
        Some(radio)
    } else {
        None
    };

    let temp = static_init!(
        capsules::temperature::TemperatureSensor<'static>,
        capsules::temperature::TemperatureSensor::new(
            &nrf52::temperature::TEMP,
            board_kernel.create_grant(&memory_allocation_capability)
        )
    );
    kernel::hil::sensors::TemperatureDriver::set_client(&nrf52::temperature::TEMP, temp);

    let rng = components::rng::RngComponent::new(board_kernel, &nrf52::trng::TRNG).finalize(());

    // SPI
    let mux_spi = static_init!(
        MuxSpiMaster<'static, nrf52::spi::SPIM>,
        MuxSpiMaster::new(&nrf52::spi::SPIM0)
    );
    hil::spi::SpiMaster::set_client(&nrf52::spi::SPIM0, mux_spi);
    hil::spi::SpiMaster::init(&nrf52::spi::SPIM0);
    nrf52::spi::SPIM0.configure(
        nrf52::pinmux::Pinmux::new(spi_pins.mosi as u32),
        nrf52::pinmux::Pinmux::new(spi_pins.miso as u32),
        nrf52::pinmux::Pinmux::new(spi_pins.clk as u32),
    );

    let nonvolatile_storage: Option<
        &'static capsules::nonvolatile_storage_driver::NonvolatileStorage<'static>,
    > = if let Some(driver) = mx25r6435f {
        // Create a SPI device for the mx25r6435f flash chip.
        let mx25r6435f_spi = static_init!(
            capsules::virtual_spi::VirtualSpiMasterDevice<'static, nrf52::spi::SPIM>,
            capsules::virtual_spi::VirtualSpiMasterDevice::new(
                mux_spi,
                &gpio_port[driver.chip_select]
            )
        );
        // Create an alarm for this chip.
        let mx25r6435f_virtual_alarm = static_init!(
            VirtualMuxAlarm<'static, nrf52::rtc::Rtc>,
            VirtualMuxAlarm::new(mux_alarm)
        );
        // Setup the actual MX25R6435F driver.
        let mx25r6435f = static_init!(
            capsules::mx25r6435f::MX25R6435F<
                'static,
                capsules::virtual_spi::VirtualSpiMasterDevice<'static, nrf52::spi::SPIM>,
                nrf52::gpio::GPIOPin,
                VirtualMuxAlarm<'static, nrf52::rtc::Rtc>,
            >,
            capsules::mx25r6435f::MX25R6435F::new(
                mx25r6435f_spi,
                mx25r6435f_virtual_alarm,
                &mut capsules::mx25r6435f::TXBUFFER,
                &mut capsules::mx25r6435f::RXBUFFER,
                Some(&gpio_port[driver.write_protect_pin]),
                Some(&gpio_port[driver.hold_pin])
            )
        );
        mx25r6435f_spi.set_client(mx25r6435f);
        hil::time::Alarm::set_client(mx25r6435f_virtual_alarm, mx25r6435f);

        pub static mut FLASH_PAGEBUFFER: capsules::mx25r6435f::Mx25r6435fSector =
            capsules::mx25r6435f::Mx25r6435fSector::new();
        let nv_to_page = static_init!(
            capsules::nonvolatile_to_pages::NonvolatileToPages<
                'static,
                capsules::mx25r6435f::MX25R6435F<
                    'static,
                    capsules::virtual_spi::VirtualSpiMasterDevice<'static, nrf52::spi::SPIM>,
                    nrf52::gpio::GPIOPin,
                    VirtualMuxAlarm<'static, nrf52::rtc::Rtc>,
                >,
            >,
            capsules::nonvolatile_to_pages::NonvolatileToPages::new(
                mx25r6435f,
                &mut FLASH_PAGEBUFFER
            )
        );
        hil::flash::HasClient::set_client(mx25r6435f, nv_to_page);

        let nonvolatile_storage = static_init!(
            capsules::nonvolatile_storage_driver::NonvolatileStorage<'static>,
            capsules::nonvolatile_storage_driver::NonvolatileStorage::new(
                nv_to_page,
                board_kernel.create_grant(&memory_allocation_capability),
                0x60000, // Start address for userspace accessible region
                0x20000, // Length of userspace accessible region
                0,       // Start address of kernel accessible region
                0x60000, // Length of kernel accessible region
                &mut capsules::nonvolatile_storage_driver::BUFFER
            )
        );
        hil::nonvolatile_storage::NonvolatileStorage::set_client(nv_to_page, nonvolatile_storage);
        Some(nonvolatile_storage)
    } else {
        None
    };

    // Start all of the clocks. Low power operation will require a better
    // approach than this.
    nrf52::clock::CLOCK.low_stop();
    nrf52::clock::CLOCK.high_stop();

    nrf52::clock::CLOCK.low_set_source(nrf52::clock::LowClockSource::XTAL);
    nrf52::clock::CLOCK.low_start();
    nrf52::clock::CLOCK.high_set_source(nrf52::clock::HighClockSource::XTAL);
    nrf52::clock::CLOCK.high_start();
    while !nrf52::clock::CLOCK.low_started() {}
    while !nrf52::clock::CLOCK.high_started() {}

    let dynamic_deferred_call_clients =
        static_init!([DynamicDeferredCallClientState; 1], Default::default());
    let dynamic_deferred_call = static_init!(
        DynamicDeferredCall,
        DynamicDeferredCall::new(dynamic_deferred_call_clients)
    );
    DynamicDeferredCall::set_global_instance(dynamic_deferred_call);

    let platform = Platform {
        button: button,
        ble_radio: ble_radio,
        ieee802154_radio: ieee802154_radio,
        console: console,
        led: led,
        gpio: gpio,
        rng: rng,
        temp: temp,
        alarm: alarm,
        nonvolatile_storage: nonvolatile_storage,
        ipc: kernel::ipc::IPC::new(board_kernel, &memory_allocation_capability),
    };

    let chip = static_init!(nrf52::chip::NRF52, nrf52::chip::NRF52::new(gpio_port));

    debug!("Initialization complete. Entering main loop\r");
    debug!("{}", &nrf52::ficr::FICR_INSTANCE);

    extern "C" {
        /// Beginning of the ROM region containing app images.
        static _sapps: u8;
    }
    kernel::procs::load_processes(
        board_kernel,
        chip,
        &_sapps as *const u8,
        app_memory,
        process_pointers,
        app_fault_response,
        &process_management_capability,
    );

    board_kernel.kernel_loop(&platform, chip, Some(&platform.ipc), &main_loop_capability);
}

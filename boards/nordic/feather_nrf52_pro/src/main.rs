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
use kernel::{debug, debug_gpio, debug_verbose, static_init};

use nrf52832::gpio::Pin;
use nrf52dk_base::{SpiPins, UartPins};

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

// FIXME: Ideally this should be replaced with Rust's builtin tests by conditional compilation
//
// Also read the instructions in `tests` how to run the tests
#[allow(dead_code)]
mod tests;

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

    nrf52dk_base::setup_board(
        board_kernel,
        BUTTON_RST_PIN,
        &nrf52832::gpio::PORT,
        gpio_pins,
        LED1_PIN,
        LED2_PIN,
        DEBUG_PIN,
        led_pins,
        &UartPins::new(UART_RTS, UART_TXD, UART_CTS, UART_RXD),
        &SpiPins::new(SPI_MOSI, SPI_MISO, SPI_CLK),
        &None,
        button_pins,
        false,
        &mut APP_MEMORY,
        &mut PROCESSES,
        FAULT_RESPONSE,
        nrf52832::uicr::Regulator0Output::DEFAULT,
        false,
    );
}

IoT Wind River Rocket
==

> Help your Internet of Things (IoT) projects take off with Wind River® Rocket™, a free embedded operating system specifically designed to quickly and easily build small, intelligent devices.

> Projects with Rocket are dramatically simplified by our new cloud-based development environment, Wind River Helix™ App Cloud. Find out more at the Rocket Developer Zone.

> Support for other IA platforms such as Edison, Curie, etc.as well as support for ARM support is planned

> An Embedded OS Designed for Rapid IoT Development
> > Rocket provides a robust set of capabilities, giving development teams a best-in-class, scalable real-time operating system for 32-bit microcontroller units (MCUs)—ideal for building sensors, wearables, industrial controllers, and other resource-constrained smart, connected devices.

- [Wind River® Rocket™ Homepage](http://windriver.com/products/operating-systems/rocket/)
- [Wind River® Rocket™ Intel](https://software.intel.com/en-us/iot/rocket)
- [Wind River® Rocket™ Platforms Supported](https://software.intel.com/sites/default/files/managed/8d/f0/PLATFORM_SUPPORT.pdf)
- [Wind River® Rocket™ Github](https://github.com/wind-river-rocket)
- [Wind River® Rocket™ Application Development Primer](https://software.intel.com/sites/default/files/managed/8a/6a/APPLICATION_DEV_PRIMER.pdf)
- [Wind River® Rocket™ Developer Zone](https://communities.intel.com/community/tech/rocket/)
- [Wind River® Rocket™ Getting Started Guides, Documents, Samples and More!](https://software.intel.com/en-us/iot/documentation?field_os_tid[]=81488&value=81488)

## Documentation Features

- Getting Started
- Embedded OS
- Developing application
- Hardware
- Developer Zone
- Customers

## Complete Log outdir/zephyr.elf 

```sh
    make
    Using /users/xe1gyq/MyHelloWorld/zephyr/arch/x86/configs/basic_minuteia_defconfig as base
    Merging /users/xe1gyq/MyHelloWorld/zephyr/kernel/configs/micro.config
    Merging prj_x86.conf
    *
    * Restart config...
    *
    *
    * Zephyr Kernel/x86 Configuration
    *
    Kernel Type
      1. Nano Kernel (NANOKERNEL) (NEW)
    > 2. Micro Kernel (MICROKERNEL)
    choice[1-2]: *
    * General Kernel Options
    *
    System tick frequency (in ticks/second) (SYS_CLOCK_TICKS_PER_SEC) [100] (NEW) 
    System clock's h/w timer frequency (SYS_CLOCK_HW_CYCLES_PER_SEC) [25000000] 25000000
    Initialize stack areas (INIT_STACKS) [N/y/?] (NEW) 
    Execute in place (XIP) [N/y/?] (NEW) 
    Enable ring buffers (RING_BUFFER) [N/y/?] (NEW) 
    Enable event logger (EVENT_LOGGER) [N/y/?] (NEW) 
    Enable profiler features (KERNEL_PROFILER) [N/y/?] (NEW) 
    *
    * Security Options
    *
    Customized security (CUSTOM_SECURITY) [N/y/?] (NEW) 
    Compiler stack canaries (STACK_CANARIES) [N/y/?] (NEW) 
    *
    * Nanokernel Options
    *
    Boot banner (BOOT_BANNER) [N/y/?] (NEW) 
    Interrupt latency metrics [EXPERIMENTAL] (INT_LATENCY_BENCHMARK) [N/y/?] (NEW) 
    Background task stack size (in bytes) (MAIN_STACK_SIZE) [1024] (NEW) 
    ISR and initialization stack size (in bytes) (ISR_STACK_SIZE) [2048] (NEW) 
    Task and fiber custom data (THREAD_CUSTOM_DATA) [N/y/?] (NEW) 
    Enable timeouts on nanokernel objects (NANO_TIMEOUTS) [N/y/?] (NEW) 
    Enable nanokernel timers (NANO_TIMERS) [N/y/?] (NEW) 
    *
    * Microkernel Options
    *
    Microkernel server fiber (_k_server) stack size (MICROKERNEL_SERVER_STACK_SIZE) [1024] (NEW) 
    Priority of the kernel service fiber (MICROKERNEL_SERVER_PRIORITY) [0] (NEW) 
    Maximum priority for priority inheritance algorithm (PRIORITY_CEILING) [0] (NEW) 
    Microkernel server command stack size (in packets) (COMMAND_STACK_SIZE) [64] (NEW) 
    Number of command packets (NUM_COMMAND_PACKETS) [16] (NEW) 
    Number of timer packets (NUM_TIMER_PACKETS) [10] (NEW) 
    Number of task priorities (NUM_TASK_PRIORITIES) [16] (NEW) 
    Workload monitoring [EXPERIMENTAL] (WORKLOAD_MONITOR) [N/y/?] (NEW) 
    Number of task IRQ objects (MAX_NUM_TASK_IRQS) [0] (NEW) 
    *
    * Timer API Options
    *
    Task time slicing (TIMESLICING) [Y/n/?] (NEW) 
      Time slice size (in ticks) (TIMESLICE_SIZE) [0] (NEW) 
      Time slicing task priority threshold (TIMESLICE_PRIORITY) [0] (NEW) 
    Task monitoring [EXPERIMENTAL] (TASK_MONITOR) [N/y/?] (NEW) 
    Kernel object monitoring [EXPERIMENTAL] (OBJECT_MONITOR) [N/y/?] (NEW) 
    Task and fiber monitoring [EXPERIMENTAL] (THREAD_MONITOR) [N/y/?] (NEW) 
    Advanced power management (ADVANCED_POWER_MANAGEMENT) [N/y/?] (NEW) 
    *
    * X86 Platform Configuration options
    *
    Amount of RAM given to the kernel (in kb) (RAM_SIZE) [192] (NEW) 
    Platform Selection
      1. Galileo Gen2 (PLATFORM_GALILEO) (NEW)
      2. IA32 with PCI (PLATFORM_IA32_PCI) (NEW)
    > 3. IA32 (PLATFORM_IA32)
    choice[1-3]: Intel Processor
      1. Atom (CPU_ATOM) (NEW)
    > 2. Minute IA (CPU_MINUTEIA)
    choice[1-2]: Enable cache flushing mechanism (CACHE_FLUSHING) [Y/?] (NEW) y
    *
    * Platform Capabilities
    *
    Advanced Idle Supported (ADVANCED_IDLE_SUPPORTED) [N/y/?] (NEW) 
    EOI Handler Supported (EOI_HANDLER_SUPPORTED) [Y/?] (NEW) y
    Unaligned Write Unsupported (UNALIGNED_WRITE_UNSUPPORTED) [N/y/?] (NEW) 
    Lock Instruction Unsupported (LOCK_INSTRUCTION_UNSUPPORTED) [N/y/?] (NEW) 
    Number of dynamic int stubs (NUM_DYNAMIC_STUBS) [0] (NEW) 
    Disable PIC (PIC_DISABLE) [Y/n/?] y
    *
    * Processor Capabilities
    *
    Support IA32 legacy IO ports (IA32_LEGACY_IO_PORTS) [Y/n/?] y
    Detect cache line size at runtime (CACHE_LINE_SIZE_DETECT) [Y/n/?] (NEW) 
    Detect support of CLFLUSH instruction at runtime (CLFLUSH_DETECT) [Y/n/?] (NEW) 
    *
    * Floating Point Options
    *
    Floating point instructions (FLOAT) [N/y/?] (NEW) 
    *
    * x86 Core Options
    *
    No Asynchronous Interrupts (NO_ISRS) [N/y/?] (NEW) 
    Enable nested interrupts (NO_NESTED_INTERRUPTS) [N/y/?] (NEW) 
    *
    * Memory Layout Options
    *
    Number of IDT vectors (IDT_NUM_VECTORS) [256] (NEW) 
    Number of spare GDT entries (NUM_GDT_SPARE_ENTRIES) [0] (NEW) 
    Physical load address (PHYS_LOAD_ADDR) [0x00100000] (NEW) 
    Reboot implementation
    > 1. Use the RST_CNT register (REBOOT_RST_CNT) (NEW)
    choice[1]: 1
    *
    * Device Drivers
    *
    Simple UART driver (UART_SIMPLE) [N/y/?] (NEW) 
    *
    * Console drivers
    *
    Console drivers (CONSOLE) [Y/n] y
      Enable console input handler (CONSOLE_HANDLER) [Y/?] (NEW) y
      Use UART for console (UART_CONSOLE) [Y/n/?] y
        UART Console Index (UART_CONSOLE_INDEX) [0] (NEW) 
        UART Console Baud Rate (UART_CONSOLE_BAUDRATE) [115200] (NEW) 
      Use RAM console (RAM_CONSOLE) [N/y/?] (NEW) 
      Inter-processor Interrupt console sender (IPI_CONSOLE_SENDER) [N/y/?] (NEW) 
      Inter-processor interrupt console receiver (IPI_CONSOLE_RECEIVER) [N/y/?] (NEW) 
    *
    * Serial Drivers
    *
    Serial Drivers (SERIAL) [Y/n/?] y
      *
      * Serial Port Options
      *
      Serial interrupt level (SERIAL_INTERRUPT_LEVEL) [Y/n/?] (NEW) 
      Serial interrupt low (SERIAL_INTERRUPT_LOW) [N/y/?] (NEW) 
      Interrupt driven UART support (UART_INTERRUPT_DRIVEN) [Y/?] (NEW) y
      NS16550 serial driver (NS16550) [Y/n/?] y
      K20 serial driver (K20_UART) [N/y/?] (NEW) 
      Stellaris serial driver (STELLARIS_UART) [N/y/?] (NEW) 
    *
    * Interrupt Controllers
    *
    LOAPIC (LOAPIC) [Y/?] y
      LOAPIC Debug (LOAPIC_DEBUG) [N/y/?] (NEW) 
      Local APIC Base Address (LOAPIC_BASE_ADDRESS) [0xFEE00000] (NEW) 
      IO-APIC (IOAPIC) [Y/?] (NEW) y
        IO-APIC Debugging (IOAPIC_DEBUG) [N/y/?] (NEW) 
        IO-APIC Base Address (IOAPIC_BASE_ADDRESS) [0xFEC00000] (NEW) 
        Number of Redirection Table Entries available (IOAPIC_NUM_RTES) [24] (NEW) 
    *
    * Timer Drivers
    *
    HPET timer (HPET_TIMER) [Y/n/?] y
      HPET timer legacy emulation mode (HPET_TIMER_LEGACY_EMULATION) [Y/n/?] y
      Enable HPET debug output (HPET_TIMER_DEBUG) [N/y/?] (NEW) 
      HPET Base Address (HPET_TIMER_BASE_ADDRESS) [0xFED00000] (NEW) 
      HPET Timer IRQ (HPET_TIMER_IRQ) [2] 2
      HPET Timer IRQ Priority (HPET_TIMER_IRQ_PRIORITY) [4] (NEW) 
      HPET Interrupt Trigger Condition
      > 1. Falling Edge (HPET_TIMER_FALLING_EDGE) (NEW)
        2. Rising Edge (HPET_TIMER_RISING_EDGE)
        3. Level High (HPET_TIMER_LEVEL_HIGH) (NEW)
        4. Level Low (HPET_TIMER_LEVEL_LOW)
      choice[1-4]: LOAPIC timer (LOAPIC_TIMER) [N/y/?] (NEW) 
    API to disable system clock (SYSTEM_CLOCK_DISABLE) [Y/?] (NEW) y
    *
    * Random Generation Configuration
    *
    Custom random generator (RANDOM_GENERATOR) [N/y/?] (NEW) 
      Non-random number generator (TEST_RANDOM_GENERATOR) [N/y/?] (NEW) 
    *
    * PCI Settings
    *
    Enable PCI (PCI) [N/y/?] (NEW) 
    *
    * GPIO Drivers
    *
    GPIO Drivers (GPIO) [N/y/?] (NEW) 
    Shared interrupt driver (SHARED_IRQ) [N/y/?] (NEW) 
    Shared interrupt instance 1 (SHARED_IRQ_1) [N/y/?] (NEW) 
    *
    * SPI hardware bus support
    *
    SPI hardware bus support (SPI) [N/y/?] (NEW) 
    *
    * I2C Drivers
    *
    I2C Drivers (I2C) [N/y/?] (NEW) 
    *
    * PWM (Pulse Width Modulation) Drivers
    *
    PWM (Pulse Width Modulation) Drivers (PWM) [N/y/?]     (NEW) 
    *
    * Enable platform pinmux driver
    *
    Enable platform pinmux driver (PINMUX) [N/y] (NEW) 
    *
    * ADC drivers
    *
    ADC drivers (ADC) [N/y/?] (NEW) 
    *
    * Cryptography
    *
    *
    * Compile and Link Features
    *
    The kernel binary name (KERNEL_BIN_NAME) [zephyr] (NEW) 
    Custom linker scripts provided (HAVE_CUSTOM_LINKER_SCRIPT) [N/y/?] (NEW) 
    Cross-compiler tool prefix (CROSS_COMPILE) [] (NEW) 
    Task-aware debugging with GDB (GDB_INFO) [N/y/?] (NEW) 
    Allow linking with --whole-archive (LINK_WHOLE_ARCHIVE) [N/y/?] (NEW) 
    Custom compiler options (COMPILER_OPT) [-O0 -g] -O0 -g
    Cross-compiler variant name (TOOLCHAIN_VARIANT) [] (NEW) 
    C Library
    > 1. Build minimal c library (MINIMAL_LIBC) (NEW)
      2. Build with newlib c library (NEWLIB_LIBC) (NEW)
    choice[1-2]: Build additional libc functions [EXPERIMENTAL] (MINIMAL_LIBC_EXTENDED) [N/y/?] (NEW) 
    *
    * Debugging Options
    *
    Build kernel with debugging enabled (DEBUG) [N/y/?] (NEW) 
    Send printk() to console (PRINTK) [Y/n/?] (NEW) 
    Send stdout to console (STDOUT_CONSOLE) [Y/n/?] y
    Send stdout at the earliest stage possible (EARLY_CONSOLE) [N/y/?] (NEW) 
    Enable __ASSERT() macro (ASSERT) [N/y/?] (NEW) 
    Enable system debugging information (DEBUG_INFO) [Y/?] (NEW) y
    Enable GDB Server [EXPERIMENTAL] (GDB_SERVER) [Y/n/?] y
      Maximum number of GDB Server Software breakpoints (GDB_SERVER_MAX_SW_BP) [100] (NEW) 
      Enable GDB interrupt mode (GDB_SERVER_INTERRUPT_DRIVEN) [Y/n/?] (NEW) 
      Enable the bootloader mode (GDB_SERVER_BOOTLOADER) [N/y/?] (NEW) 
    *
    * Safe memory access
    *
    Enable safe memory access (MEM_SAFE) [Y/?] (NEW) y
      Safe memory access implementation
      > 1. Software validation of memory access within memory regions (MEM_SAFE_CHECK_BOUNDARIES) (NEW)
      choice[1]: 1
    Number of safe memory access regions that can be added at runtime (MEM_SAFE_NUM_EXTRA_REGIONS) [0] (NEW) 
    Boot using Linux kexec() system call (BOOTLOADER_KEXEC) [N/y/?] (NEW) 
    Generic boot loader support (BOOTLOADER_UNKNOWN) [Y/?] (NEW) y
    *
    * System Monitoring Options
    *
    Enable performance metrics [EXPERIMENTAL] (PERFORMANCE_METRICS) [N/y/?] (NEW) 
    Reboot functionalities (REBOOT) [Y/?] (NEW) y
    #
    # configuration written to .config
    #
    make[1]: Entering directory '/users/xe1gyq/MyHelloWorld/zephyr'
    make[2]: Entering directory '/users/xe1gyq/MyHelloWorld/outdir'
      GEN     ./Makefile
    scripts/kconfig/conf --silentoldconfig Kconfig
      Using /users/xe1gyq/MyHelloWorld/zephyr as source for kernel
      GEN     ./Makefile
      CHK     include/generated/version.h
      UPD     include/generated/version.h
      HOSTCC  scripts/gen_idt/gen_idt.o
      HOSTLD  scripts/gen_idt/gen_idt
      CHK     misc/generated/configs.c
      UPD     misc/generated/configs.c
      CHK     include/generated/offsets.h
      UPD     include/generated/offsets.h
      CHK     misc/generated/sysgen/prj.mdef
      UPD     misc/generated/sysgen/prj.mdef
      LD      lib/libc/minimal/source/stdlib/built-in.o
      CC      lib/libc/minimal/source/stdout/fprintf.o
      CC      lib/libc/minimal/source/stdout/prf.o
      CC      lib/libc/minimal/source/stdout/sprintf.o
      CC      lib/libc/minimal/source/stdout/stdout_console.o
      LD      lib/libc/minimal/source/stdout/built-in.o
      CC      lib/libc/minimal/source/string/string.o
      LD      lib/libc/minimal/source/string/built-in.o
      LD      lib/libc/minimal/source/built-in.o
      LD      lib/libc/minimal/built-in.o
      LD      lib/libc/built-in.o
      LD      lib/built-in.o
      CC      arch/x86/core/gdt.o
      CC      arch/x86/core/thread.o
      CC      arch/x86/core/fatal.o
      AS      arch/x86/core/cpuhalt.o
      AS      arch/x86/core/excstub.o
      AS      arch/x86/core/swap.o
      AS      arch/x86/core/intboiexit.o
      AS      arch/x86/core/msr.o
      CC      arch/x86/core/excconnect.o
      CC      arch/x86/core/inthndlset.o
      CC      arch/x86/core/sys_fatal_error_handler.o
      AS      arch/x86/core/crt0.o
      AS      arch/x86/core/driver_static_irq_stubs.o
      CC      arch/x86/core/atomic_nolock.o
      AS      arch/x86/core/atomic.o
      AS      arch/x86/core/cache_s.o
      CC      arch/x86/core/cache.o
      CC      arch/x86/core/intconnect.o
      AS      arch/x86/core/intstub.o
      CC      arch/x86/core/strtask.o
      CC      arch/x86/core/x86_reboot.o
      CC      arch/x86/core/debug/debug_frames.o
      LD      arch/x86/core/debug/built-in.o
      LD      arch/x86/core/built-in.o
      CC      arch/x86/debug/gdb_arch.o
      LD      arch/x86/debug/built-in.o
      CC      arch/x86/platforms/ia32/ia32_config.o
      CC      arch/x86/platforms/ia32/ia32.o
      LD      arch/x86/platforms/ia32/built-in.o
      LD      arch/x86/built-in.o
      LD      arch/built-in.o
      CC      kernel/microkernel/k_task.o
      CC      kernel/microkernel/k_idle.o
      CC      kernel/microkernel/k_init.o
      CC      kernel/microkernel/k_command_packet.o
      CC      kernel/microkernel/k_move_data.o
      CC      kernel/microkernel/k_ticker.o
      CC      kernel/microkernel/k_memory_map.o
      CC      kernel/microkernel/k_memory_pool.o
      CC      kernel/microkernel/k_irq.o
      CC      kernel/microkernel/k_nop.o
      CC      kernel/microkernel/k_offload.o
      CC      kernel/microkernel/k_event.o
      CC      kernel/microkernel/k_mailbox.o
      CC      kernel/microkernel/k_mutex.o
      CC      kernel/microkernel/k_fifo.o
      CC      kernel/microkernel/k_semaphore.o
      CC      kernel/microkernel/k_timer.o
      CC      kernel/microkernel/k_pipe_buffer.o
      CC      kernel/microkernel/k_pipe.o
      CC      kernel/microkernel/k_pipe_get.o
      CC      kernel/microkernel/k_pipe_put.o
      CC      kernel/microkernel/k_pipe_util.o
      CC      kernel/microkernel/k_pipe_xfer.o
      CC      misc/printk.o
      CC      misc/mem_safe_check_boundaries.o
      CC      misc/reboot.o
      CC      misc/generated/configs.o
      CC      misc/generated/sysgen/kernel_main.o
      LD      misc/generated/sysgen/built-in.o
      LD      misc/generated/built-in.o
      LD      misc/built-in.o
      CC      debug/gdb_server.o
      LD      debug/built-in.o
      CC      ../src/../src/main.o
      LD      ../src/built-in.o
      CC      drivers/console/uart_console.o
      LD      drivers/console/built-in.o
      CC      drivers/interrupt_controller/i8259.o
      CC      drivers/interrupt_controller/system_apic.o
      CC      drivers/interrupt_controller/loapic_intr.o
      CC      drivers/interrupt_controller/ioapic_intr.o
      LD      drivers/interrupt_controller/built-in.o
      LD      drivers/random/built-in.o
      CC      drivers/serial/serial.o
      CC      drivers/serial/ns16550.o
      LD      drivers/serial/built-in.o
      CC      drivers/timer/hpet.o
      CC      drivers/timer/sys_clock_init.o
      LD      drivers/timer/built-in.o
      LD      drivers/built-in.o
      LINK    zephyr
      LD      zephyr.elf
      GEN     .version
      SIDT    zephyr.elf
      BIN     zephyr.bin
      SYSMAP  System.map
    make[2]: Leaving directory '/users/xe1gyq/MyHelloWorld/outdir'
    make[1]: Leaving directory '/users/xe1gyq/MyHelloWorld/zephyr'
    Loading /users/xe1gyq/MyHelloWorld//outdir/zephyr.elf on target...  done
    Starting application...  done
    Application is now running

## Partial Log outdir/zephyr.elf 

    make
    * Zephyr Kernel/x86 Configuration
    * General Kernel Options
    * Security Options
    * Nanokernel Options
    * Microkernel Options
    * Timer API Options
    * X86 Platform Configuration options
    * Platform Capabilities
    * Processor Capabilities
    * Floating Point Options
    * x86 Core Options
    * Memory Layout Options
    * Device Drivers
    * Console drivers
    * Serial Drivers
    * Interrupt Controllers
    * Timer Drivers
    * Random Generation Configuration
    * PCI Settings
    * GPIO Drivers
    * SPI hardware bus support
    * I2C Drivers
    * PWM (Pulse Width Modulation) Drivers
    * Enable platform pinmux driver
    * ADC drivers
    * Cryptography
    * Compile and Link Features
    * Debugging Options
    * Safe memory access
    * System Monitoring Options
    # configuration written to .config
    Using /users/xe1gyq/MyHelloWorld/zephyr as source for kernel
      SYSMAP  System.map
    make[2]: Leaving directory '/users/xe1gyq/MyHelloWorld/outdir'
    make[1]: Leaving directory '/users/xe1gyq/MyHelloWorld/zephyr'
    Loading /users/xe1gyq/MyHelloWorld//outdir/zephyr.elf on target...  done
    Starting application...  done
    Application is now running
```
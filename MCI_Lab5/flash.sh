#!/usr/bin/env bash
###############################################################################
# Universal STM32CubeMX CMake Build & Flash Script  v2.0
#
# Usage:  ./flash.sh [options] [project_dir]
#
# Options:
#   --clean          Force clean rebuild (default: incremental)
#   --build-only     Build but do not flash
#   --flash-only     Flash last build without rebuilding
#   --debug          Build with Debug configuration (default: Release)
#   --release        Build with Release configuration (explicit)
#   --verify         Read back flash after programming to verify
#   --openocd        Use OpenOCD instead of st-flash
#   --programmer     Use STM32CubeProgrammer CLI instead of st-flash
#   --serial         Flash over UART (uses stm32flash)
#   --help           Show this help
#
# Auto-detects: project name, MCU, CPU/FPU flags, linker script, flash size,
#               flash start address, programmer tool.
# Enables: floating-point printf & scanf, hardware FPU.
# Works with any STM32CubeMX project containing a .ioc + CMakeLists.txt.
###############################################################################
set -euo pipefail

# ── Colors & logging ────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
step()  { echo -e "${CYAN}[STEP]${NC}  ${BOLD}$*${NC}"; }
die()   { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }

# ── Timing helper ───────────────────────────────────────────────────────────
timer_start() { SECONDS=0; }
timer_elapsed() { echo "${SECONDS}s"; }

# ── Defaults ────────────────────────────────────────────────────────────────
DO_CLEAN=false
BUILD_ONLY=false
FLASH_ONLY=false
DO_VERIFY=false
BUILD_TYPE="Release"
FLASH_TOOL="auto"   # auto | st-flash | openocd | programmer | serial
PROJECT_DIR=""

# ── Parse arguments ─────────────────────────────────────────────────────────
show_help() {
    sed -n '2,/^###*$/p' "$0" | grep '^#' | sed 's/^# \?//'
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --clean)       DO_CLEAN=true ;;
        --build-only)  BUILD_ONLY=true ;;
        --flash-only)  FLASH_ONLY=true ;;
        --debug)       BUILD_TYPE="Debug" ;;
        --release)     BUILD_TYPE="Release" ;;
        --verify)      DO_VERIFY=true ;;
        --openocd)     FLASH_TOOL="openocd" ;;
        --programmer)  FLASH_TOOL="programmer" ;;
        --serial)      FLASH_TOOL="serial" ;;
        --help|-h)     show_help ;;
        -*)            die "Unknown option: $1  (try --help)" ;;
        *)             PROJECT_DIR="$1" ;;
    esac
    shift
done

# ── Resolve project directory ────────────────────────────────────────────────
PROJECT_DIR="${PROJECT_DIR:-.}"
PROJECT_DIR="$(cd "$PROJECT_DIR" && pwd)"
cd "$PROJECT_DIR"

echo ""
echo -e "${BOLD}══════════════════════════════════════════════════════════════${NC}"
echo -e "${BOLD}  STM32 Build & Flash  │  $(date '+%Y-%m-%d %H:%M:%S')${NC}"
echo -e "${BOLD}══════════════════════════════════════════════════════════════${NC}"
echo ""
info "Project directory: $PROJECT_DIR"

# ── Verify required tools ───────────────────────────────────────────────────
step "Checking toolchain..."
MISSING_TOOLS=()
for tool in arm-none-eabi-gcc arm-none-eabi-g++ arm-none-eabi-objcopy arm-none-eabi-size cmake; do
    command -v "$tool" &>/dev/null || MISSING_TOOLS+=("$tool")
done
if (( ${#MISSING_TOOLS[@]} > 0 )); then
    die "Missing tools: ${MISSING_TOOLS[*]}"
fi
GCC_VERSION=$(arm-none-eabi-gcc --version | head -1)
info "Compiler: $GCC_VERSION"

# Auto-detect flash tool if set to auto
if [[ "$FLASH_TOOL" == "auto" ]]; then
    if command -v st-flash &>/dev/null; then
        FLASH_TOOL="st-flash"
    elif command -v STM32_Programmer_CLI &>/dev/null; then
        FLASH_TOOL="programmer"
    elif command -v openocd &>/dev/null; then
        FLASH_TOOL="openocd"
    elif command -v stm32flash &>/dev/null; then
        FLASH_TOOL="serial"
    else
        if [[ "$BUILD_ONLY" == false && "$FLASH_ONLY" == false ]]; then
            warn "No flash tool found (st-flash, STM32_Programmer_CLI, openocd, stm32flash)"
            warn "Will build only. Install one of the above to flash."
            BUILD_ONLY=true
        fi
    fi
fi
if [[ "$BUILD_ONLY" == false ]]; then
    info "Flash tool: $FLASH_TOOL"
fi

# ── Find the .ioc file ──────────────────────────────────────────────────────
step "Detecting project..."
IOC_FILES=()
while IFS= read -r -d '' f; do
    IOC_FILES+=("$f")
done < <(find "$PROJECT_DIR" -maxdepth 1 -name '*.ioc' -type f -print0)
if (( ${#IOC_FILES[@]} == 0 )); then
    die "No .ioc file found in $PROJECT_DIR"
elif (( ${#IOC_FILES[@]} > 1 )); then
    warn "Multiple .ioc files found, using first: $(basename "${IOC_FILES[0]}")"
fi
IOC_FILE="${IOC_FILES[0]}"
info "IOC file: $(basename "$IOC_FILE")"

# ── Extract project name from CMakeLists.txt or .ioc filename ────────────────
PROJECT_NAME=""
if [[ -f CMakeLists.txt ]]; then
    PROJECT_NAME=$(grep -oP 'set\s*\(\s*CMAKE_PROJECT_NAME\s+\K[^\s)]+' CMakeLists.txt 2>/dev/null || true)
fi
if [[ -z "$PROJECT_NAME" ]]; then
    PROJECT_NAME="$(basename "$IOC_FILE" .ioc)"
fi
info "Project name: $PROJECT_NAME"

# ── Extract MCU info from .ioc ──────────────────────────────────────────────
MCU_FAMILY=$(grep -oP '^Mcu\.Family=\K.*' "$IOC_FILE" | tr '[:upper:]' '[:lower:]' || true)
MCU_CPN=$(grep -oP '^Mcu\.CPN=\K.*' "$IOC_FILE" | tr '[:upper:]' '[:lower:]' || true)
MCU_PACKAGE=$(grep -oP '^Mcu\.Package=\K.*' "$IOC_FILE" || true)

[[ -n "$MCU_CPN" ]] || die "Cannot determine MCU part number from .ioc"
info "MCU: ${MCU_CPN^^}  (family: ${MCU_FAMILY^^}, package: $MCU_PACKAGE)"

# ── Determine CPU core, FPU flags, and flash size ───────────────────────────
# STM32 naming: STM32<Family><Sub><Flash>[package]
# Flash size codes: 4=16K 6=32K 8=64K B=128K C=256K D=384K E=512K F=768K G=1M H=1.5M I=2M
CPU_FLAGS=""
OPENOCD_TARGET=""
case "$MCU_CPN" in
    stm32f0*|stm32l0*|stm32g0*|stm32c0*)
        CPU_FLAGS="-mcpu=cortex-m0plus -mthumb"
        OPENOCD_TARGET="stm32f0x" ;;
    stm32f1*)
        CPU_FLAGS="-mcpu=cortex-m3 -mthumb"
        OPENOCD_TARGET="stm32f1x" ;;
    stm32f2*)
        CPU_FLAGS="-mcpu=cortex-m3 -mthumb"
        OPENOCD_TARGET="stm32f2x" ;;
    stm32l1*)
        CPU_FLAGS="-mcpu=cortex-m3 -mthumb"
        OPENOCD_TARGET="stm32l1" ;;
    stm32f3*)
        CPU_FLAGS="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard"
        OPENOCD_TARGET="stm32f3x" ;;
    stm32f4*)
        CPU_FLAGS="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard"
        OPENOCD_TARGET="stm32f4x" ;;
    stm32l4*)
        CPU_FLAGS="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard"
        OPENOCD_TARGET="stm32l4x" ;;
    stm32g4*)
        CPU_FLAGS="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard"
        OPENOCD_TARGET="stm32g4x" ;;
    stm32wb*)
        CPU_FLAGS="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard"
        OPENOCD_TARGET="stm32wbx" ;;
    stm32wl*)
        CPU_FLAGS="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard"
        OPENOCD_TARGET="stm32wlx" ;;
    stm32f7*)
        CPU_FLAGS="-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard"
        OPENOCD_TARGET="stm32f7x" ;;
    stm32h7*)
        CPU_FLAGS="-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard"
        OPENOCD_TARGET="stm32h7x" ;;
    stm32l5*)
        CPU_FLAGS="-mcpu=cortex-m33 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard"
        OPENOCD_TARGET="stm32l5x" ;;
    stm32u5*)
        CPU_FLAGS="-mcpu=cortex-m33 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard"
        OPENOCD_TARGET="stm32u5x" ;;
    stm32h5*)
        CPU_FLAGS="-mcpu=cortex-m33 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard"
        OPENOCD_TARGET="stm32h5x" ;;
    *)
        die "Unsupported MCU: ${MCU_CPN^^} — add it to the script's case table"
        ;;
esac
info "CPU flags: $CPU_FLAGS"

# ── Get flash size from MCU part number ──────────────────────────────────────
get_flash_size_from_pn() {
    local pn="$1"
    # STM32 part: stm32XYYYZ... where Z is flash-size code at position 10 (0-indexed)
    local code
    code=$(echo "$pn" | sed -n 's/^stm32.\{4\}\(.\).*/\1/p')
    case "$code" in
        4) echo 16384 ;;   6) echo 32768 ;;   8) echo 65536 ;;
        b) echo 131072 ;;  c) echo 262144 ;;  d) echo 393216 ;;
        e) echo 524288 ;;  f) echo 786432 ;;  g) echo 1048576 ;;
        h) echo 1572864 ;; i) echo 2097152 ;; *) echo 0 ;;
    esac
}
FLASH_SIZE=$(get_flash_size_from_pn "$MCU_CPN")
if (( FLASH_SIZE > 0 )); then
    info "Flash size: $((FLASH_SIZE / 1024)) KB (from part number)"
fi

# ── Find linker script ──────────────────────────────────────────────────────
LINKER_SCRIPT=$(find "$PROJECT_DIR" -maxdepth 1 -name '*.ld' -type f | head -1)
if [[ -z "$LINKER_SCRIPT" ]]; then
    LINKER_SCRIPT=$(find "$PROJECT_DIR" -maxdepth 3 -name '*.ld' -type f | head -1)
fi
[[ -n "$LINKER_SCRIPT" ]] || die "No linker script (.ld) found"
info "Linker script: $(basename "$LINKER_SCRIPT")"

# ── Extract flash start address from linker script ──────────────────────────
FLASH_ADDR=$(grep -oP 'FLASH.*?ORIGIN\s*=\s*\K0x[0-9A-Fa-f]+' "$LINKER_SCRIPT" 2>/dev/null | head -1 || true)
FLASH_ADDR="${FLASH_ADDR:-0x8000000}"
info "Flash address: $FLASH_ADDR"

# ── Find toolchain file (optional, CubeMX generates one) ────────────────────
TOOLCHAIN_FILE=""
if [[ -f cmake/gcc-arm-none-eabi.cmake ]]; then
    TOOLCHAIN_FILE="$PROJECT_DIR/cmake/gcc-arm-none-eabi.cmake"
    info "Toolchain file: cmake/gcc-arm-none-eabi.cmake"
fi

# ── Skip to flash if --flash-only ───────────────────────────────────────────
BUILD_DIR="$PROJECT_DIR/build/firmware"

if [[ "$FLASH_ONLY" == true ]]; then
    step "Flash-only mode — skipping build"
    BIN_FILE="$BUILD_DIR/${PROJECT_NAME}.bin"
    ELF_FILE=""
    for candidate in "$BUILD_DIR/${PROJECT_NAME}.elf" "$BUILD_DIR/${PROJECT_NAME}"; do
        [[ -f "$candidate" ]] && ELF_FILE="$candidate" && break
    done
    [[ -f "$BIN_FILE" ]] || die "No .bin found at $BIN_FILE — run without --flash-only first"
    BIN_SIZE=$(stat -c%s "$BIN_FILE" 2>/dev/null || stat -f%z "$BIN_FILE")
    info "Using existing binary: $BIN_FILE ($BIN_SIZE bytes)"
else

# ── Clean or incremental build ──────────────────────────────────────────────
if [[ "$DO_CLEAN" == true ]]; then
    step "Cleaning build directory..."
    rm -rf "$BUILD_DIR"
else
    if [[ -d "$BUILD_DIR" ]]; then
        info "Incremental build (use --clean to force full rebuild)"
    fi
fi

# ── Configure ────────────────────────────────────────────────────────────────
step "Configuring CMake ($BUILD_TYPE)..."
timer_start

CMAKE_ARGS=(
    -S "$PROJECT_DIR"
    -B "$BUILD_DIR"
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
)

if [[ -n "$TOOLCHAIN_FILE" ]]; then
    CMAKE_ARGS+=(-DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN_FILE")
    CMAKE_ARGS+=(
        "-DCMAKE_EXE_LINKER_FLAGS_INIT=${CPU_FLAGS} -T \"${LINKER_SCRIPT}\" --specs=nano.specs -u _printf_float -u _scanf_float -Wl,-Map=${PROJECT_NAME}.map -Wl,--gc-sections -Wl,--print-memory-usage"
    )
else
    CMAKE_ARGS+=(
        -DCMAKE_C_COMPILER=arm-none-eabi-gcc
        -DCMAKE_CXX_COMPILER=arm-none-eabi-g++
        -DCMAKE_ASM_COMPILER=arm-none-eabi-gcc
        -DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY
        "-DCMAKE_C_FLAGS=${CPU_FLAGS} -Wall -fdata-sections -ffunction-sections"
        "-DCMAKE_CXX_FLAGS=${CPU_FLAGS} -Wall -fdata-sections -ffunction-sections -fno-rtti -fno-exceptions"
        "-DCMAKE_ASM_FLAGS=${CPU_FLAGS} -x assembler-with-cpp"
        "-DCMAKE_EXE_LINKER_FLAGS=${CPU_FLAGS} -T ${LINKER_SCRIPT} --specs=nano.specs -u _printf_float -u _scanf_float -Wl,-Map=${PROJECT_NAME}.map -Wl,--gc-sections -Wl,--print-memory-usage"
    )
fi

cmake "${CMAKE_ARGS[@]}" || die "CMake configure failed"
info "Configure time: $(timer_elapsed)"

# ── Build ────────────────────────────────────────────────────────────────────
JOBS=$(nproc)
step "Building ($JOBS parallel jobs)..."
timer_start
cmake --build "$BUILD_DIR" -j"$JOBS" || die "Build failed"
info "Build time: $(timer_elapsed)"

# ── Locate the ELF ──────────────────────────────────────────────────────────
ELF_FILE=""
for candidate in "$BUILD_DIR/${PROJECT_NAME}.elf" "$BUILD_DIR/${PROJECT_NAME}"; do
    [[ -f "$candidate" ]] && ELF_FILE="$candidate" && break
done
[[ -n "$ELF_FILE" ]] || die "Cannot find built ELF binary in $BUILD_DIR"

# ── Generate .bin and .hex ───────────────────────────────────────────────────
BIN_FILE="$BUILD_DIR/${PROJECT_NAME}.bin"
HEX_FILE="$BUILD_DIR/${PROJECT_NAME}.hex"
arm-none-eabi-objcopy -O binary "$ELF_FILE" "$BIN_FILE" || die "objcopy .bin failed"
arm-none-eabi-objcopy -O ihex   "$ELF_FILE" "$HEX_FILE" || die "objcopy .hex failed"

# ── Symlink compile_commands.json to project root (for clangd / IDE) ────────
if [[ -f "$BUILD_DIR/compile_commands.json" ]]; then
    ln -sf "$BUILD_DIR/compile_commands.json" "$PROJECT_DIR/compile_commands.json" 2>/dev/null || true
fi

# ── Sanity checks ───────────────────────────────────────────────────────────
BIN_SIZE=$(stat -c%s "$BIN_FILE" 2>/dev/null || stat -f%z "$BIN_FILE")
if (( BIN_SIZE == 0 )); then
    die "Generated .bin is empty!"
fi
if (( FLASH_SIZE > 0 && BIN_SIZE > FLASH_SIZE )); then
    die "Binary ($BIN_SIZE bytes) exceeds MCU flash ($((FLASH_SIZE / 1024)) KB)!"
elif (( FLASH_SIZE > 0 )); then
    USAGE_PCT=$(( (BIN_SIZE * 100) / FLASH_SIZE ))
    info "Flash usage: $BIN_SIZE / $((FLASH_SIZE / 1024))K bytes (${USAGE_PCT}%)"
else
    info "Binary size: $BIN_SIZE bytes"
fi

# ── Size summary ────────────────────────────────────────────────────────────
echo ""
arm-none-eabi-size "$ELF_FILE"
echo ""
info "Outputs:"
info "  ELF: $ELF_FILE"
info "  BIN: $BIN_FILE"
info "  HEX: $HEX_FILE"

fi   # end of build section (--flash-only jumps here)

# ── Flash ────────────────────────────────────────────────────────────────────
if [[ "$BUILD_ONLY" == true ]]; then
    echo ""
    info "Build-only mode — skipping flash"
    info "Done!"
    exit 0
fi

echo ""
MAX_RETRIES=1

# Pre-flight: check if programmer is accessible
step "Checking programmer connection..."
case "$FLASH_TOOL" in
    st-flash)
        if command -v st-info &>/dev/null; then
            PROBE=$(st-info --probe 2>&1 || true)
            if echo "$PROBE" | grep -qi "no.*found\|couldn't find"; then
                warn "No ST-Link detected. Make sure the board is connected."
                warn "Attempting to flash anyway..."
            else
                info "ST-Link detected"
                echo "$PROBE" | head -5
            fi
        fi
        ;;
esac

step "Flashing via $FLASH_TOOL..."
for attempt in $(seq 1 $MAX_RETRIES); do
    info "Attempt $attempt/$MAX_RETRIES → address $FLASH_ADDR"

    FLASH_OK=false
    case "$FLASH_TOOL" in
        st-flash)
            if st-flash --reset write "$BIN_FILE" "$FLASH_ADDR" 2>&1; then
                FLASH_OK=true
            fi
            ;;
        openocd)
            OCD_CFG=""
            for iface in "interface/stlink.cfg" "interface/stlink-v2.cfg" "interface/stlink-v2-1.cfg"; do
                if [[ -f "/usr/share/openocd/scripts/$iface" ]] || [[ -f "/usr/local/share/openocd/scripts/$iface" ]]; then
                    OCD_CFG="$iface"
                    break
                fi
            done
            [[ -n "$OCD_CFG" ]] || OCD_CFG="interface/stlink.cfg"

            if openocd -f "$OCD_CFG" \
                       -f "target/${OPENOCD_TARGET}.cfg" \
                       -c "program $ELF_FILE verify reset exit" 2>&1; then
                FLASH_OK=true
            fi
            ;;
        programmer)
            if STM32_Programmer_CLI -c port=SWD -d "$BIN_FILE" "$FLASH_ADDR" -v -rst 2>&1; then
                FLASH_OK=true
            fi
            ;;
        serial)
            SERIAL_PORT=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -1 || true)
            [[ -n "$SERIAL_PORT" ]] || die "No serial port found for UART flashing"
            info "Using serial port: $SERIAL_PORT"
            if stm32flash -w "$BIN_FILE" -v -g "$FLASH_ADDR" "$SERIAL_PORT" 2>&1; then
                FLASH_OK=true
            fi
            ;;
    esac

    if [[ "$FLASH_OK" == true ]]; then
        if [[ "$DO_VERIFY" == true && "$FLASH_TOOL" == "st-flash" ]]; then
            step "Verifying flash..."
            VERIFY_FILE="$BUILD_DIR/${PROJECT_NAME}_readback.bin"
            if st-flash read "$VERIFY_FILE" "$FLASH_ADDR" "$BIN_SIZE" 2>&1; then
                if cmp -s "$BIN_FILE" "$VERIFY_FILE"; then
                    info "Verification PASSED"
                else
                    die "Verification FAILED — flash content does not match binary!"
                fi
                rm -f "$VERIFY_FILE"
            else
                warn "Could not read back flash for verification"
            fi
        fi

        echo ""
        echo -e "${GREEN}${BOLD}══════════════════════════════════════════════════════════════${NC}"
        echo -e "${GREEN}${BOLD}  Flash successful! Device has been reset and is running.${NC}"
        echo -e "${GREEN}${BOLD}══════════════════════════════════════════════════════════════${NC}"
        echo ""
        exit 0
    fi

    warn "Flash attempt $attempt failed"
    if (( attempt < MAX_RETRIES )); then
        info "Retrying in 2 seconds..."
        sleep 2
    fi
done

echo ""
die "Flashing failed after $MAX_RETRIES attempts.
  - Check USB cable and ST-Link connection
  - Try: st-info --probe
  - Try unplugging and re-plugging the board
  - Try: sudo st-flash --reset write $BIN_FILE $FLASH_ADDR"
#include "heap_driver.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define HEAP_START_ADDR  ((uint8_t*)0x20001000)
#define HEAP_SIZE        (4 * 1024)
#define BLOCK_SIZE       16
#define BLOCK_COUNT      (HEAP_SIZE / BLOCK_SIZE)

// Students should be provided the above code (includes and defines) and the function declarations in this file.
// They can figure out the rest.

// Allocation bitmap: 0 = free, 1 = used

// Add you code below
/*
 * Heap region: 0x20001000 - 0x20001FFF (4 KB), split into 256 blocks of 16 bytes.
 *
 * block_map[i]   0 = block i is free, 1 = block i is in use
 * block_start[i] 1 = block i is the FIRST block of an allocation
 *
 * The two arrays are ordinary globals (in .bss). Only the user's data is
 * stored inside the 4 KB heap region itself.
 */
static uint8_t block_map[BLOCK_COUNT];
static uint8_t block_start[BLOCK_COUNT];

/* ---------------- Debug trace over UART ---------------- */
#define HEAP_DEBUG  1      // 1 = print a trace of every call, 0 = silent

#if HEAP_DEBUG
#include <stdio.h>
extern void print_uart(const char *msg);   // defined in the provided main.c
#define HEAP_LOG(...)                                  \
    do {                                               \
        char log_buf[96];                              \
        snprintf(log_buf, sizeof(log_buf), __VA_ARGS__); \
        print_uart(log_buf);                           \
    } while (0)
#else
#define HEAP_LOG(...)  do { } while (0)
#endif

extern uint8_t _end;       // linker symbol: end of .data/.bss

/* ---------------- heap_init ---------------- */
void heap_init(void)
{
    memset(block_map, 0, sizeof(block_map));       // every block free
    memset(block_start, 0, sizeof(block_start));   // no allocations yet

    /* Boundary check: the heap must not overlap the program's globals */
    if ((uintptr_t)&_end > (uintptr_t)HEAP_START_ADDR)
    {
        HEAP_LOG("[heap] WARNING: globals end at %p, past heap start %p\r\n",
                 (void *)&_end, (void *)HEAP_START_ADDR);
    }

    HEAP_LOG("[heap] init: %u blocks x %u bytes at %p\r\n",
             (unsigned)BLOCK_COUNT, (unsigned)BLOCK_SIZE, (void *)HEAP_START_ADDR);
}

/* ---------------- heap_alloc ---------------- */
void* heap_alloc(size_t size)
{
    if (size == 0 || size > HEAP_SIZE)
    {
        HEAP_LOG("[heap] alloc %u bytes rejected (size 0 or larger than heap)\r\n",
                 (unsigned)size);
        return NULL;
    }

    /* 1. How many blocks? Round up: 20 bytes -> 2 blocks of 16 */
    size_t needed = (size + BLOCK_SIZE - 1) / BLOCK_SIZE;

    /* 2. Scan for 'needed' free blocks in a row (first fit) */
    size_t run = 0;                                 // free blocks seen in a row
    for (size_t i = 0; i < BLOCK_COUNT; i++)
    {
        if (block_map[i] == 0)
        {
            run++;
            if (run == needed)
            {
                size_t first = i + 1 - needed;      // where the run began

                /* 3. Mark the run as used */
                for (size_t j = first; j <= i; j++)
                {
                    block_map[j] = 1;
                }
                block_start[first] = 1;

                /* 4. Turn the block index into a real SRAM address */
                uint8_t *ptr = HEAP_START_ADDR + first * BLOCK_SIZE;

                HEAP_LOG("[heap] alloc %u bytes -> %u block(s), index %u, addr %p\r\n",
                         (unsigned)size, (unsigned)needed, (unsigned)first, (void *)ptr);
                return ptr;
            }
        }
        else
        {
            run = 0;                                // run broken, count again
        }
    }

    HEAP_LOG("[heap] alloc %u bytes failed: no %u free blocks in a row\r\n",
             (unsigned)size, (unsigned)needed);
    return NULL;
}

/* ---------------- heap_free ---------------- */
void heap_free(void* ptr)
{
    if (ptr == NULL)
    {
        return;                                     // same as free(NULL)
    }

    /* 1. Validate: the pointer must lie inside the heap region */
    uintptr_t addr  = (uintptr_t)ptr;
    uintptr_t start = (uintptr_t)HEAP_START_ADDR;
    if (addr < start || addr >= start + HEAP_SIZE)
    {
        HEAP_LOG("[heap] free %p ignored: not inside the heap\r\n", ptr);
        return;
    }

    /* 2. Align: turn the address back into a block index (round down) */
    size_t index = (addr - start) / BLOCK_SIZE;

    if (block_start[index] == 0)
    {
        HEAP_LOG("[heap] free %p ignored: not the start of an allocation\r\n", ptr);
        return;
    }

    /* 3. Free blocks by scanning forward until a free block is reached,
          or the next allocation begins (so a neighbour is never freed) */
    size_t i = index;
    size_t freed = 0;
    do
    {
        block_map[i] = 0;
        block_start[i] = 0;
        i++;
        freed++;
    } while (i < BLOCK_COUNT && block_map[i] == 1 && block_start[i] == 0);

    HEAP_LOG("[heap] free %p -> %u block(s) released\r\n", ptr, (unsigned)freed);
}

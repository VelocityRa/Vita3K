// Vita3K emulator project
// Copyright (C) 2018 Vita3K team
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

#include <cpu/cpu_dynarmic.h>
#include <cpu/functions.h>
#include "mem/mem.h"
#include "util/log.h"
#include "disasm/functions.h"
#include "mem/ptr.h"

//struct CPUDynarmicState {
//    MemState *mem = nullptr;
//    Address entry_point;
//
//    std::unique_ptr<DynarmicCallbacks> cb;
//    std::unique_ptr<Dynarmic::A32::Jit> jit;
//};

/*
std::uint8_t DynarmicCallbacks::MemoryRead8(Dynarmic::A32::VAddr vaddr) {
    const auto &m = g_mem->memory;
    return m[vaddr];
}
std::uint16_t DynarmicCallbacks::MemoryRead16(Dynarmic::A32::VAddr vaddr) {
    const auto &m = g_mem->memory;
    return m[vaddr] | (uint16_t)m[vaddr + 1] << 8;
}
std::uint32_t DynarmicCallbacks::MemoryRead32(Dynarmic::A32::VAddr vaddr) {
    const auto &m = g_mem->memory;
    return m[vaddr] | (uint32_t)m[vaddr + 1] << 8 | (uint32_t)m[vaddr + 2] << 16 | (uint32_t)m[vaddr + 3] << 24;
}
std::uint64_t DynarmicCallbacks::MemoryRead64(Dynarmic::A32::VAddr vaddr) {
    const auto &m = g_mem->memory;
    return m[vaddr] | (uint64_t)m[vaddr + 1] << 8 | (uint64_t)m[vaddr + 2] << 16 | (uint64_t)m[vaddr + 3] << 24 | (uint64_t)m[vaddr + 4] << 32 | m[vaddr + 5] << 40 | (uint64_t)m[vaddr + 6] << 48 | (uint64_t)m[vaddr + 7] << 56;
}

void DynarmicCallbacks::MemoryWrite8(Dynarmic::A32::VAddr vaddr, std::uint8_t value) {
    const auto &m = g_mem->memory;
    m[vaddr] = value;
}
void DynarmicCallbacks::MemoryWrite16(Dynarmic::A32::VAddr vaddr, std::uint16_t value) {
    const auto &m = g_mem->memory;
    m[vaddr] = value;
    m[vaddr + 1] = value >> 8;
}
void DynarmicCallbacks::MemoryWrite32(Dynarmic::A32::VAddr vaddr, std::uint32_t value) {
    const auto &m = g_mem->memory;
    m[vaddr] = value;
    m[vaddr + 1] = value >> 8;
    m[vaddr + 2] = value >> 16;
    m[vaddr + 3] = value >> 24;
}
void DynarmicCallbacks::MemoryWrite64(Dynarmic::A32::VAddr vaddr, std::uint64_t value) {
    const auto &m = g_mem->memory;
    m[vaddr] = value;
    m[vaddr + 1] = value >> 8;
    m[vaddr + 2] = value >> 16;
    m[vaddr + 3] = value >> 24;
    m[vaddr + 4] = value >> 32;
    m[vaddr + 5] = value >> 40;
    m[vaddr + 6] = value >> 48;
    m[vaddr + 7] = value >> 56;
}
*/

std::uint32_t DynarmicCallbacks::MemoryReadCode(std::uint32_t vaddr) {
    const auto vaddr_un = vaddr;
    vaddr &= 0xFFFFFFF0;
    LOG_DEBUG("un: code: {} vaddr: {}", log_hex(MemoryRead32(vaddr_un)), log_hex(vaddr_un));
    LOG_DEBUG("al: code: {} vaddr: {}", log_hex(MemoryRead32(vaddr)), log_hex(vaddr));
    const uint8_t *const code = Ptr<const uint8_t>(static_cast<Address>(vaddr)).get(*g_mem);
    const uint8_t *const code_un = Ptr<const uint8_t>(static_cast<Address>(vaddr_un)).get(*g_mem);
    const size_t buffer_size = GB(4) - vaddr;
    const size_t buffer_size_un = GB(4) - vaddr_un;
    const bool thumb = g_jit->Cpsr() & 0b10000;
    const std::string disassembly_un = disassemble(disasm, code_un, buffer_size_un, vaddr_un, thumb);
    LOG_TRACE("un: {} {}", log_hex(vaddr), disassembly_un);
    const std::string disassembly = disassemble(disasm, code, buffer_size, vaddr, thumb);
    LOG_TRACE("al: {} {}", log_hex(vaddr), disassembly);
    return MemoryRead32(vaddr);
}

u8 DynarmicCallbacks::MemoryRead8(u32 vaddr) {
    if (vaddr >= GB(4)) {
        return 0;
    }
    return g_mem->memory[vaddr];
}

u16 DynarmicCallbacks::MemoryRead16(u32 vaddr) {
    return u16(MemoryRead8(vaddr)) | u16(MemoryRead8(vaddr + 1)) << 8;
}

u32 DynarmicCallbacks::MemoryRead32(u32 vaddr) {
    return u32(MemoryRead16(vaddr)) | u32(MemoryRead16(vaddr + 2)) << 16;
}

u64 DynarmicCallbacks::MemoryRead64(u32 vaddr) {
    return u64(MemoryRead32(vaddr)) | u64(MemoryRead32(vaddr + 4)) << 32;
}

void DynarmicCallbacks::MemoryWrite8(u32 vaddr, u8 value) {
    if (vaddr >= GB(4)) {
        return;
    }
    g_mem->memory[vaddr] = value;
}

void DynarmicCallbacks::MemoryWrite16(u32 vaddr, u16 value) {
    MemoryWrite8(vaddr, u8(value));
    MemoryWrite8(vaddr + 1, u8(value >> 8));
}

void DynarmicCallbacks::MemoryWrite32(u32 vaddr, u32 value) {
    MemoryWrite16(vaddr, u16(value));
    MemoryWrite16(vaddr + 2, u16(value >> 16));
}

void DynarmicCallbacks::MemoryWrite64(u32 vaddr, u64 value) {
    MemoryWrite32(vaddr, u32(value));
    MemoryWrite32(vaddr + 4, u32(value >> 32));
}


void DynarmicCallbacks::InterpreterFallback(Dynarmic::A32::VAddr pc, size_t num_instructions) {
    LOG_CRITICAL("DynarmicCallbacks::ExceptionRaised: pc: {} num_instr: {}", log_hex(pc), num_instructions);
}

void DynarmicCallbacks::CallSVC(std::uint32_t swi) {
    LOG_CRITICAL("DynarmicCallbacks::CallSVC {}", log_hex(swi));
}

void DynarmicCallbacks::ExceptionRaised(Dynarmic::A32::VAddr pc, Dynarmic::A32::Exception exception) {
    LOG_CRITICAL("DynarmicCallbacks::ExceptionRaised ({}) {}", log_hex(pc), (u32)exception);
}

void DynarmicCallbacks::AddTicks(std::uint64_t ticks) {
    LOG_CRITICAL("DynarmicCallbacks::AddTicks: {}", ticks);
    
    if (ticks > ticks_left) {
        ticks_left = 0;
        return;
    }
    ticks_left -= ticks;
}

std::uint64_t DynarmicCallbacks::GetTicksRemaining() {
    LOG_CRITICAL("DynarmicCallbacks::GetTicksRemaining: ticks_left: {}", ticks_left);
    return ticks_left;
}

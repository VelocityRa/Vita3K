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

#pragma once

#include <dynarmic/A32/a32.h>
#include <dynarmic/A32/config.h>
#include "disasm/state.h"

using u8 = std::uint8_t; ///< 8-bit unsigned byte
using u16 = std::uint16_t; ///< 16-bit unsigned short
using u32 = std::uint32_t; ///< 32-bit unsigned word
using u64 = std::uint64_t; ///< 64-bit unsigned int

using s8 = std::int8_t; ///< 8-bit signed byte
using s16 = std::int16_t; ///< 16-bit signed short
using s32 = std::int32_t; ///< 32-bit signed word
using s64 = std::int64_t; ///< 64-bit signed int

using f32 = float; ///< 32-bit floating point
using f64 = double; ///< 64-bit floating point

// TODO: It would be nice to eventually replace these with strong types that prevent accidental
// conversion between each other.
using VAddr = u64; ///< Represents a pointer in the userspace virtual address space.
using PAddr = u64; ///< Represents a pointer in the ARM11 physical address space.

class DynarmicCallbacks : public Dynarmic::A32::UserCallbacks {
public:
    std::uint32_t MemoryReadCode(std::uint32_t vaddr) override;
    std::uint8_t MemoryRead8(u32 vaddr) override;
    std::uint16_t MemoryRead16(u32 vaddr) override;
    std::uint32_t MemoryRead32(u32 vaddr) override;
    std::uint64_t MemoryRead64(u32 vaddr) override;
    void MemoryWrite8(u32 vaddr, u8 value) override;
    void MemoryWrite16(u32 vaddr, u16 value) override;
    void MemoryWrite32(u32 vaddr, u32 value) override;
    void MemoryWrite64(u32 vaddr, u64 value) override;
    void InterpreterFallback(Dynarmic::A32::VAddr pc, size_t num_instructions) override;
    void CallSVC(std::uint32_t swi) override;
    void ExceptionRaised(Dynarmic::A32::VAddr pc, Dynarmic::A32::Exception exception) override;
    void AddTicks(std::uint64_t ticks) override;
    std::uint64_t GetTicksRemaining() override;

    uint64_t ticks_left = 0;
    uint32_t until_pc = 0;
    DisasmState disasm;
};

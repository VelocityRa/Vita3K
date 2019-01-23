#include <shader/functions.h>

#include <shader/types.h>
#include <util/log.h>

#include <SPIRV/SpvBuilder.h>
#include <boost/optional.hpp>
#include <spdlog/fmt/fmt.h>
#include <spirv.hpp>

#include <iostream>
#include <tuple>

// For debugging
#pragma optimize("", off)

// TODO: Rename file & refactor file structure

using namespace USSE;
using boost::optional;

// TODO: make LOG_RAW
#define LOG_DISASM(fmt_str, ...) std::cout << fmt::format(fmt_str, __VA_ARGS__) << std::endl

//translator
namespace shader {
namespace usse {

namespace Disasm {
//
// Disasm helpers
//

inline const std::string &opcode_str(const Opcode &e) {
    static const std::unordered_map<Opcode, const std::string> names = {
#define OPCODE(n) { Opcode::n, std::string(#n) },
#include "shader/usse_opcodes.inc"
#undef OPCODE
    };
    return names.at(e);
}

static const char *e_predicate_str(ExtPredicate p) {
    switch (p) {
    case ExtPredicate::NONE: return "";
    case ExtPredicate::P0: return "p0 ";
    case ExtPredicate::P1: return "p1 ";
    case ExtPredicate::P2: return "p2 ";
    case ExtPredicate::P3: return "p3 ";
    case ExtPredicate::NEGP0: return "!p0 ";
    case ExtPredicate::NEGP1: return "!p1 ";
    case ExtPredicate::PN: return "pN ";
    default: return "invalid";
    }
}
static const char *s_predicate_str(ShortPredicate p) {
    switch (p) {
    case ShortPredicate::NONE: return "";
    case ShortPredicate::P0: return "p0 ";
    case ShortPredicate::P1: return "p1 ";
    case ShortPredicate::NEGP0: return "!p0 ";
    default: return "invalid";
    }
}

static const char *move_data_type_str(MoveDataType p) {
    switch (p) {
    case MoveDataType::INT8: return "i8";
    case MoveDataType::INT16: return "i16";
    case MoveDataType::INT32: return "i32";
    case MoveDataType::C10: return "c10";
    case MoveDataType::F16: return "f16";
    case MoveDataType::F32: return "f32";
    default: return "invalid";
    }
}

} // namespace Disasm

//
// Decoder helpers
//

/**
 * \brief Changes TEMP registers to FPINTERNAL if certain conditions are met
 */
static void check_reg_internal(Operand &inout_reg, bool is_double_regs) {
    const auto max_reg_num = is_double_regs ? 128 : 64;
    const auto temp_reg_limit = max_reg_num - (is_double_regs ? 8 : 4);

    if (inout_reg.bank == RegisterBank::TEMP && inout_reg.num >= temp_reg_limit) {
        inout_reg.num -= temp_reg_limit;
        if (is_double_regs)
            inout_reg.num >>= 1;
        inout_reg.bank = RegisterBank::FPINTERNAL;
    }
}

/**
 * \brief Doubles 'reg' register if necessary
 */
static void double_reg(Imm6 &reg, RegisterBank reg_bank) {
    if (reg_bank != RegisterBank::SPECIAL && reg_bank != RegisterBank::IMMEDIATE)
        reg <<= 1;
}

static Swizzle4 decode_swizzle4(uint32_t encoded_swizzle) {
    return {
        (SwizzleChannel)((encoded_swizzle & 0b000'000'000'111) >> 0),
        (SwizzleChannel)((encoded_swizzle & 0b000'000'111'000) >> 3),
        (SwizzleChannel)((encoded_swizzle & 0b000'111'000'000) >> 6),
        (SwizzleChannel)((encoded_swizzle & 0b111'000'000'000) >> 9),
    };
}
// Operand bank decoding

static RegisterBank decode_dest_bank(Imm2 dest_bank, bool bank_ext) {
    if (dest_bank == 3)
        return RegisterBank::INDEXED;
    else
        // TODO: Index stuff
        if (bank_ext)
        switch (dest_bank) {
        case 0: return RegisterBank::SECATTR;
        case 1: return RegisterBank::SPECIAL;
        case 2: return RegisterBank::INDEX;
        case 3: return RegisterBank::FPINTERNAL;
        }
    else
        switch (dest_bank) {
        case 0: return RegisterBank::TEMP;
        case 1: return RegisterBank::OUTPUT;
        case 2: return RegisterBank::PRIMATTR;
        }
    LOG_ERROR("Invalid dest_bank");
    return RegisterBank::INVALID;
}

static RegisterBank decode_src0_bank(Imm2 src0_bank, Imm1 bank_ext) {
    if (bank_ext)
        switch (src0_bank) {
        case 0: return RegisterBank::OUTPUT;
        case 1: return RegisterBank::SECATTR;
        }
    else
        switch (src0_bank) {
        case 0: return RegisterBank::TEMP;
        case 1: return RegisterBank::PRIMATTR;
        }
    LOG_ERROR("Invalid src0_bank");
    return RegisterBank::INVALID;
}

static RegisterBank decode_src12_bank(Imm2 src12_bank, Imm1 bank_ext) {
    // TODO: Index stuff
    if (bank_ext)
        switch (src12_bank) {
        case 1: return RegisterBank::SPECIAL;
        case 2: return RegisterBank::IMMEDIATE;
        }
    else
        switch (src12_bank) {
        case 0: return RegisterBank::TEMP;
        case 1: return RegisterBank::OUTPUT;
        case 2: return RegisterBank::PRIMATTR;
        case 3: return RegisterBank::SECATTR;
        }
    LOG_ERROR("Invalid src12_bank");
    return RegisterBank::INVALID;
}

// Register/Operand decoding

static void finalize_register(Operand &reg, bool is_double_regs) {
    check_reg_internal(reg, is_double_regs);
}

static Operand decode_dest(Imm6 dest_n, Imm2 dest_bank, bool bank_ext, bool is_double_regs) {
    Operand dest{};
    dest.num = dest_n;
    dest.bank = decode_dest_bank(dest_bank, bank_ext);

    if (is_double_regs)
        double_reg(dest.num, dest.bank);

    finalize_register(dest, is_double_regs);
    return dest;
}

static Operand decode_src12(Imm6 src_n, Imm2 src_bank_sel, Imm1 src_bank_ext, bool is_double_regs) {
    Operand src{};
    src.num = src_n;
    src.bank = decode_src12_bank(src_bank_sel, src_bank_ext);

    if (is_double_regs)
        double_reg(src.num, src.bank);

    finalize_register(src, is_double_regs);
    return src;
}

//
// Translation
//

// For debugging SPIR-V output
static uint32_t instr_idx = 0;

class USSETranslatorVisitor final {
public:
    using instruction_return_type = bool;

    USSETranslatorVisitor() = delete;
    explicit USSETranslatorVisitor(spv::Builder &_b, const uint64_t &_instr, const SpirvShaderParameters &spirv_params, emu::SceGxmProgramType program_type)
        : m_b(_b)
        , m_instr(_instr)
        , m_spirv_params(spirv_params)
        , m_program_type(program_type) {}

private:
    //
    // Translation helpers
    //
    const SpirvVarRegBank &get_reg_bank(RegisterBank reg_bank) const {
        switch (reg_bank) {
        case RegisterBank::PRIMATTR:
            return m_spirv_params.ins;
        case RegisterBank::SECATTR:
            return m_spirv_params.uniforms;
        case RegisterBank::OUTPUT:
            return m_spirv_params.outs;
        case RegisterBank::TEMP:
            return m_spirv_params.temps;
        case RegisterBank::FPINTERNAL:
            return m_spirv_params.internals;
        }

        LOG_WARN("Reg bank {} unsupported", static_cast<uint8_t>(reg_bank));
        return {};
    }

    spv::Id swizzle_to_spv_comp(spv::Id composite, spv::Id type, SwizzleChannel swizzle) {
        switch (swizzle) {
        case SwizzleChannel::_X:
        case SwizzleChannel::_Y:
        case SwizzleChannel::_Z:
        case SwizzleChannel::_W:
            return m_b.createCompositeExtract(composite, type, static_cast<Imm4>(swizzle));

        // TODO: Implement these with OpCompositeExtract
        case SwizzleChannel::_0: break;
        case SwizzleChannel::_1: break;
        case SwizzleChannel::_2: break;

        case SwizzleChannel::_H: break;
        }

        LOG_WARN("Swizzle channel {} unsupported", static_cast<Imm4>(swizzle));
        return static_cast<spv::Id>(-1u);
    }

public:
    //
    // Instructions
    //
    bool vnmad(ExtPredicate pred, bool skipinv, Imm2 src1_swiz_10_11, bool syncstart, Imm1 dest_bank_ext, Imm1 src1_swiz_9, Imm1 src1_bank_ext, Imm1 src2_bank_ext, Imm4 src2_swiz, bool nosched, DestinationMask dest_mask, Imm2 src1_mod, Imm1 src2_mod, Imm2 src1_swiz_7_8, Imm2 dest_bank_sel, Imm2 src1_bank_sel, Imm2 src2_bank_sel, Imm6 dest_n, Imm7 src1_swiz_0_6, Imm3 op2, Imm6 src1_n, Imm6 src2_n) {
        Instruction inst{};

        static const Opcode tb_decode_vop_f32[] = {
            Opcode::VMUL,
            Opcode::VADD,
            Opcode::VFRC,
            Opcode::VDSX,
            Opcode::VDSY,
            Opcode::VMIN,
            Opcode::VMAX,
            Opcode::VDP,
        };
        static const Opcode tb_decode_vop_f16[] = {
            Opcode::VF16MUL,
            Opcode::VF16ADD,
            Opcode::VF16FRC,
            Opcode::VF16DSX,
            Opcode::VF16DSY,
            Opcode::VF16MIN,
            Opcode::VF16MAX,
            Opcode::VF16DP,
        };
        Opcode opcode;
        const bool is_32_bit = m_instr & (1ull << 59);
        if (is_32_bit)
            opcode = tb_decode_vop_f32[op2];
        else
            opcode = tb_decode_vop_f16[op2];

        LOG_DISASM("{:016x}: {}{}", m_instr, Disasm::e_predicate_str(pred), Disasm::opcode_str(opcode));

        // Decode operands

        inst.opr.dest = decode_dest(dest_n, dest_bank_sel, dest_bank_ext, true);
        inst.opr.src1 = decode_src12(src1_n, src1_bank_sel, src1_bank_ext, true);
        inst.opr.src2 = decode_src12(src2_n, src2_bank_sel, src2_bank_ext, true);

        const auto src1_swizzle_enc = src1_swiz_0_6 | src1_swiz_7_8 << 7 | src1_swiz_9 << 9 | src1_swiz_10_11 << 10;
        inst.opr.src1.swizzle = decode_swizzle4(src1_swizzle_enc);

        static const Swizzle4 tb_decode_src2_swizzle[] = {
            SWIZZLE_CHANNEL_4(X, X, X, X),
            SWIZZLE_CHANNEL_4(Y, Y, Y, Y),
            SWIZZLE_CHANNEL_4(Z, Z, Z, Z),
            SWIZZLE_CHANNEL_4(W, W, W, W),
            SWIZZLE_CHANNEL_4(X, Y, Z, W),
            SWIZZLE_CHANNEL_4(Y, Z, W, W),
            SWIZZLE_CHANNEL_4(X, Y, Z, Z),
            SWIZZLE_CHANNEL_4(X, X, Y, Z),
            SWIZZLE_CHANNEL_4(X, Y, X, Y),
            SWIZZLE_CHANNEL_4(X, Y, W, Z),
            SWIZZLE_CHANNEL_4(Z, X, Y, W),
            SWIZZLE_CHANNEL_4(Z, W, Z, W),
            SWIZZLE_CHANNEL_4(Y, Z, X, Z),
            SWIZZLE_CHANNEL_4(X, X, Y, Y),
            SWIZZLE_CHANNEL_4(X, Z, W, W),
            SWIZZLE_CHANNEL_4(X, Y, Z, 1),
        };

        inst.opr.src2.swizzle = tb_decode_src2_swizzle[src2_swiz];

        // TODO: source modifiers

        // Recompile
        m_b.setLine(usse::instr_idx);

        return true;
    }

    bool vmov(ExtPredicate pred, bool skipinv, bool test_2, Imm1 src2_bank_sel, bool syncstart, Imm1 dest_bank_ext, Imm1 end_or_src0_ext_bank, Imm1 src1_bank_ext, Imm1 src2_bank_ext, MoveType move_type, RepeatCount repeat_count, bool nosched, MoveDataType move_data_type, bool test_1, Imm4 src_swiz, Imm1 src0_bank_sel, Imm2 dest_bank_sel, Imm2 src1_bank_sel, Imm2 src0_comp_sel, DestinationMask dest_mask, Imm6 dest_n, Imm6 src0_n, Imm6 src1_n, Imm6 src2_n) {
        Instruction inst{};

        static const Opcode tb_decode_vmov[] = {
            Opcode::VMOV,
            Opcode::VMOVC,
            Opcode::VMOVCU8,
            Opcode::INVALID,
        };

        inst.opcode = tb_decode_vmov[(Imm3)move_type];

        LOG_DISASM("{:016x}: {}{}.{}", m_instr, Disasm::e_predicate_str(pred), Disasm::opcode_str(inst.opcode), Disasm::move_data_type_str(move_data_type));

        // TODO: dest mask
        // TODO: flags
        // TODO: test type

        const bool is_double_regs = move_data_type == MoveDataType::C10 || move_data_type == MoveDataType::F16 || move_data_type == MoveDataType::F32;
        const bool is_conditional = (move_type != MoveType::UNCONDITIONAL);

        // Decode operands

        inst.opr.dest = decode_dest(dest_n, dest_bank_sel, dest_bank_ext, is_double_regs);
        inst.opr.src1 = decode_src12(src1_n, src1_bank_sel, src1_bank_ext, is_double_regs);

        static const Swizzle4 tb_decode_src1_swizzle[] = {
            SWIZZLE_CHANNEL_4(X, X, X, X),
            SWIZZLE_CHANNEL_4(Y, Y, Y, Y),
            SWIZZLE_CHANNEL_4(Z, Z, Z, Z),
            SWIZZLE_CHANNEL_4(W, W, W, W),
            SWIZZLE_CHANNEL_4(X, Y, Z, W),
            SWIZZLE_CHANNEL_4(Y, Z, W, W),
            SWIZZLE_CHANNEL_4(X, Y, Z, Z),
            SWIZZLE_CHANNEL_4(X, X, Y, Z),
            SWIZZLE_CHANNEL_4(X, Y, X, Y),
            SWIZZLE_CHANNEL_4(X, Y, W, Z),
            SWIZZLE_CHANNEL_4(Z, X, Y, W),
            SWIZZLE_CHANNEL_4(Z, W, Z, W),
            SWIZZLE_CHANNEL_4(Y, Z, X, Z),
            SWIZZLE_CHANNEL_4(X, X, Y, Y),
            SWIZZLE_CHANNEL_4(X, Z, W, W),
            SWIZZLE_CHANNEL_4(X, Y, Z, 1),
        };

        // TODO(?): Do this in decode_*
        inst.opr.src1.swizzle = tb_decode_src1_swizzle[src_swiz];

        // TODO: adjust dest mask if needed
        if (move_data_type != MoveDataType::F32) {
            LOG_WARN("Data type != F32 unsupported");
            return false;
        }

        if (is_conditional) {
            LOG_WARN("Conditional vmov instructions unsupported");
            return false;
        }

        if (inst.opr.dest.bank == RegisterBank::SPECIAL || inst.opr.src0.bank == RegisterBank::SPECIAL || inst.opr.src1.bank == RegisterBank::SPECIAL || inst.opr.src2.bank == RegisterBank::SPECIAL) {
            LOG_WARN("Special regs unsupported");
            return false;
        }

        //        if (is_conditional) {
        //            inst.operands.src0 = decode_src0(src0_n, src0_bank_sel, end_or_src0_ext_bank, is_double_regs);
        //            inst.operands.src2 = decode_src12(src2_n, src2_bank_sel, src2_bank_ext, is_double_regs);
        //        }

        // Recompile
        m_b.setLine(usse::instr_idx);

        auto src_swz = inst.opr.src1.swizzle;

        const SpirvVarRegBank &dest_regs = get_reg_bank(inst.opr.dest.bank);
        const SpirvVarRegBank &src_regs = get_reg_bank(inst.opr.src1.bank);

        // TODO: There can be more than one source regs

        const auto spv_float = m_b.makeFloatType(32);
        const auto spv_vec4 = m_b.makeVectorType(spv_float, 4);

        //        const auto repeat_count_num = repeat_count == RepeatCount::REPEAT_0 ? 1 : (uint8_t)repeat_count;
        //        const auto repeat_offset_jump = 2; // use dest_mask # of bits?

        const auto repeat_count_num = (uint8_t)repeat_count + 1;
        const auto repeat_offset_jump = 2; // use dest_mask # of bits?

        for (auto repeat_offset = 0;
             repeat_offset < repeat_count_num * repeat_offset_jump;
             repeat_offset += repeat_offset_jump) {
            SpirvReg src_reg;
            SpirvReg dest_reg;
            uint32_t src_comp_offset;
            uint32_t dest_comp_offset;

            if (!src_regs.find_reg_at(inst.opr.src1.num + repeat_offset, src_reg, src_comp_offset)) {
                LOG_WARN("Register with num {} (src1) not found.", log_hex(inst.opr.src1.num));
                return false;
            }
            if (!dest_regs.find_reg_at(inst.opr.dest.num + repeat_offset, dest_reg, dest_comp_offset)) {
                LOG_WARN("Register with num {} (dest) not found.", log_hex(inst.opr.dest.num));
                return false;
            }

            const auto src_var_id = src_reg.var_id;
            const auto dest_var_id = dest_reg.var_id;

            uint32_t shuffle_components[4];
            for (auto c = 0; c < 4; ++c) {
                auto cycling_dest_mask_offset = (c + (uint8_t)repeat_offset) % 4;

                if (dest_mask.val & (1 << cycling_dest_mask_offset)) {
                    // use modified, source component
                    shuffle_components[c] = (uint32_t)src_swz[c];
                } else {
                    // use original, dest component
                    shuffle_components[c] = 4 + c;
                }
            }

            auto shuffle = m_b.createOp(spv::OpVectorShuffle, spv_vec4,
                { src_var_id, dest_var_id, shuffle_components[0], shuffle_components[1], shuffle_components[2], shuffle_components[3] });

            m_b.createStore(shuffle, dest_var_id);
        }

        return true;
    }

	bool vpck(ExtPredicate pred, bool skipinv, bool nosched, Imm1 src2_bank_sel, bool syncstart, Imm1 end, Imm1 src1_ext_bank, Imm2 src2_ext_bank, RepeatCount repeat_count, Imm3 src_fmt, Imm3 dest_fmt, DestinationMask dest_mask, Imm2 src1_bank_sel, Imm5 dest_n, Imm2 comp_sel_3, Imm1 scale, Imm2 comp_sel_1, Imm2 comp_sel_2, Imm5 src1_n, Imm1 comp0_sel_bit1, Imm4 src2_n, Imm1 comp_sel_0_bit0) {
        Instruction inst{};

        LOG_DISASM("{:016x}: {}{}", m_instr, Disasm::e_predicate_str(pred), "VPCK");
        return true;
    }

    bool special(bool special, SpecialCategory category) {
        usse::instr_idx--;
        usse::instr_idx--; // TODO: Remove?
        LOG_DISASM("{:016x}: SPEC category: {}, special: {}", m_instr, (uint8_t)category, special);
        return false;
    }
    bool special_phas() {
        usse::instr_idx--;
        LOG_DISASM("{:016x}: PHAS", m_instr);
        return false;
    }

private:
    // SPIR-V emitter
    spv::Builder &m_b;

    // Instruction word being translated
    const uint64_t &m_instr;

    // SPIR-V IDs
    const SpirvShaderParameters &m_spirv_params;

    // Shader program type
    emu::SceGxmProgramType m_program_type;
};

} // namespace usse
} // namespace shader

//
// Decoder entry
//
#include <shader/decoder_detail.h>
#include <shader/matcher.h>

namespace shader {
namespace usse {

template <typename Visitor>
using USSEMatcher = shader::decoder::Matcher<Visitor, uint64_t>;

template <typename V>
boost::optional<const USSEMatcher<V> &> DecodeUSSE(uint64_t instruction) {
    static const std::vector<USSEMatcher<V>> table = {
    // clang-format off
#define INST(fn, name, bitstring) shader::decoder::detail::detail<USSEMatcher<V>>::GetMatcher(fn, name, bitstring)

        // Vector ops (except MAD)
        /*
            MISC: p = predicate, s = skipinv, y = syncstart, n = nosched, 
            TYPE: t = op2
            DEST: u dest, b = dest bank sel, x = dest bank ext, w = dest mask
            SRC1: i = src1, g = src1 bank sel, h = src1 bank ext, z = src1 mod, k = src2 swizzle p1
            SRC1 SWIZZLE: k = src2 swizzle 0-6, v = src2 swizzle 7-8, c = src2 swizzle 9, m = src2 swizzle 10-11
            SRC2: j = src2, l = src2 bank sel, f = src2 bank ext, r = src2 mod (abs)
            SRC2 SWIZZLE: o = src2 swizzle
        */
        INST(&V::vnmad, "VNMADF32 ()",       "00001pppsmmyxchfoooonwwwwzzrvvbbgglluuuuuukkkkkkktttiiiiiijjjjjj"),
        INST(&V::vnmad, "VNMADF16 ()",       "00010pppsmmyxchfoooonwwwwzzrvvbbgglluuuuuukkkkkkktttiiiiiijjjjjj"),

		// Vector move
		/*
            MISC: p = predicate, s = skipinv, y = syncstart, e = end / src0 ext bank (COND ONLY), n = nosched, r = repeat
            TYPE: t = move type, z = move data type
            TEST: c = test bit 1, o = test bit 2
            DEST: u = dest, b = dest bank sel, x = dest bank ext, w = dest mask
            SRC:  q = src swizzle
            SRC0 (COND ONLY): d = src0, v = src0 bank sel, a = src0 comp sel
            SRC1: i = src1, g = src1 bank sel, h = src1 ext bank
            SRC2 (COND ONLY): j = src2, l = src2 bank sel, f = src2 ext bank
		*/
        INST(&V::vmov, "VMOV ()",            "00111pppsolyxehfttrrnzzzcqqqqvbbggaawwwwuuuuuuddddddiiiiiijjjjjj"),

		// Vector pack/unpack
		/*
		    MISC: p = predicate, r = repeat, k = skipinv, y = syncstart, e = end, n = nosched, c = scale
            FMT: s = src fmt, d = dest fmt
			DEST: u = dest, w = dest mask
            SRC:  q = comp sel 0 bit 0 / comp0 sel bit0 (COND1 ONLY), m = comp sel 1, b = comp sel 2, v = comp sel 3
			SRC1: f = src1 (COND2 ONLY), g = src1 bank sel, j = src1 ext bank
			SRC (COND1 ONLY): h = comp0 sel bit1
            SRC2 (COND1 ONLY): k = src2 / comp sel 0 bit 1 (NOT COND1) , o = src2 bank sel, t = src2 ext bank
			COND1: if src is f32
			COND2: if src is f32, f16 or c10
		*/
		INST(&V::vpck, "VPCK ()",            "01000pppknoy-ejt-rrrsssdddwwww--gg-uuuuu--vvcmm-bbffffffh--kkkkq"),

        // Special
        // s = special, c = category
		INST(&V::special_phas, "PHAS ()",    "11111----100----------------------------------------------------"),
		INST(&V::special, "SPEC ()",         "11111----scc----------------------------------------------------"),

        // clang-format on
    };
#undef INST

    const auto matches_instruction = [instruction](const auto &matcher) { return matcher.Matches(instruction); };

    auto iter = std::find_if(table.begin(), table.end(), matches_instruction);
    return iter != table.end() ? boost::optional<const USSEMatcher<V> &>(*iter) : boost::none;
}

} // namespace usse
} // namespace shader

//
// Decoder/translator usage
//
namespace shader {
namespace detail {

void convert_gxp_usse_to_spirv(spv::Builder &b, const SceGxmProgram &program, const SpirvShaderParameters &parameters, bool force_shader_debug) {
    // TODO:
    const uint64_t *code_ptr = program.get_code_start_ptr();
    const uint64_t instr_count = program.code_instr_count;
    const emu::SceGxmProgramType program_type = program.get_type();

    uint64_t instr;
    usse::USSETranslatorVisitor visitor(b, instr, parameters, program_type);

    for (auto instr_idx = 0; instr_idx < instr_count; ++instr_idx) {
        instr = code_ptr[instr_idx];

        //LOG_DEBUG("instr: 0x{:016x}", instr);

        usse::instr_idx = instr_idx;

        auto decoder = usse::DecodeUSSE<usse::USSETranslatorVisitor>(instr);
        if (decoder)
            decoder->call(visitor, instr);
        else
            LOG_DISASM("{:016x}: error: instruction unmatched", instr);
    }
}

} // namespace detail
} // namespace shader

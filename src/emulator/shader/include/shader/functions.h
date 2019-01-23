#pragma once

#include <shader/types.h>
#include <gxm/types.h>
#include <psp2/gxm.h>

#include <string>

namespace spv { class Builder; }

namespace shader {
namespace detail {

void convert_gxp_usse_to_spirv(spv::Builder &b, const SceGxmProgram &program, const SpirvShaderParameters &parameters, bool force_shader_debug);

} // namespace detail

std::string convert_gxp_to_glsl(const SceGxmProgram &program, const std::string &shader_hash, bool force_shader_debug = false);
void convert_gxp_to_glsl_from_filepath(const std::string &shader_filepath);

} // namespace shader

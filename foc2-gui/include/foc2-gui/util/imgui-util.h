#pragma once

#include <cstdint>
#include <imgui.h>
#include <string>

namespace ImGui {
    constexpr ImU32 ImColor(const uint8_t r, const uint8_t g, const uint8_t b, const uint8_t a) noexcept {
        return static_cast<ImU32>(a) << IM_COL32_A_SHIFT |
            static_cast<ImU32>(b) << IM_COL32_B_SHIFT |
            static_cast<ImU32>(g) << IM_COL32_G_SHIFT |
            static_cast<ImU32>(r) << IM_COL32_R_SHIFT;
    }

    inline ImVec2 CalcTextSize(const std::string& text, const bool hide_text_after_double_hash = false, const float wrap_width = -1.0f) {
        return CalcTextSize(text.c_str(), text.c_str() + text.length(), hide_text_after_double_hash, wrap_width);
    }
};

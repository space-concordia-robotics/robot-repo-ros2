#pragma once

#include <imgui.h>
#include <string>
#include <fmt/format.h>

namespace ImGui {
    constexpr ImU32 ImColor(const uint8_t red, const uint8_t green, const uint8_t blue, const uint8_t alpha) noexcept {
        return static_cast<ImU32>(alpha) << IM_COL32_A_SHIFT |
            static_cast<ImU32>(blue) << IM_COL32_B_SHIFT |
            static_cast<ImU32>(green) << IM_COL32_G_SHIFT |
            static_cast<ImU32>(red) << IM_COL32_R_SHIFT;
    }

    template <typename S = std::string>
    ImVec2 CalcTextSize(const S& text, const bool hide_text_after_double_hash = false, const float wrap_width = -1.0f)
        requires (std::is_same_v<std::decay_t<S>, std::string> || std::is_same_v<std::decay_t<S>, std::string_view>) {
        return CalcTextSize(text.data(), text.data() + text.length(), hide_text_after_double_hash, wrap_width);
    }

    // region {fmt} Formatting

    template <typename... T>
    void TextFmt(fmt::format_string<T...> fmt, T&&... args) { // NOLINT(*-missing-std-forward)
        const auto result = fmt::vformat(fmt, fmt::make_format_args(args...));
        TextUnformatted(result.data(), result.data() + result.length());
    }

    template <typename... T>
    void TextColoredFmt(fmt::format_string<T...> fmt, T&&... args) { // NOLINT(*-missing-std-forward)
        const auto result = fmt::vformat(fmt, fmt::make_format_args(args...));
        TextColoredUnformatted(result.data(), result.data() + result.length());
    }

    template <typename... T>
    void TextDisabledFmt(fmt::format_string<T...> fmt, T&&... args) { // NOLINT(*-missing-std-forward)
        const auto result = fmt::vformat(fmt, fmt::make_format_args(args...));
        TextDisabledUnformatted(result.data(), result.data() + result.length());
    }

    template <typename... T>
    void TextWrappedFmt(fmt::format_string<T...> fmt, T&&... args) { // NOLINT(*-missing-std-forward)
        const auto result = fmt::vformat(fmt, fmt::make_format_args(args...));
        TextWrappedUnformatted(result.data(), result.data() + result.length());
    }

    template <typename S = std::string, typename... T>
    void LabelTextFmt(const S& label, fmt::format_string<T...> fmt, T&&... args) // NOLINT(*-missing-std-forward)
        requires (std::is_same_v<std::decay_t<S>, std::string> || std::is_same_v<std::decay_t<S>, std::string_view>) {
        const auto result = fmt::vformat(fmt, fmt::make_format_args(args...));
        LabelTextUnformatted(label.data(), result.data(), result.data() + result.length());
    }

    template <typename... T>
    void BulletTextFmt(fmt::format_string<T...> fmt, T&&... args) { // NOLINT(*-missing-std-forward)
        const auto result = fmt::vformat(fmt, fmt::make_format_args(args...));
        BulletTextUnformatted(result.data(), result.data() + result.length());
    }

    template <typename... T>
    void SetTooltipFmt(fmt::format_string<T...> fmt, T&&... args) { // NOLINT(*-missing-std-forward)
        const auto result = fmt::vformat(fmt, fmt::make_format_args(args...));
        SetTooltipUnformatted(result.data(), result.data() + result.length());
    }

    template <typename... T>
    void SetItemTooltipFmt(fmt::format_string<T...> fmt, T&&... args) { // NOLINT(*-missing-std-forward)
        const auto result = fmt::vformat(fmt, fmt::make_format_args(args...));
        SetItemTooltipUnformatted(result.data(), result.data() + result.length());
    }

    // endregion

    // region Unformatted Text

    template <typename S = std::string>
    void TextUnformatted(const S& text)
        requires (std::is_same_v<std::decay_t<S>, std::string> || std::is_same_v<std::decay_t<S>, std::string_view>) {
        TextUnformatted(text.data(), text.data() + text.length());
    }

    template <typename S = std::string>
    void TextColoredUnformatted(const S& text)
        requires (std::is_same_v<std::decay_t<S>, std::string> || std::is_same_v<std::decay_t<S>, std::string_view>) {
        // %.*s is special-cased to avoid printf()
        TextColored("%.*s", text.data() + text.length(), text.data());
    }

    template <typename S = std::string>
    void TextDisabledUnformatted(const S& text)
        requires (std::is_same_v<std::decay_t<S>, std::string> || std::is_same_v<std::decay_t<S>, std::string_view>) {
        // %.*s is special-cased to avoid printf()
        TextDisabled("%.*s", text.data() + text.length(), text.data());
    }

    template <typename S = std::string>
    void TextWrappedUnformatted(const S& text)
        requires (std::is_same_v<std::decay_t<S>, std::string> || std::is_same_v<std::decay_t<S>, std::string_view>) {
        // %.*s is special-cased to avoid printf()
        TextWrapped("%.*s", text.data() + text.length(), text.data());
    }

    template <typename S1 = std::string, typename S2 = std::string>
    void LabelTextUnformatted(const S1& label, const S2& text)
        requires ((std::is_same_v<std::decay_t<S1>, std::string> || std::is_same_v<std::decay_t<S1>, std::string_view>) &&
            (std::is_same_v<std::decay_t<S2>, std::string> || std::is_same_v<std::decay_t<S2>, std::string_view>)) {
        // %.*s is special-cased to avoid printf()
        LabelText(label.data(), "%.*s", text.data() + text.length(), text.data());
    }

    template <typename S = std::string>
    void BulletTextUnformatted(const S& text)
        requires (std::is_same_v<std::decay_t<S>, std::string> || std::is_same_v<std::decay_t<S>, std::string_view>) {
        // %.*s is special-cased to avoid printf()
        BulletText("%.*s", text.data() + text.length(), text.data());
    }

    template <typename S = std::string>
    void SetTooltip(const S& text)
        requires (std::is_same_v<std::decay_t<S>, std::string> || std::is_same_v<std::decay_t<S>, std::string_view>) {
        // %.*s is special-cased to avoid printf()
        SetTooltip("%.*s", text.data() + text.length(), text.data());
    }

    template <typename S = std::string>
    void SetItemTooltip(const S& text)
        requires (std::is_same_v<std::decay_t<S>, std::string> || std::is_same_v<std::decay_t<S>, std::string_view>) {
        // %.*s is special-cased to avoid printf()
        SetItemTooltip("%.*s", text.data() + text.length(), text.data());
    }

    // endregion

    template <typename S = std::string>
    bool MenuItem(const S& label, std::optional<S&> shortcut, const bool selected = false, const bool enabled = true)
        requires (std::is_same_v<std::decay_t<S>, std::string> || std::is_same_v<std::decay_t<S>, std::string_view>) {
        return MenuItem(label.data(), shortcut.has_value() ? shortcut.value() : nullptr, selected, enabled);
    }

    template <typename S = std::string>
    bool MenuItem(const S& label, std::optional<S&> shortcut, bool* p_selected, const bool enabled = true)
        requires (std::is_same_v<std::decay_t<S>, std::string> || std::is_same_v<std::decay_t<S>, std::string_view>) {
        return MenuItem(label.data(), shortcut.has_value() ? shortcut.value() : nullptr, p_selected, enabled);
    }

    // NOLINTNEXTLINE(*-avoid-c-arrays)
    inline bool InputDouble2(const char* label, double v[2], const char* format = "%.3f", const ImGuiInputTextFlags flags = 0) {
        return InputScalarN(label, ImGuiDataType_Double, v, 2, nullptr, nullptr, format, flags);
    }

    // NOLINTNEXTLINE(*-avoid-c-arrays)
    inline bool InputDouble3(const char* label, double v[3], const char* format = "%.3f", const ImGuiInputTextFlags flags = 0) {
        return InputScalarN(label, ImGuiDataType_Double, v, 3, nullptr, nullptr, format, flags);
    }

    // NOLINTNEXTLINE(*-avoid-c-arrays)
    inline bool InputDouble4(const char* label, double v[4], const char* format = "%.3f", const ImGuiInputTextFlags flags = 0) {
        return InputScalarN(label, ImGuiDataType_Double, v, 4, nullptr, nullptr, format, flags);
    }

    inline bool SliderDouble(
        const char* label,
        double* value,
        const double v_min,
        const double v_max,
        const char* format = "%.3f",
        const ImGuiSliderFlags flags = 0
    ) {
        return SliderScalar(label, ImGuiDataType_Double, value, &v_min, &v_max, format, flags);
    }

    inline bool SliderDoubleSnapping(
        const char* label,
        double* value,
        const double v_min,
        const double v_max,
        const double threshold = 0.1,
        const char* format = "%.3f",
        const ImGuiSliderFlags flags = 0
    ) {
        const auto before = *value;
        const auto changed = SliderDouble(label, value, v_min, v_max, format, flags);

        if (before != *value) {
            if (const auto nearest = std::round(*value); std::abs(*value - nearest) <= threshold) {
                *value = nearest;
                return true;
            }
        }
        return changed;
    }

    inline bool SliderFloatSnapping(
        const char* label,
        float* value,
        const float v_min,
        const float v_max,
        const float threshold = 0.1,
        const char* format = "%.3f",
        const ImGuiSliderFlags flags = 0
    ) {
        const auto before = *value;
        const auto changed = SliderFloat(label, value, v_min, v_max, format, flags);

        if (before != *value) {
            if (const auto nearest = std::round(*value); std::abs(*value - nearest) <= threshold) {
                *value = nearest;
                return true;
            }
        }
        return changed;
    }
}

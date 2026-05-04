// MetricsOverlay.cpp
#include "metrics/MetricsOverlay.h"
 
#include <imgui.h>
#include <cstdio>
 
namespace destruct {
 
static void row(const char* label, const char* valueStr)
{
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::TextUnformatted(label);
    ImGui::TableSetColumnIndex(1);
    ImGui::TextUnformatted(valueStr);
}
 
void drawMetricsOverlay(const FractureMetrics& m,
                        const FractureMetrics::Thresholds& /*t*/,
                        bool settling)
{
    // Pin to top-right corner.
    const ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(
        ImVec2(io.DisplaySize.x - 10.0f, 10.0f),
        ImGuiCond_Always,
        ImVec2(1.0f, 0.0f));
    ImGui::SetNextWindowSize(ImVec2(420.0f, 0.0f));
    ImGui::SetNextWindowBgAlpha(0.80f);
 
    ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoDecoration      |
        ImGuiWindowFlags_AlwaysAutoResize  |
        ImGuiWindowFlags_NoSavedSettings   |
        ImGuiWindowFlags_NoFocusOnAppearing|
        ImGuiWindowFlags_NoNav             |
        ImGuiWindowFlags_NoMove;
 
    if (!ImGui::Begin("##metrics_overlay", nullptr, flags)) {
        ImGui::End();
        return;
    }
 
    ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.2f, 1.0f), "FRACTURE METRICS");
    ImGui::Separator();
 
    constexpr ImGuiTableFlags tableFlags =
        ImGuiTableFlags_SizingFixedFit |
        ImGuiTableFlags_NoHostExtendX;
 
    if (ImGui::BeginTable("metrics_table", 2, tableFlags)) {
        ImGui::TableSetupColumn("Metric", ImGuiTableColumnFlags_WidthFixed, 200.0f);
        ImGui::TableSetupColumn("Value",  ImGuiTableColumnFlags_WidthFixed, 200.0f);
 
        char buf[64];
 
        // -- Fragment --
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextDisabled("-- Fragment --");
 
        snprintf(buf, sizeof(buf), "%d / %d  (%.0f%%)",
                 m.actualFragments, m.requestedFragments,
                 m.yieldRate * 100.0f);
        row("Fragment yield", buf);
 
        snprintf(buf, sizeof(buf), "%.2f%%", m.volumeConservation * 100.0f);
        row("Volume conservation", buf);
 
        snprintf(buf, sizeof(buf), "%.2f%%", m.uniformityScore * 100.0f);
        row("Uniformity score", buf);
 
        // -- Pattern --
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextDisabled("-- Pattern --");
 
        snprintf(buf, sizeof(buf), "%.1f%%", m.convexityRatio * 100.0f);
        row("Convexity ratio", buf);
 
        snprintf(buf, sizeof(buf), "%.2f", m.meanAspectRatio);
        row("Mean aspect ratio", buf);
 
        snprintf(buf, sizeof(buf), "min %.3f  max %.3f", m.minVolume, m.maxVolume);
        row("Volume range", buf);
 
        snprintf(buf, sizeof(buf), "mean %.3f  sd %.3f", m.meanVolume, m.stddevVolume);
        row("Volume mean/sd", buf);
 
        // -- Physics --
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextDisabled("-- Physics --");
 
        snprintf(buf, sizeof(buf), settling ? "%.2f s ..." : "%.2f s", m.settlingTime);
        row("Settling time", buf);
 
        // Overlap error is updated every frame (even when paused), so always show it.
        snprintf(buf, sizeof(buf), "%.4f m", m.finalOverlapError);
        row("Final overlap error", buf);
 
        snprintf(buf, sizeof(buf), "%.2f J", m.peakKineticEnergy);
        row("Peak kinetic energy", buf);
 
        ImGui::EndTable();
    }
 
    // ---- Mini volume histogram -------------------------------------------
    if (!m.perFragment.empty()) {
        ImGui::Separator();
        ImGui::TextDisabled("Volume distribution");
 
        const float maxV   = m.maxVolume > 0.0f ? m.maxVolume : 1.0f;
        const float barW   = std::max(2.0f, (400.0f - 4.0f) / (float)m.perFragment.size());
        const float barMaxH = 40.0f;
 
        ImDrawList* dl    = ImGui::GetWindowDrawList();
        ImVec2      origin = ImGui::GetCursorScreenPos();
        origin.x += 2.0f;
 
        for (std::size_t i = 0; i < m.perFragment.size(); ++i) {
            float ratio = m.perFragment[i].volume / maxV;
            float h = ratio * barMaxH;
            float x0 = origin.x + (float)i * barW;
            float y0 = origin.y + barMaxH - h;
            float x1 = x0 + barW - 1.0f;
            float y1 = origin.y + barMaxH;
            ImU32 col = m.perFragment[i].isConvex
                        ? IM_COL32(100, 180, 255, 220)
                        : IM_COL32(220, 130, 40, 220);
            dl->AddRectFilled(ImVec2(x0, y0), ImVec2(x1, y1), col);
        }
 
        ImGui::Dummy(ImVec2(400.0f, barMaxH + 2.0f));
        ImGui::TextColored(ImVec4(0.4f, 0.7f, 1.0f, 1.0f), "convex");
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.9f, 0.5f, 0.1f, 1.0f), "non-convex");
    }
 
    ImGui::End();
}
 
} // namespace destruct
 
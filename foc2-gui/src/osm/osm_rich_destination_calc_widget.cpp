#include "foc2-gui/osm/osm_rich_destination_calc_widget.hpp"

#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>

#include "foc2-gui/osm/osm_coords.hpp"
#include "foc2-gui/osm/osm_rich_mark_storage.hpp"

namespace ImOsm::Rich {
    struct DestinationCalcWidget::Ui {
        std::string mark_name_a;
        GeographicLib::GeoCoords mark_a;
        GeographicLib::GeoCoords mark_b;
        float distance;
        float bearing;
    };

    DestinationCalcWidget::DestinationCalcWidget(const std::shared_ptr<MarkStorage>& storage) : ui(std::make_unique<Ui>()), storage(storage) {}

    DestinationCalcWidget::~DestinationCalcWidget() = default;

    void DestinationCalcWidget::paint() const {
        ImGui::PushID(this);
        ImGui::TextUnformatted("Destination Calculator");
        ImGui::SetNextItemWidth(100.f);
        ImGui::InputText("Mark A", &ui->mark_name_a);
        ImGui::SetNextItemWidth(100.f);
        ImGui::InputFloat("Distance [m]##", &ui->distance, {}, {}, "%.2f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(100.f);
        ImGui::InputFloat("Bearing [deg]##", &ui->bearing, {}, {}, "%.2f");
        ImGui::SameLine();
        if (ImGui::Button("Calculate##")) {
            bool found_a = false;
            ui->mark_a = storage->findMark(ui->mark_name_a, found_a);
            if (found_a) {
                ui->mark_b = destination(ui->mark_a, ui->distance, ui->bearing);
            }
        }

        ImGui::Text("Destination [deg] Lat: %.6f Lon: %.6f", ui->mark_b.Latitude(), ui->mark_b.Longitude());
        ImGui::SameLine();
        if (ImGui::Button("Pick##")) {
            storage->setPickCoords(ui->mark_b);
        }
        ImGui::PopID();
    }
}

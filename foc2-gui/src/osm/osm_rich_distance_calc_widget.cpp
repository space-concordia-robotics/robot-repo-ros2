#include "foc2-gui/osm/osm_rich_distance_calc_widget.hpp"

#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>

#include "foc2-gui/osm/osm_rich_mark_storage.hpp"
#include "foc2-gui/util/imgui_util.hpp"

namespace ImOsm::Rich {
    struct DistanceCalcWidget::Ui {
        std::string mark_name_a;
        std::string mark_name_b;
        GeographicLib::GeoCoords mark_a;
        GeographicLib::GeoCoords mark_b;
        GeographicLib::GeoCoords mark_c;
        double distance;
        double bearing;
    };

    DistanceCalcWidget::DistanceCalcWidget(const std::shared_ptr<MarkStorage>& storage)
        : ui(std::make_unique<Ui>()), storage(storage) {}

    DistanceCalcWidget::~DistanceCalcWidget() = default;

    void DistanceCalcWidget::paint() const {
        ImGui::PushID(this);
        ImGui::TextUnformatted("D/B/M Calculator");
        ImGui::SetNextItemWidth(100.f);
        ImGui::InputText("Mark A##", &ui->mark_name_a);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(100.f);
        ImGui::InputText("Mark B##", &ui->mark_name_b);
        ImGui::SameLine();
        if (ImGui::Button("Calculate##")) {
            bool foundA = false;
            bool foundB = false;
            ui->mark_a = storage->findMark(ui->mark_name_a, foundA);
            ui->mark_b = storage->findMark(ui->mark_name_b, foundB);
            if (foundA && foundB) {
                ui->distance = distance(ui->mark_a, ui->mark_b);
                ui->bearing = bearing(ui->mark_a, ui->mark_b);
                ui->mark_c = midpoint(ui->mark_a, ui->mark_b);
            }
        }
        if (ui->distance < 1e3) {
            ImGui::TextFmt("Distance [m] {:.2f}", ui->distance);
        } else {
            ImGui::TextFmt("Distance [km] {:.2f}", ui->distance * 1e-3);
        }
        ImGui::TextFmt("Bearing [deg] {:.2f}", ui->bearing);
        ImGui::TextFmt("Midpoint [deg] Lat: {:.6f} Lon: {:.6f}", ui->mark_c.Latitude(), ui->mark_c.Longitude());
        ImGui::SameLine();
        if (ImGui::Button("Pick##")) {
            storage->setPickCoords(ui->mark_c);
        }
        ImGui::PopID();
    }
}

#include "foc2-gui/osm/osm_rich_mark_item_widget.hpp"

#include <misc/cpp/imgui_stdlib.h>

#include "foc2-gui/osm/osm_rich_mark_item.hpp"
#include "foc2-gui/util/imgui_util.hpp"

namespace ImOsm::Rich {
    struct MarkItemWidget::Ui {
        static constexpr auto LAT_LON_FORMAT = "%.6f";
        static constexpr auto RADIUS_FORMAT = "%.0f";

        std::array<double, 2> latLon;
        double radius = 0.0;
        std::string text;
        std::string markerTypeName;
        ImPlotMarker markerType;
        bool textEnabled = false;
        bool markerEnabled = false;
        bool radiusEnabled;
        double markerSize = 0.0;
        double markerWeight = 0.0;
        double radiusWeight = 0.0;
        std::array<float, 3> markerFill;
    };

    static constexpr const char* MarkerTypeName(ImPlotMarker marker);

    MarkItemWidget::MarkItemWidget(const std::shared_ptr<MarkItem>& item, const GeographicLib::GeoCoords& picked_coords)
        : item(item),
          picked_coords(picked_coords),
          ui(std::make_unique<Ui>()) {
        ui->text = item->getText();
        ui->latLon[0] = item->getCoords().Latitude();
        ui->latLon[1] = item->getCoords().Longitude();
        ui->radius = item->getRadius();
        ui->textEnabled = item->getStyle().textEnabled;
        ui->markerEnabled = item->getStyle().markerEnabled;
        ui->radiusEnabled = item->getStyle().radiusEnabled;
        ui->radiusWeight = item->getStyle().radiusWeight;
        ui->markerType = item->getStyle().markerType;
        ui->markerTypeName = MarkerTypeName(ui->markerType);
        ui->markerSize = item->getStyle().markerSize;
        ui->markerWeight = item->getStyle().markerWeight;
        ui->markerFill = {
            item->getStyle().markerFill.x, item->getStyle().markerFill.y,
            item->getStyle().markerFill.z
        };
    }

    MarkItemWidget::~MarkItemWidget() = default;

    void MarkItemWidget::paint() const {
        ImGui::InputText("Name", &ui->text);
        ImGui::InputDouble2("Lat/Lon [deg]", ui->latLon.data(), ui->LAT_LON_FORMAT);
        ImGui::SameLine();
        paint_pickedBtn();
        ImGui::InputDouble("Radius [m]", &ui->radius, 0.0, 0.0, ui->RADIUS_FORMAT);
        ImGui::Separator();
        ImGui::TextUnformatted("Style");
        ImGui::Checkbox("Text Enabled", &ui->textEnabled);
        ImGui::SameLine();
        ImGui::Checkbox("Marker Enabled", &ui->markerEnabled);
        ImGui::SameLine();
        ImGui::Checkbox("Radius Enabled", &ui->radiusEnabled);
        ImGui::InputDouble("Radius Weight", &ui->radiusWeight, 1.0);
        paint_markerCombo();
        ImGui::InputDouble("Marker Size", &ui->markerSize, 1.0f);
        // ImGui::InputFloat("Marker Weight", &_markerWeight, 1.0f);
        ImGui::ColorEdit3("Marker Fill", ui->markerFill.data());
    }

    void MarkItemWidget::apply() const {
        item->setText(ui->text);
        item->setCoords({ui->latLon[0], ui->latLon[1]});
        item->setRadius(ui->radius);

        item->getStyle().textEnabled = ui->textEnabled;
        item->getStyle().markerEnabled = ui->markerEnabled;
        item->getStyle().radiusEnabled = ui->radiusEnabled;

        item->getStyle().radiusWeight = ui->radiusWeight;
        item->getStyle().markerType = ui->markerType;
        item->getStyle().markerSize = ui->markerSize;
        item->getStyle().markerWeight = ui->markerWeight;
        item->getStyle().markerFill = {
            ui->markerFill[0], ui->markerFill[1],
            ui->markerFill[2], 1.0
        };
    }

    void MarkItemWidget::paint_pickedBtn() const {
        if (ImGui::Button("Picked")) {
            ui->latLon = {picked_coords.Latitude(), picked_coords.Longitude()};
        }
    }

    void MarkItemWidget::paint_markerCombo() const {
        if (ImGui::BeginCombo("Marker", ui->markerTypeName.c_str())) {
            if (ImGui::Selectable(MarkerTypeName(ImPlotMarker_Circle), ui->markerType == ImPlotMarker_Circle)) {
                ui->markerType = ImPlotMarker_Circle;
                ui->markerTypeName = MarkerTypeName(ImPlotMarker_Circle);
            } else if (ImGui::Selectable(MarkerTypeName(ImPlotMarker_Square), ui->markerType == ImPlotMarker_Square)) {
                ui->markerType = ImPlotMarker_Square;
                ui->markerTypeName = MarkerTypeName(ImPlotMarker_Square);
            } else if (ImGui::Selectable(MarkerTypeName(ImPlotMarker_Diamond), ui->markerType == ImPlotMarker_Diamond)) {
                ui->markerType = ImPlotMarker_Diamond;
                ui->markerTypeName = MarkerTypeName(ImPlotMarker_Diamond);
            } else if (ImGui::Selectable(MarkerTypeName(ImPlotMarker_Up), ui->markerType == ImPlotMarker_Up)) {
                ui->markerType = ImPlotMarker_Up;
                ui->markerTypeName = MarkerTypeName(ImPlotMarker_Up);
            }
            ImGui::EndCombo();
        }
    }

    static constexpr const char* MarkerTypeName(const ImPlotMarker marker) {
        if (marker == ImPlotMarker_Circle) {
            return "Circle";
        } else if (marker == ImPlotMarker_Square) {
            return "Square";
        } else if (marker == ImPlotMarker_Diamond) {
            return "Diamond";
        } else if (marker == ImPlotMarker_Up) {
            return "Triangle UP";
        }
        return "Unknow";
    }
}

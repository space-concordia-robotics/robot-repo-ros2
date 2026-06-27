#include "foc2-gui/osm/osm_rich_mark_editor_widget.hpp"

#include <misc/cpp/imgui_stdlib.h>

#include "foc2-gui/osm/osm_rich_map_plot.hpp"
#include "foc2-gui/osm/osm_rich_mark_item.hpp"
#include "foc2-gui/osm/osm_rich_mark_item_widget.hpp"
#include "foc2-gui/util/imgui_util.hpp"

namespace ImOsm::Rich {
    struct MarkEditorWidget::Ui {
        static constexpr auto LAT_LON_FORMAT = "%.6f";
        static constexpr auto RADIUS_FORMAT_M = "%.1f [m]";
        static constexpr auto RADIUS_FORMAT_KM = "%.1f [km]";

        GeographicLib::GeoCoords lat_lon_input = GeographicLib::GeoCoords(0, 0);
        std::string mark_name_input_text;
        bool is_mouse_pick = false;
        bool is_mark_add = false;
    };

    MarkEditorWidget::MarkEditorWidget(const std::shared_ptr<RichMapPlot>& plot, const std::shared_ptr<MarkStorage>& storage)
        : plot(plot), storage(storage), ui(std::make_unique<Ui>()) {
        ui->mark_name_input_text.reserve(32);
    }

    MarkEditorWidget::~MarkEditorWidget() = default;

    void MarkEditorWidget::paint() {
        ImGui::TextUnformatted("Mark Editor");
        paintLatLonInput();
        ImGui::SameLine();
        paintMousePickBtn();
        paintMarkNameInput();
        ImGui::SameLine();
        paintAddMarkBtn();
        paintMarkTable();

        if (ui->is_mouse_pick && plot->mouseOnPlot() && ImGui::IsMouseClicked(0)) {
            ui->is_mouse_pick = false;
            ui->lat_lon_input = plot->mouse();
        }

        if (ui->is_mark_add) {
            ui->is_mark_add = false;
            const auto coords = ui->lat_lon_input;
            storage->addMark(coords, ui->mark_name_input_text);
            plot->addItem(storage->mark_items.back().ptr);
        }

        if (storage->handleLoadState()) {
            const auto& markItems = storage->markItems();
            std::ranges::for_each(
                markItems,
                [this](auto& item) {
                    plot->addItem(item.ptr);
                }
            );
        }

        if (storage->handlePickState()) {
            ui->lat_lon_input = GeographicLib::GeoCoords(storage->pick_coords.Latitude(), storage->pick_coords.Longitude());
        }
    }

    void MarkEditorWidget::paintLatLonInput() const {
        ImGui::PushItemWidth(200 + ImGui::GetStyle().ItemSpacing.x / 2);
        double data[] = {ui->lat_lon_input.Latitude(), ui->lat_lon_input.Longitude()};
        ImGui::InputDouble2("Lat/Lon", data, ui->LAT_LON_FORMAT);
        ImGui::PopItemWidth();
    }

    void MarkEditorWidget::paintMousePickBtn() const {
        if (!ui->is_mouse_pick) {
            if (ImGui::Button("Mouse Pick")) {
                ui->is_mouse_pick = !ui->is_mouse_pick;
            }
        } else {
            const auto color = ImGui::GetStyleColorVec4(ImGuiCol_ButtonHovered);
            ImGui::PushStyleColor(ImGuiCol_Button, color);
            if (ImGui::Button("Mouse Pick")) {
                ui->is_mouse_pick = !ui->is_mouse_pick;
            }
            ImGui::PopStyleColor();
        }
    }

    void MarkEditorWidget::paintMarkNameInput() const {
        ImGui::PushItemWidth(100);
        ImGui::InputText("Mark Name", &ui->mark_name_input_text);
        ImGui::PopItemWidth();
    }

    void MarkEditorWidget::paintAddMarkBtn() const {
        if (ImGui::Button("Add Mark")) {
            ui->is_mark_add = true;
        }
    }

    void MarkEditorWidget::paintMarkTable() {
        static constexpr int TABLE_COLUMNS = 6;
        static constexpr ImGuiTableColumnFlags COLUMN_FLAGS = ImGuiTableColumnFlags_WidthFixed;

        if (ImGui::BeginTable("MarkTabe", TABLE_COLUMNS)) {
            ImGui::TableSetupColumn("Name", COLUMN_FLAGS, 100);
            ImGui::TableSetupColumn("Lat", COLUMN_FLAGS, 100);
            ImGui::TableSetupColumn("Lon", COLUMN_FLAGS, 100);
            ImGui::TableSetupColumn("Radius", COLUMN_FLAGS, 100);
            ImGui::TableSetupColumn("Setup", COLUMN_FLAGS, 50);
            ImGui::TableSetupColumn("Delete", COLUMN_FLAGS, 50);
            ImGui::TableHeadersRow();

            storage->rmMarks();

            const auto& markItems = storage->markItems();
            std::ranges::for_each(markItems, [this](auto& item) {
                ImGui::TableNextRow();
                ImGui::PushID(item.ptr.get());
                paintMarkTableRow(item);
                ImGui::PopID();
            });

            ImGui::EndTable();
        }
    }

    void MarkEditorWidget::paintMarkTableRow(const MarkStorage::ItemNode& item) {
        // Name
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(item.ptr->getText().c_str());

        // Lat
        ImGui::TableNextColumn();
        ImGui::Text(ui->LAT_LON_FORMAT, item.ptr->getCoords().Latitude());

        // Lon
        ImGui::TableNextColumn();
        ImGui::Text(ui->LAT_LON_FORMAT, item.ptr->getCoords().Longitude());

        // Radius
        ImGui::TableNextColumn();
        if (item.ptr->getRadius() >= 1e3) {
            ImGui::Text(ui->RADIUS_FORMAT_KM, item.ptr->getRadius() * 1e-3);
        } else {
            ImGui::Text(ui->RADIUS_FORMAT_M, item.ptr->getRadius());
        }

        // Setup
        ImGui::TableNextColumn();
        if (ImGui::Button("Setup")) {
            ImGui::OpenPopup("Setup Item");
            item_widget = std::make_unique<MarkItemWidget>(item.ptr, ui->lat_lon_input);
        }

        if (ImGui::BeginPopupModal("Setup Item")) {
            // Draw popup contents
            item_widget->paint();
            ImGui::Separator();
            if (ImGui::Button("Cancel")) {
                ImGui::CloseCurrentPopup();
                item_widget.reset();
            }
            ImGui::SameLine();
            if (ImGui::Button("Apply")) {
                item_widget->apply();
                ImGui::CloseCurrentPopup();
                item_widget.reset();
            }
            ImGui::EndPopup();
        }

        // Delete
        ImGui::TableNextColumn();
        if (ImGui::Button("Delete")) {
            item.rm_flag = true;
        }
    }
}

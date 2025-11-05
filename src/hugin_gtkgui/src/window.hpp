// Copyright (c) Sensrad 2023
#pragma once

#include <gtkmm-3.0/gtkmm.h>

#include <iostream>

#include "rafgui_node.hpp"

#include "radar_statistics_frame.hpp"
#include "radar_status_frame.hpp"

#include "filter.hpp"
#include "info_bar.hpp"

class Window : public Gtk::Window {
  Glib::RefPtr<Gtk::Builder> builder_;
  std::shared_ptr<RAFGuiNode> rafgui_node_;

  // Widgets
  Gtk::Button *start_btn_ = nullptr;
  Gtk::Button *stop_btn_ = nullptr;
  Gtk::Button *set_time_btn_ = nullptr;
  Gtk::ComboBoxText *sequence_combo_ = nullptr;
  Gtk::Scale *static_sensitivity_scale_ = nullptr;
  Gtk::Scale *dynamic_azimuth_scale_ = nullptr;
  Gtk::Scale *dynamic_elevation_scale_ = nullptr;

  // Derived widgets
  RadarStatusFrame *radar_status_frame_ = nullptr;
  RadarStatisticsFrame *radar_statistics_frame_ = nullptr;
  InfoBar *info_bar_ = nullptr;

  // First order differential filter for frame rate smoothing.
  FirstOrderDiffFilter<double> frame_rate_filter_{/* alpha = */ 0.8};

  void on_sequence_changed() {
    // Allowed sequence types
    const std::unordered_map<std::string, RAFGuiNode::ESeuqenceType>
        sequence_types = {{"Idle", RAFGuiNode::IdleSeq},
                          {"Short", RAFGuiNode::FineShortSeq},
                          {"Mid", RAFGuiNode::FineMidSeq},
                          {"Long", RAFGuiNode::FineLongSeq},
                          {"Ultra Long", RAFGuiNode::FineUltraLongSeq}};

    const auto sequence = sequence_combo_->get_active_text();

    if (const auto it = sequence_types.find(sequence);
        it != sequence_types.end()) {
      rafgui_node_->set_sequence(it->second);
    } else {
      std::cerr << "Unknown sequence type: " << sequence << std::endl;
    }
  }

  // Signal callbacks
  void on_threshold_value_changed() {
    rafgui_node_->set_threshold(
        static_cast<float>(static_sensitivity_scale_->get_value()),
        static_cast<float>(dynamic_azimuth_scale_->get_value()),
        static_cast<float>(dynamic_elevation_scale_->get_value()));
  }

  void on_control_state(RAFGuiNode::ControlStateSharedPtr msg) {

    const std::unordered_map<RAFGuiNode::ESeuqenceType, std::string>
        sequence_strings = {{RAFGuiNode::IdleSeq, "Idle"},
                            {RAFGuiNode::FineShortSeq, "Short"},
                            {RAFGuiNode::FineMidSeq, "Mid"},
                            {RAFGuiNode::FineLongSeq, "Long"},
                            {RAFGuiNode::FineUltraLongSeq, "Ultra Long"}};

    const auto sequence_type =
        static_cast<RAFGuiNode::ESeuqenceType>(msg->active_seq.data);
    const auto sequence_name = sequence_strings.at(sequence_type);

    radar_status_frame_->set_timestamp(msg->header.stamp.sec,
                                       msg->header.stamp.nanosec);
    radar_status_frame_->set_tx_enabled(msg->tx_enabled);

    radar_status_frame_->set_sequence_name(sequence_name);
    radar_status_frame_->set_thresholds(msg->thresholds.static_threshold,
                                        msg->thresholds.dynamic_azimuth,
                                        msg->thresholds.dynamic_elevation);
  }

  void on_pointcloud(RAFGuiNode::PointCloud2SharedPtr msg) {
    // Estimate frame rate
    const double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    const double d_t = frame_rate_filter_.filter(t);
    // First sample will be 0, ignore that.
    if (d_t > 0) {
      radar_statistics_frame_->set_framerate(1.0 / d_t);
    }

    radar_statistics_frame_->set_timestamp(msg->header.stamp.sec,
                                           msg->header.stamp.nanosec);

    radar_statistics_frame_->set_target_count(msg->width);
  }

  void on_t11_pointcloud() {
    // std::cerr << "T11 pointcloud received" << std::endl;
  }

  void on_message(std::string &message) { info_bar_->set_message(message); }

public:
  Window(BaseObjectType *cobject, const Glib::RefPtr<Gtk::Builder> &builder,
         std::shared_ptr<RAFGuiNode> &rafgui_node)
      : Gtk::Window(cobject), builder_(builder), rafgui_node_(rafgui_node) {

    g_assert(rafgui_node_);

    // Connect signals
    // Start button
    builder_->get_widget("start_btn", start_btn_);
    g_assert(start_btn_);
    start_btn_->signal_clicked().connect(
        sigc::mem_fun(*rafgui_node, &RAFGuiNode::start_tx));

    // Stop button
    builder_->get_widget("stop_btn", stop_btn_);
    g_assert(stop_btn_);
    stop_btn_->signal_clicked().connect(
        sigc::mem_fun(*rafgui_node, &RAFGuiNode::stop_tx));

    // Set time button
    builder_->get_widget("set_time_btn", set_time_btn_);
    g_assert(set_time_btn_);
    set_time_btn_->signal_clicked().connect(
        sigc::mem_fun(*rafgui_node, &RAFGuiNode::set_time));

    // Sequence combo
    builder_->get_widget("sequence", sequence_combo_);
    g_assert(sequence_combo_);
    sequence_combo_->signal_changed().connect(
        sigc::mem_fun(this, &Window::on_sequence_changed));

    // Threshold scales
    builder_->get_widget("static_sensitivity", static_sensitivity_scale_);
    g_assert(static_sensitivity_scale_);
    builder_->get_widget("dynamic_azimuth", dynamic_azimuth_scale_);
    g_assert(dynamic_azimuth_scale_);
    builder_->get_widget("dynamic_elevation", dynamic_elevation_scale_);
    g_assert(dynamic_elevation_scale_);

    // Get derived widgets
    builder->get_widget_derived("info_bar", info_bar_);
    g_assert(info_bar_);

    builder_->get_widget_derived("radar_status_frame", radar_status_frame_);
    g_assert(radar_status_frame_);

    builder_->get_widget_derived("radar_statistics_frame",
                                 radar_statistics_frame_);
    g_assert(radar_statistics_frame_);

    Glib::RefPtr<Gdk::Pixbuf> iconPixbuf = Gdk::Pixbuf::create_from_resource(
        "/image/sensrad_eye_magenta_small.png");
    set_icon(iconPixbuf);

    // Connect them all to the same slot, they are always set at
    // the same time.
    static_sensitivity_scale_->signal_value_changed().connect(
        sigc::mem_fun(this, &Window::on_threshold_value_changed));
    dynamic_azimuth_scale_->signal_value_changed().connect(
        sigc::mem_fun(this, &Window::on_threshold_value_changed));
    dynamic_elevation_scale_->signal_value_changed().connect(
        sigc::mem_fun(this, &Window::on_threshold_value_changed));

    // Connect node signals
    rafgui_node_->signal_on_control_state().connect(
        sigc::mem_fun(*this, &Window::on_control_state));

    rafgui_node_->signal_on_pointcloud().connect(
        sigc::mem_fun(*this, &Window::on_pointcloud));

    rafgui_node_->signal_on_t11_pointcloud().connect(
        sigc::mem_fun(*this, &Window::on_t11_pointcloud));

    rafgui_node_->signal_on_message().connect(
        sigc::mem_fun(info_bar_, &InfoBar::set_message));
  }

  virtual ~Window(){};
};

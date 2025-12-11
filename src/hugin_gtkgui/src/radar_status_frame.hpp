#pragma once

#include <gtkmm-3.0/gtkmm.h>

#include <iostream>

class RadarStatusFrame : public Gtk::Frame {
  Glib::RefPtr<Gtk::Builder> builder_;

  Gtk::Label *status_timestamp_lbl = nullptr;
  Gtk::Label *status_tx_enabled_lbl = nullptr;
  Gtk::Label *status_sequence_lbl = nullptr;
  Gtk::Label *status_threshold_lbl = nullptr;
  Gtk::Label *status_dynamic_azimuth_lbl = nullptr;
  Gtk::Label *status_dynamic_elevation_lbl = nullptr;

protected:
  Glib::RefPtr<Gtk::Builder> builder() const;

public:
  RadarStatusFrame(BaseObjectType *cobject,
                   const Glib::RefPtr<Gtk::Builder> &builder)
      : Gtk::Frame(cobject), builder_(builder) {

    builder_->get_widget("status_timestamp", status_timestamp_lbl);
    g_assert(status_timestamp_lbl);
    builder_->get_widget("status_tx_enabled", status_tx_enabled_lbl);
    g_assert(status_tx_enabled_lbl);
    builder_->get_widget("status_sequence", status_sequence_lbl);
    g_assert(status_sequence_lbl);
    builder_->get_widget("status_threshold", status_threshold_lbl);
    g_assert(status_threshold_lbl);
    builder_->get_widget("status_dynamic_azimuth", status_dynamic_azimuth_lbl);
    g_assert(status_dynamic_azimuth_lbl);
    builder_->get_widget("status_dynamic_elevation",
                         status_dynamic_elevation_lbl);
    g_assert(status_dynamic_elevation_lbl);
  }

  void set_timestamp(const std::int32_t sec, const std::uint32_t nsec) {
    status_timestamp_lbl->set_text(
        Glib::ustring::sprintf("%d.%09d s", sec, nsec));
  }

  void set_tx_enabled(const bool enabled) {
    status_tx_enabled_lbl->set_text(enabled ? "Enabled" : "Disabled");
  }

  void set_sequence_name(const std::string &sequence_name) {
    status_sequence_lbl->set_text(sequence_name);
  }

  void set_thresholds(const float static_threshold,
                      const float dynamic_azimutyh,
                      const float dynamic_elevation) {

    status_threshold_lbl->set_text(
        Glib::ustring::sprintf("%.01f", static_threshold));
    status_dynamic_azimuth_lbl->set_text(
        Glib::ustring::sprintf("%.01f", dynamic_azimutyh));
    status_dynamic_elevation_lbl->set_text(
        Glib::ustring::sprintf("%.01f", dynamic_elevation));
  }

  virtual ~RadarStatusFrame() {}
};

#pragma once

#include <gtkmm-3.0/gtkmm.h>

#include <iostream>

class RadarStatisticsFrame : public Gtk::Frame {
  Glib::RefPtr<Gtk::Builder> builder_;

  Gtk::Label *statistics_timestamp_lbl = nullptr;
  Gtk::Label *statistics_framerate_lbl = nullptr;
  Gtk::Label *statistics_target_count_lbl = nullptr;

protected:
  Glib::RefPtr<Gtk::Builder> builder() const;

public:
  RadarStatisticsFrame(BaseObjectType *cobject,
                       const Glib::RefPtr<Gtk::Builder> &builder)
      : Gtk::Frame(cobject), builder_(builder) {
    builder_->get_widget("statistics_timestamp", statistics_timestamp_lbl);
    g_assert(statistics_timestamp_lbl);
    builder_->get_widget("statistics_framerate", statistics_framerate_lbl);
    g_assert(statistics_framerate_lbl);
    builder_->get_widget("statistics_target_count",
                         statistics_target_count_lbl);
    g_assert(statistics_target_count_lbl);
  }

  virtual ~RadarStatisticsFrame() {}

  void set_timestamp(const std::int32_t sec, const std::uint32_t nanosec) {
    statistics_timestamp_lbl->set_text(
        Glib::ustring::sprintf("%d.%09d s", sec, nanosec));
  }

  void set_framerate(const double fps) {
    statistics_framerate_lbl->set_text(Glib::ustring::sprintf("%.2f Hz", fps));
  }

  void set_target_count(const std::uint32_t target_count) {
    statistics_target_count_lbl->set_text(
        Glib::ustring::sprintf("%d", target_count));
  }
};

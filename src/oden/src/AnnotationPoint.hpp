// Copyright (c) Sensrad
#pragma once

#ifndef POINT_H_
#define POINT_H_

#include <cmath>
#include <cstdint>
#include <vector>

// Type definition enabling propagation of annotated data (which sometimes is
// present in bags
struct AnnotationPoint {
  int16_t annotation_cluster_idx;
  uint8_t annotation_class;

  AnnotationPoint(const int16_t annotation_cluster_idx,
                  const uint8_t annotation_class)
      : annotation_cluster_idx{annotation_cluster_idx}, annotation_class{
                                                            annotation_class} {}
};
typedef std::vector<AnnotationPoint> AnnotationPointCloud;

#endif // POINT_H_H

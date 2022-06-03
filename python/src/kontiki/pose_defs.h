//
// Created by tuomas on 2022-06-03.
//

#ifndef KONTIKI_POSE_DEFS_H
#define KONTIKI_POSE_DEFS_H

#include <boost/hana.hpp>
namespace hana = boost::hana;

#include "kontiki/sensors/pose.h"

namespace TC = kontiki::sensors;

// Define valid camera types
static constexpr auto pose_types = hana::tuple_t<
    TC::Pose
>;

#endif //KONTIKI_POSE_DEFS_H

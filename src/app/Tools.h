#pragma once
#include "app/Mission.h"
#include "util/Config.h"

int toolGenTags(const Config& cfg, const MissionOptions& opt);
int toolCalibColor(const Config& cfg, const MissionOptions& opt);
int toolCalibGround(const Config& cfg, const MissionOptions& opt);
int toolTelemetry(const Config& cfg, const MissionOptions& opt);

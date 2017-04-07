#pragma once

#include "State.h"

State RK4Integrate(	State &state,
					float t,
					float dt);

State FEIntegrate(	State &state,
					float t,
					float dt);
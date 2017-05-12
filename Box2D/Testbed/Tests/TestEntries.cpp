/*
* Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "../Framework/Test.h"
#include "../Framework/Render.h"
#include "../../freeglut/GL/glut.h"
#include <cstring>

#include "ThrottleControl.h"
#include "LateralControl.h"
#include "ThrottleLateralControl.h"
#include "PIDTuning.h"
#include "PIDTuning2.h"

TestEntry g_testEntries[] =
{
    {"Throttle", ThrottleControl::Create},
    {"Lateral", LateralControl::Create},
    {"Throttle + lateral", ThrottleLateralControl::Create},
    {"PID tuning", PIDTuning::Create},
    {"PID tuning 2", PIDTuning2::Create},
	{NULL, NULL}
};
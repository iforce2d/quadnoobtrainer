# Some flags for Freeglut and GLUI.
add_definitions( -DFREEGLUT_EXPORTS -DFREEGLUT_STATIC -D_CRT_SECURE_NO_WARNINGS )

# Define the framework files.
set(Testbed_Framework_SRCS
	Framework/Main.cpp
	Framework/Render.cpp
	Framework/Render.h
	Framework/Test.cpp
	Framework/Test.h
        Framework/jsoncpp.cpp
)

#define the test files.
set(Testbed_Tests_SRCS
        Tests/TestEntries.cpp
        Tests/quad.cpp
        Tests/trainersimTest.h
        Tests/ThrottleControl.h
        Tests/LateralControl.h
        Tests/ThrottleLateralControl.h
        Tests/PIDController.h
        Tests/PIDController.cpp
        Tests/PIDTuning.h
        Tests/PIDTuning2.h
        Tests/Calibration.h
        Tests/Calibration.cpp
)

# These are used to create visual studio folders.
source_group(Framework FILES ${Testbed_Framework_SRCS})
source_group(Tests FILES ${Testbed_Tests_SRCS})

include_directories (
	${OPENGL_INCLUDE_DIR}
	${Box2D_SOURCE_DIR}
)

if(APPLE)
	# We are not using the Apple's framework version, but X11's
	include_directories( /usr/X11/include )
	link_directories( /usr/X11/lib )
	set (OPENGL_LIBRARIES GL GLU GLUT X11)
endif(APPLE)

add_executable(Testbed
	${Testbed_Framework_SRCS}
	${Testbed_Tests_SRCS}
)

target_link_libraries (
	Testbed
	Box2D
	freeglut_static
        glui
	${OPENGL_LIBRARIES}
	
	# linux
	/home/chris/projects/budgetMiniQuad/quadnoobtrainer/Box2D/Testbed/libstem_gamepad/library/debug-linux64/libstem_gamepad.a

	# mac
        #/Users/chris/projects/budgetMiniQuad/quadnoobtrainer/Box2D/Testbed/libstem_gamepad/library/debug-macosx/libstem_gamepad.a
	#"-framework Foundation"
	#"-framework IOKit"
) 
